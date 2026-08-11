#include "display.h"
#include "zf_common_headfile.h"
#include "key_switch.h"

#define MERGE_MARK_HOLD_MS 1000U

// ================= [新增] 页面2 连通域详情模式 =================
// key2/key3 短按进入: 立即暂停两幅图刷新 (不拷贝图像), 拷贝 cam_down 及
// 关联 IMU 快照。进入后 key2 下翻 / key3 上翻 遍历所有连通域, 红色光标
// 标出当前连通域中心, 两幅图下方打印其分类 (最短文字):
//   [i/n]CAR A面积 R长宽比           — 小车
//   [i/n]T0..T2 A面积 R长宽比        — 信标 (按距离排序序号, 0起)
//   [i/n]-车链因/标链因 A面积 R长宽比 — 未认证点, 小车链/信标链各自的首个拦截原因
//        (面积/长宽比/距离等; lose=该链门槛全过但竞选落选)
// 第二行打印解算后的地面距离: 有小车 → 到小车; 无小车 → 到视野中心(图像中心投影)
// 第三行打印连通域中心像素坐标 (Row, Col), 供远场亮点标定读取
// key2 进入从头开始 / key3 进入从尾开始; key4 切走页面即退出,
// 再次切回页面2 完全恢复现有逻辑。
// 按键经 ISR 粘滞旗标送达 (key_switch.c), 不随20ms显示帧漏检;
// 画面仅在进入/切换选中时重绘一次, 平时保持静止, 避免每帧重传二值图造成闪烁。
#define BLOB_DETAIL_LINES 4               // 两幅图下方可用文字行数 (16*16 ~ 16*19)
#define BLOB_DETAIL_LINE_Y(k) (16 * (16 + (k)))

// 连通域分类
enum {
    BLOB_DETAIL_CLS_NONE = 0,             // 未认证
    BLOB_DETAIL_CLS_CAR,                  // 小车
    BLOB_DETAIL_CLS_TARGET                // 信标 (blob_detail_rank 存序号 0~2)
};

// 未认证原因 (与 image.c sort_lights 判定链一一对应, 最短文字)
enum {
    BLOB_DETAIL_REJ_NONE = 0,
    BLOB_DETAIL_REJ_AREA,                 // area  基础面积/距离过滤不达标
    BLOB_DETAIL_REJ_CAR_AREA,             // cArea 小车面积 < CAR_MIN_AREA
    BLOB_DETAIL_REJ_CAR_DIST,             // cDist 小车距离超限
    BLOB_DETAIL_REJ_CAR_RATIO,            // cRat  小车长宽比不符
    BLOB_DETAIL_REJ_CAR_EDGE,             // cEdge 小车越出安全边距
    BLOB_DETAIL_REJ_CAR_CENTER,           // cCtr  小车距画面中心过远
    BLOB_DETAIL_REJ_TGT_DIST,             // tDist 距小车距离超限
    BLOB_DETAIL_REJ_TGT_AREA,             // tArea 信标面积不足
    BLOB_DETAIL_REJ_TGT_RATIO,            // tRat  信标长宽比不符
    BLOB_DETAIL_REJ_LOST,                 // lose  通过全部门槛但竞选落选
};

static CameraObject blob_detail_snap;                 // cam_down 快照 (图像数据本身不拷贝)
static Image_IMU_Snapshot_t blob_detail_snap_imu;     // 关联 IMU 快照
static uint8_t blob_detail_cls[MAX_LIGHTS];           // 各连通域分类
static uint8_t blob_detail_rank[MAX_LIGHTS];          // target 序号 (0~2)
static uint8_t blob_detail_rej_car[MAX_LIGHTS];       // 未认证: 小车链首个拦截原因
static uint8_t blob_detail_rej_tgt[MAX_LIGHTS];       // 未认证: 信标链首个拦截原因
static uint8_t blob_detail_on = 0;                    // 详情模式激活标志
static uint8_t blob_detail_sel = 0;                   // 当前选中的连通域序号
static uint8_t blob_detail_dirty = 0;                 // 需要重绘标志 (进入/切换选中时置位)
static float blob_detail_dist[MAX_LIGHTS];            // 各连通域解算距离 (cm): 有小车→到小车, 无小车→到视野中心
static uint8_t blob_detail_has_car = 0;               // 距离参考点: 1=小车, 0=视野中心

void display_init()
{
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_color(RGB565_RED, RGB565_BLACK);
    ips200_init(IPS200_TYPE);
}
void display_motor_output_display()
{
  //电机输出
    ips200_show_string(0,16*2,"lf:");
    ips200_show_int(40,16*2,motor_out.lf, 4);
    ips200_show_string(120,16*2,"rf:");
    ips200_show_int(160,16*2,motor_out.rf, 4);
    ips200_show_string(0,16*3,"lb:");
    ips200_show_int(40,16*3,motor_out.lb, 4);
    ips200_show_string(120,16*3,"rb:");
    ips200_show_int(160,16*3,motor_out.rb, 4);
    
  
    ips200_show_string(0,16*4,"y:");
    ips200_show_float(40,16*4 , cam_down.centers[0][0], 3, 2); // [修改] 显示浮点数坐标
    ips200_show_string(120,16*4,"x:");
    ips200_show_float(160,16*4 , cam_down.centers[0][1], 3, 2); // [修改] 显示浮点数坐标
    
    ips200_show_string(0,16*5,"t_p:");
    ips200_show_float(40,16*5 , flight_target.target_pitch, 3,2);
    ips200_show_string(120,16*5,"t_r:");
    ips200_show_float(160,16*5 , flight_target.target_roll, 3,2);

    // //imu数据
    ips200_show_string(0,16*7,"Ro:");
    ips200_show_float(40,16*7 , imu_data.roll, 3,2);
    ips200_show_string(120,16*7,"Pi:");
    ips200_show_float(160,16*7 , imu_data.pitch, 3,2);
    ips200_show_string(0,16*8,"Ya:");
    ips200_show_float(40,16*8 , imu_data.yaw, 3,2);
    ips200_show_string(120,16*8,"z:");
    ips200_show_float(160,16*8 , imu_data.z, 3,2);
    
    ips200_show_string(0,16*9,"GR:");
    ips200_show_int(40,16*9 , (int)imu_data.groll, 4);
    ips200_show_string(120,16*9,"GP:");
    ips200_show_int(160,16*9 , (int)imu_data.gpitch, 4);
    ips200_show_string(0,16*10,"GY:");
    ips200_show_int(40,16*10 , (int)imu_data.gyaw, 4);

    ips200_show_string(0,16*11,"a_kp:");
    ips200_show_float(40,16*11 , pid_roll.kp, 4 , 2);
    ips200_show_string(120,16*11,"a_ki:");
    ips200_show_float(160,16*11 , pid_roll.ki, 4 , 2);
    ips200_show_string(0,16*12,"a_kp2:");
    ips200_show_float(40,16*12 , pid_roll.kp2, 4 , 2);
    ips200_show_string(120,16*12,"g_kp:");
    ips200_show_float(160,16*12 , pid_g_roll.kp, 4 , 2);
    ips200_show_string(0,16*13,"g_ki:");
    ips200_show_float(40,16*13 , pid_g_roll.ki, 4 , 2);
    ips200_show_string(120,16*13,"g_kd:");
    ips200_show_float(160,16*13 , pid_g_roll.kd, 4 , 2);

    ips200_show_string(0,16*15,"v_kp:");
    ips200_show_float(40,16*15 , pid_image_x.kp, 4 , 2);
    ips200_show_string(120,16*15,"v_ki:");
    ips200_show_float(160,16*15 , pid_image_x.ki, 4 , 2);
    ips200_show_string(0,16*16,"v_kd:");
    ips200_show_float(40,16*16 , pid_image_x.kd, 4 , 2);
    ips200_show_string(120,16*16,"v_kp2:");
    ips200_show_float(160,16*16 , pid_image_x.kp2, 4 , 2);
    
    ips200_show_string(0,16*17,"x:");
    ips200_show_float(40,16*17, share_data_from_1[S1_K_CAR_X], 4, 2);
    ips200_show_string(120,16*17,"y:");
    ips200_show_float(160,16*17, share_data_from_1[S1_K_CAR_Y], 4, 2);
}

void display_image_display(void){
    ips200_show_string(0,16*1,"x:");
    ips200_show_float(40,16*1, share_data_from_1[S1_CAR_CENTER_X], 4, 2);
    ips200_show_string(120,16*1,"y:");
    ips200_show_float(160,16*1, share_data_from_1[S1_CAR_CENTER_Y], 4, 2);

    ips200_show_string(0,16*3,"cx:");
    ips200_show_float(40,16*3, share_data_from_1[S1_CAR_RAW_X], 4, 2);
    ips200_show_string(120,16*3,"cy:");
    ips200_show_float(160,16*3, share_data_from_1[S1_CAR_RAW_Y], 4, 2);
    ips200_show_string(0,16*4,"tx:");
    ips200_show_float(40,16*4, share_data_from_1[S1_TARGET_X], 4, 2);
    ips200_show_string(120,16*4,"ty:");
    ips200_show_float(160,16*4, share_data_from_1[S1_TARGET_Y], 4, 2);

    ips200_show_string(0,16*5,"rx:");
    ips200_show_float(40,16*5, share_data_from_1[S1_RAW_TARGET_X], 4, 2);
    ips200_show_string(120,16*5,"ry:");
    ips200_show_float(160,16*5, share_data_from_1[S1_RAW_TARGET_Y], 4, 2);
    ips200_show_string(0,16*6,"z:");
    ips200_show_float(40,16*6, share_data_from_1[S1_K_CAR_X], 4, 2);

    ips200_show_string(0,16*7,"d:");
    ips200_show_float(40,16*7, share_data_from_1[S1_CAR_TARGET_DIST], 4, 2);
    ips200_show_string(120,16*7,"y:");
    ips200_show_float(160,16*7, share_data_from_1[11], 4, 2); // reserved
    ips200_show_string(0,16*8,"z:");
    ips200_show_float(40,16*8, share_data_from_1[S1_K_CAR_Y], 4, 2);

    ips200_show_string(0,16*9,"Ro:");
    ips200_show_float(40,16*9 , imu_data.roll, 3,2);
    ips200_show_string(120,16*9,"Pi:");
    ips200_show_float(160,16*9 , imu_data.pitch, 3,2);
    ips200_show_string(0,16*10,"Ya:");
    ips200_show_float(40,16*10 , imu_data.yaw, 3,2);
    ips200_show_string(120,16*10,"z:");
    ips200_show_float(160,16*10 , imu_data.z, 3,2);

    ips200_show_string(0,16*12,"di:");
    ips200_show_float(40,16*12 , share_data_from_1[S1_CAR_TARGET_DIST], 4,2);
}

// 冻结时对快照做一次分类: 复现 sort_lights 的判定链, 给每个连通域标定
// car / target[0..2] / 未认证(附首个拦截原因)。car/target 归属按存储结果
// (car_center / target_centers) 精确匹配, 保证与算法实际输出一致。
static void blob_detail_classify(void) {
    CameraObject *cam = &blob_detail_snap;
    uint8_t n = cam->light_number;
    if (n > MAX_LIGHTS) n = MAX_LIGHTS;

    for (int i = 0; i < MAX_LIGHTS; i++) {
        blob_detail_cls[i] = BLOB_DETAIL_CLS_NONE;
        blob_detail_rank[i] = 0;
        blob_detail_rej_car[i] = BLOB_DETAIL_REJ_NONE;
        blob_detail_rej_tgt[i] = BLOB_DETAIL_REJ_NONE;
    }

    // 物理距离/地面坐标 (与 sort_lights 第2步一致)
    float height = blob_detail_snap_imu.height;
    if (height < HEIGHT_ESTIMATE_MIN) height = HEIGHT_ESTIMATE_MIN;
    float car_plane_scale = (height - CAR_LIGHT_HEIGHT_CM) / height;
    float car_plane_scale_sq = car_plane_scale * car_plane_scale;

    float phys_dist_sq[MAX_LIGHTS] = {0};
    float ground_x[MAX_LIGHTS] = {0};
    float ground_y[MAX_LIGHTS] = {0};
    uint8_t valid[MAX_LIGHTS] = {0};

    for (int i = 0; i < n; i++) {
        double ox, oy, od;
        get_accurate_ground_distance((double)cam->centers[i][1], (double)cam->centers[i][0],
                                     (double)height, (double)blob_detail_snap_imu.pitch,
                                     (double)blob_detail_snap_imu.roll, &ox, &oy, &od);
        ground_x[i] = (float)ox;
        ground_y[i] = (float)oy;
        phys_dist_sq[i] = (float)(od * od);

        float area_for_filter = (float)cam->dot_num[i];
        if (od > DIST_COMP_THRESHOLD) area_for_filter *= (float)(od / DIST_COMP_SCALE);
        valid[i] = (area_for_filter >= BASE_MIN_AREA) ? 1 : 0;
    }

    // 按存储结果锁定 car / target 归属 (值由 sort_lights 原样拷贝, 精确匹配)
    int car_idx = -1;
    if (cam->car_valid) {
        for (int i = 0; i < n; i++) {
            if (cam->centers[i][0] == cam->car_center_y && cam->centers[i][1] == cam->car_center_x) {
                car_idx = i;
                break;
            }
        }
    }

    // 小车平面坐标 (信标距离的参考点)
    float car_plane_x = 0.0f, car_plane_y = 0.0f;
    if (car_idx >= 0) {
        car_plane_x = ground_x[car_idx] * car_plane_scale;
        car_plane_y = ground_y[car_idx] * car_plane_scale;
    }

    // [新增] 各连通域解算距离 (cm): 有小车 → 到小车, 无小车 → 到视野中心。
    // 小车参考点与信标链 car_dist 同一度量 (car_plane); 视野中心 = 图像中心
    // (CAM_CX, CAM_CY) 经同一套投影解算到地面的点。
    double fov_x = 0.0, fov_y = 0.0, fov_d = 0.0;
    if (car_idx < 0) {
        get_accurate_ground_distance((double)CAM_CX, (double)CAM_CY,
                                     (double)height, (double)blob_detail_snap_imu.pitch,
                                     (double)blob_detail_snap_imu.roll, &fov_x, &fov_y, &fov_d);
    }
    blob_detail_has_car = (car_idx >= 0) ? 1 : 0;
    for (int i = 0; i < n; i++) {
        if (i == car_idx) {
            blob_detail_dist[i] = 0.0f;            // 小车自身 → 0
            continue;
        }
        float ref_x = (car_idx >= 0) ? car_plane_x : (float)fov_x;
        float ref_y = (car_idx >= 0) ? car_plane_y : (float)fov_y;
        float ddx = ground_x[i] - ref_x;
        float ddy = ground_y[i] - ref_y;
        blob_detail_dist[i] = sqrtf(ddx * ddx + ddy * ddy);
    }

    for (int i = 0; i < n; i++) {
        // 1. 小车
        if (i == car_idx) {
            blob_detail_cls[i] = BLOB_DETAIL_CLS_CAR;
            continue;
        }
        // 2. 信标候选 (按存储的排序结果)
        uint8_t matched = 0;
        for (uint8_t r = 0; r < cam->target_count && r < TARGET_CANDIDATE_COUNT; r++) {
            if (cam->centers[i][0] == cam->target_centers[r][0] &&
                cam->centers[i][1] == cam->target_centers[r][1]) {
                blob_detail_cls[i] = BLOB_DETAIL_CLS_TARGET;
                blob_detail_rank[i] = r;
                matched = 1;
                break;
            }
        }
        if (matched) continue;

        // 3. 未认证: 小车链/信标链各自独立判定 (与 sort_lights 两条独立链一致),
        //    分别记录各自的首个拦截原因; 该链门槛全过但竞选落选记为 lose。
        float dx = cam->centers[i][1] - CAM_CX;
        float dy = cam->centers[i][0] - CAM_CY;
        float dist_sq = dx * dx + dy * dy;

        if (!valid[i]) {                       // 基础面积/距离过滤, 两链共同前置
            blob_detail_rej_car[i] = BLOB_DETAIL_REJ_AREA;
            blob_detail_rej_tgt[i] = BLOB_DETAIL_REJ_AREA;
        } else {
            // --- 小车链 (与 sort_lights 第1步一致) ---
            if (cam->dot_num[i] < CAR_MIN_AREA) {
                blob_detail_rej_car[i] = BLOB_DETAIL_REJ_CAR_AREA;
            } else if (phys_dist_sq[i] * car_plane_scale_sq > CAR_MAX_DISTANCE * CAR_MAX_DISTANCE) {
                blob_detail_rej_car[i] = BLOB_DETAIL_REJ_CAR_DIST;
            } else {
                float dyn_car_min_ratio = CAR_BASE_MIN_RATIO + dist_sq * CAR_RATIO_COMP_COEF;
                if (cam->aspect_ratio[i] <= dyn_car_min_ratio || cam->aspect_ratio[i] >= CAR_ABSOLUTE_MAX_RATIO) {
                    blob_detail_rej_car[i] = BLOB_DETAIL_REJ_CAR_RATIO;
                } else if (cam->centers[i][1] <= EDGE_SAFE_MARGIN_X || cam->centers[i][1] >= cam->width - EDGE_SAFE_MARGIN_X
                        || cam->centers[i][0] <= EDGE_SAFE_MARGIN_Y || cam->centers[i][0] >= cam->height - EDGE_SAFE_MARGIN_Y) {
                    blob_detail_rej_car[i] = BLOB_DETAIL_REJ_CAR_EDGE;
                } else if (dist_sq >= CAR_MAX_CENTER_DIST_SQ) {
                    blob_detail_rej_car[i] = BLOB_DETAIL_REJ_CAR_CENTER;
                } else {
                    blob_detail_rej_car[i] = BLOB_DETAIL_REJ_LOST;   // 小车链门槛全过, 竞选落选
                }
            }

            // --- 信标链 (与 sort_lights 第2步一致) ---
            float car_dist_sq = phys_dist_sq[i];
            if (car_idx >= 0) {
                float dxx = ground_x[i] - car_plane_x;
                float dyy = ground_y[i] - car_plane_y;
                car_dist_sq = dxx * dxx + dyy * dyy;
            }
            if (car_dist_sq > TARGET_MAX_DISTANCE * TARGET_MAX_DISTANCE) {
                blob_detail_rej_tgt[i] = BLOB_DETAIL_REJ_TGT_DIST;
            } else {
                float out_dist = sqrtf(phys_dist_sq[i]);
                float min_target_area = TARGET_AREA_BASE * (1.0f - out_dist / TARGET_AREA_FADE_DIST);
                if (min_target_area < 0.0f) min_target_area = 0.0f;
                if (cam->dot_num[i] <= min_target_area) {
                    blob_detail_rej_tgt[i] = BLOB_DETAIL_REJ_TGT_AREA;
                } else {
                    float dyn_tgt_max_ratio = TARGET_BASE_MAX_RATIO + dist_sq * TARGET_RATIO_COMP_COEF;
                    if (dyn_tgt_max_ratio > TARGET_LIMIT_MAX_RATIO) dyn_tgt_max_ratio = TARGET_LIMIT_MAX_RATIO;
                    if (cam->dot_num[i] > SMALL_BLOB_DIRECT_AREA && cam->aspect_ratio[i] >= dyn_tgt_max_ratio) {
                        blob_detail_rej_tgt[i] = BLOB_DETAIL_REJ_TGT_RATIO;
                    } else {
                        blob_detail_rej_tgt[i] = BLOB_DETAIL_REJ_LOST;   // 信标链门槛全过, 竞选落选
                    }
                }
            }
        }
    }
}

// 在二值图区 (y∈[y_min, y_max)) 内画红色十字光标 (半径4, 粗2)
static void blob_detail_draw_cursor(int16_t cx, int16_t cy, int16_t y_min, int16_t y_max) {
    for (int16_t offset = 0; offset <= 1; offset++) {
        int16_t y = cy + offset;
        if (y >= y_min && y < y_max) {
            int16_t x1 = cx - 4 < 0 ? 0 : cx - 4;
            int16_t x2 = cx + 4 >= MT9V03X_W ? MT9V03X_W - 1 : cx + 4;
            if (x1 <= x2) ips200_draw_line(x1, y, x2, y, RGB565_RED);
        }
        int16_t x = cx + offset;
        if (x >= 0 && x < MT9V03X_W) {
            int16_t y1 = cy - 4 < y_min ? y_min : cy - 4;
            int16_t y2 = cy + 4 >= y_max ? y_max - 1 : cy + 4;
            if (y1 <= y2) ips200_draw_line(x, y1, x, y2, RGB565_RED);
        }
    }
}

// 详情模式渲染: 重绘冻结的二值图 (image_copy 内容冻结不变) 以擦除上一帧光标,
// 再在选中连通域中心画红色光标, 两幅图下方打印该连通域的分类/原因。
static void blob_detail_render(void) {
    static const char *const rej_text[] = {
        "-",      // REJ_NONE
        "area",   // REJ_AREA       基础面积/距离过滤
        "cArea",  // REJ_CAR_AREA   小车面积
        "cDist",  // REJ_CAR_DIST   小车距离
        "cRat",   // REJ_CAR_RATIO  小车长宽比
        "cEdge",  // REJ_CAR_EDGE   小车边距
        "cCtr",   // REJ_CAR_CENTER 小车偏心
        "tDist",  // REJ_TGT_DIST   信标距离
        "tArea",  // REJ_TGT_AREA   信标面积
        "tRat",   // REJ_TGT_RATIO  信标长宽比
        "lose",   // REJ_LOST       全门槛通过但落选
    };

    // 清空整个文字区, 避免残留旧行
    for (uint8_t k = 0; k < BLOB_DETAIL_LINES; k++) {
        ips200_show_string(0, BLOB_DETAIL_LINE_Y(k), "                              ");
    }

    uint8_t n = blob_detail_snap.light_number;
    if (n > MAX_LIGHTS) n = MAX_LIGHTS;
    if (n == 0) {
        ips200_show_string(0, BLOB_DETAIL_LINE_Y(0), "no blob");
        return;
    }
    if (blob_detail_sel >= n) blob_detail_sel = n - 1;   // 防御越界
    uint8_t i = blob_detail_sel;

    // 重绘冻结的二值图, 擦除上一帧光标
    ips200_show_gray_image(0, MT9V03X_H + 16, image_copy[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);

    // 红色光标标出当前连通域中心 (二值图区)
    int16_t cx = (int16_t)blob_detail_snap.centers[i][1];
    int16_t cy = (int16_t)blob_detail_snap.centers[i][0] + MT9V03X_H + 16;
    blob_detail_draw_cursor(cx, cy, MT9V03X_H + 16, MT9V03X_H + 16 + MT9V03X_H);

    // 打印当前连通域的分类与原因 (未认证点: 车链因/标链因)
    char line[32];
    if (blob_detail_cls[i] == BLOB_DETAIL_CLS_CAR) {
        snprintf(line, sizeof(line), "[%d/%d]CAR A%u R%.1f", i, n, (unsigned)blob_detail_snap.dot_num[i], blob_detail_snap.aspect_ratio[i]);
    } else if (blob_detail_cls[i] == BLOB_DETAIL_CLS_TARGET) {
        snprintf(line, sizeof(line), "[%d/%d]T%d A%u R%.1f", i, n, blob_detail_rank[i], (unsigned)blob_detail_snap.dot_num[i], blob_detail_snap.aspect_ratio[i]);
    } else {
        snprintf(line, sizeof(line), "[%d/%d]-%s/%s A%u R%.1f", i, n, rej_text[blob_detail_rej_car[i]], rej_text[blob_detail_rej_tgt[i]], (unsigned)blob_detail_snap.dot_num[i], blob_detail_snap.aspect_ratio[i]);
    }
    line[30] = '\0';   // 每行最多30字符 (240px/8px), 防长文本溢出屏幕
    ips200_show_string(0, BLOB_DETAIL_LINE_Y(0), line);

    // [新增] 第二行: 解算后的距离 (有小车→到小车, 无小车→到视野中心)
    char dist_line[16];
    snprintf(dist_line, sizeof(dist_line), "D%.1fcm", blob_detail_dist[i]);
    ips200_show_string(0, BLOB_DETAIL_LINE_Y(1), dist_line);

    // [新增] 第三行: 连通域中心像素坐标 (Row, Col), 供远场标定读取
    char center_line[24];
    snprintf(center_line, sizeof(center_line), "R%.2f C%.2f",
             blob_detail_snap.centers[i][0], blob_detail_snap.centers[i][1]);
    ips200_show_string(0, BLOB_DETAIL_LINE_Y(2), center_line);
}

void display_image_debug_display(float car_x, float car_y, uint8_t car_valid, float target_x, float target_y, uint8_t target_valid){
    static uint8_t last_display_locked_state = 0;
    static uint8_t merge_mark_active = 0;
    static uint32_t merge_mark_start_ms = 0;
    uint8_t display_locked_state = (uint8_t)share_data_from_1[S1_LOCKED_COUNT];
    uint32_t now_ms = dataC.pit0_cnt;

    if (display_locked_state == 4 && last_display_locked_state != 4) {
        merge_mark_start_ms = now_ms;
        merge_mark_active = 1;
    }
    last_display_locked_state = display_locked_state;
    if (merge_mark_active && (uint32_t)(now_ms - merge_mark_start_ms) >= MERGE_MARK_HOLD_MS) {
        merge_mark_active = 0;
    }

    // ================= [新增] 页面2: 连通域详情模式状态机 =================
    {
        // 按键经 ISR 粘滞旗标送达 (key_switch.c), 避免20ms显示帧漏检10ms短按事件
        uint8_t k2_short = detail_btn_k2;
        detail_btn_k2 = 0;
        uint8_t k3_short = detail_btn_k3;
        detail_btn_k3 = 0;

        uint8_t blob_n = blob_detail_snap.light_number;
        if (blob_n > MAX_LIGHTS) blob_n = MAX_LIGHTS;

        if (display_page_idx != 2) {
            blob_detail_on = 0;                              // 离开页面2 → 复位 (再进入=现有逻辑)
        } else if (!blob_detail_on && (k2_short || k3_short)) {
            blob_detail_on = 1;                              // 进入详情模式并冻结快照
            blob_detail_dirty = 1;                           // 需要首帧绘制
            blob_detail_sel = k2_short ? 0 : (uint8_t)(blob_n > 0 ? blob_n - 1 : 0);  // key2进入从头 / key3进入从尾
            blob_detail_snap = cam_down;                     // 拷贝 cam_down (图像数据本身不拷贝)
            blob_detail_snap_imu = img_imu_snap;             // 拷贝关联 IMU 快照
            blob_detail_classify();
        } else if (blob_detail_on) {
            if (k2_short && blob_detail_sel + 1 < blob_n) { blob_detail_sel++; blob_detail_dirty = 1; }   // key2: 下翻
            if (k3_short && blob_detail_sel > 0) { blob_detail_sel--; blob_detail_dirty = 1; }            // key3: 上翻
        }
    }

    // 详情模式: 暂停两幅图刷新 (不拷贝图)。画面内容冻结不变,
    // 仅在进入/切换选中时重绘一次 (dirty), 平时保持静止, 不产生闪烁。
    if (blob_detail_on && display_page_idx == 2) {
        if (blob_detail_dirty) {
            blob_detail_render();
            blob_detail_dirty = 0;
        }
        return;
    }

    // 将底层的 0/1 放大为 0/255 以便屏幕显示
    for(int i = 0; i < MT9V03X_H * MT9V03X_W; i++) {
        image_copy[0][i] = cam_down.binarized_image[i] ? 255 : 0;
    }

    // 将要显示的数组刷入内存供 DMA 搬运
    SCB_CleanDCache_by_Addr((void*)image_copy, sizeof(image_copy));

    // 显示图像: 页面2上下分屏 (上:原始图像, 下:二值化图像)
    if(display_page_idx == 2)
    {
        ips200_show_gray_image(0, 0, cam_down.raw_image, MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
        ips200_show_gray_image(0, MT9V03X_H + 16, image_copy[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
    }
    else
    {
        // 显示二值化图像
        ips200_displayimage03x((const uint8 *)image_copy , MT9V03X_W, MT9V03X_H);
    }
    
    // [新增] 叠加显示小车和信标的十字准星
    // 检查小车是否有效
    if (car_valid) {
        int16_t cx = (int16_t)car_x;
        int16_t cy = (int16_t)car_y;
        
        int16_t radius = 4; // 长度扩大一倍
        for (int16_t offset = 0; offset <= 1; offset++) { // 粗细扩大一倍
            // 画水平线（带边缘截断）
            int16_t y = cy + offset;
            if (y >= 0 && y < MT9V03X_H) {
                int16_t x1 = cx - radius < 0 ? 0 : cx - radius;
                int16_t x2 = cx + radius >= MT9V03X_W ? MT9V03X_W - 1 : cx + radius;
                if (x1 <= x2) ips200_draw_line(x1, y, x2, y, RGB565_RED);
            }
            
            // 画垂直线（带边缘截断）
            int16_t x = cx + offset;
            if (x >= 0 && x < MT9V03X_W) {
                int16_t y1 = cy - radius < 0 ? 0 : cy - radius;
                int16_t y2 = cy + radius >= MT9V03X_H ? MT9V03X_H - 1 : cy + radius;
                if (y1 <= y2) ips200_draw_line(x, y1, x, y2, RGB565_RED);
            }
        }
    }

    // 检查信标是否有效
    if (target_valid) {
        int16_t tx = (int16_t)target_x;
        int16_t ty = (int16_t)target_y;

        int16_t radius = 4; // 长度扩大一倍
        for (int16_t offset = 0; offset <= 1; offset++) { // 粗细扩大一倍
            // 画水平线（带边缘截断）
            int16_t y = ty + offset;
            if (y >= 0 && y < MT9V03X_H) {
                int16_t x1 = tx - radius < 0 ? 0 : tx - radius;
                int16_t x2 = tx + radius >= MT9V03X_W ? MT9V03X_W - 1 : tx + radius;
                if (x1 <= x2) ips200_draw_line(x1, y, x2, y, RGB565_GREEN); // 改为绿色
            }
            
            // 画垂直线（带边缘截断）
            int16_t x = tx + offset;
            if (x >= 0 && x < MT9V03X_W) {
                int16_t y1 = ty - radius < 0 ? 0 : ty - radius;
                int16_t y2 = ty + radius >= MT9V03X_H ? MT9V03X_H - 1 : ty + radius;
                if (y1 <= y2) ips200_draw_line(x, y1, x, y2, RGB565_GREEN); // 改为绿色
            }
        }
    }

    // 标记第2、3候选信标；target_centers按[Row, Col]存储。
    for (uint8_t rank = 1; rank < cam_down.target_count && rank < TARGET_CANDIDATE_COUNT; rank++) {
        int16_t tx = (int16_t)cam_down.target_centers[rank][1];
        int16_t ty = (int16_t)cam_down.target_centers[rank][0];
        uint16_t color = rank == 1U ? RGB565_BLUE : RGB565_YELLOW;
        int16_t radius = 4;

        for (int16_t offset = 0; offset <= 1; offset++) {
            int16_t y = ty + offset;
            if (y >= 0 && y < MT9V03X_H) {
                int16_t x1 = tx - radius < 0 ? 0 : tx - radius;
                int16_t x2 = tx + radius >= MT9V03X_W ? MT9V03X_W - 1 : tx + radius;
                if (x1 <= x2) ips200_draw_line(x1, y, x2, y, color);
            }

            int16_t x = tx + offset;
            if (x >= 0 && x < MT9V03X_W) {
                int16_t y1 = ty - radius < 0 ? 0 : ty - radius;
                int16_t y2 = ty + radius >= MT9V03X_H ? MT9V03X_H - 1 : ty + radius;
                if (y1 <= y2) ips200_draw_line(x, y1, x, y2, color);
            }
        }
    }
    
    // 显示页面号 (页面2的指示器移到屏幕底部, 避免压住分屏图像)
    if(display_page_idx != 2)
    {
        ips200_show_string(160, 16*10, "P:");
        ips200_show_int(176, 16*10, display_page_idx, 1);
        ips200_show_string(160, 16*18, "M:");
        ips200_show_int(176, 16*18, merge_mark_active, 1);
    }

    // 页面0：核心状态
    if(display_page_idx == 0)
    {
        ips200_show_string(0, 16*10, "Thresh:");
        ips200_show_int(80, 16*10, cam_down.threshold_max, 3);

        ips200_show_string(0, 16*11, "Car A:");
        ips200_show_int(50, 16*11, cam_down.car_dot_num, 4);
        ips200_show_string(100, 16*11, "R:"); // Ratio
        ips200_show_float(120, 16*11, cam_down.car_ratio, 2, 2);

        ips200_show_string(0, 16*12, "Tgt A:");
        ips200_show_int(50, 16*12, target_valid ? cam_down.target_dot_num : 0, 4);
        ips200_show_string(100, 16*12, "R:"); 
        ips200_show_float(120, 16*12, target_valid ? cam_down.target_ratio : 0.0f, 2, 2);

        ips200_show_string(0, 16*13, "MaxR:");
        ips200_show_float(40, 16*13, cam_down.debug.max_ratio, 3, 2);
        ips200_show_string(90, 16*13, "MinR:");
        ips200_show_float(130, 16*13, cam_down.debug.min_ratio, 3, 2);

        ips200_show_string(0, 16*14, "H:");
        ips200_show_int(40, 16*14, (int)share_data_from_0[S0_IMU_HEIGHT], 4);
        
        ips200_show_string(0, 16*15, "CX:");
        ips200_show_float(40, 16*15, pos.raw_car.x, 4, 2);
        ips200_show_string(100, 16*15,"CY:");
        ips200_show_float(140, 16*15, pos.raw_car.y, 4, 2);

        ips200_show_string(0, 16*16, "TR:");
        ips200_show_float(40, 16*16, share_data_from_0[S0_TARGET_ROLL], 4, 2);
        ips200_show_string(100, 16*16, "TP:");
        ips200_show_float(140, 16*16, share_data_from_0[S0_TARGET_PITCH], 4, 2);

        ips200_show_string(0, 16*17, "iR:");
        ips200_show_float(40, 16*17, share_data_from_0[S0_IMU_ROLL], 4, 2);
        ips200_show_string(100, 16*17, "iP:");
        ips200_show_float(140, 16*17, share_data_from_0[S0_IMU_PITCH], 4, 2);
        ips200_show_string(0, 16*18, "iY:");
        ips200_show_float(40, 16*18, share_data_from_0[S0_IMU_YAW], 4, 2);
    }
    // 页面1：亮度与电机
    else if(display_page_idx == 1)
    {

        ips200_show_string(0, 16*14, "LF:");
        ips200_show_int(40, 16*14, (int)share_data_from_0[S0_MOTOR_LF], 4);
        ips200_show_string(100, 16*14, "RF:");
        ips200_show_int(140, 16*14, (int)share_data_from_0[S0_MOTOR_RF], 4);
        ips200_show_string(0, 16*15, "LB:");
        ips200_show_int(40, 16*15, (int)share_data_from_0[S0_MOTOR_LB], 4);
        ips200_show_string(100, 16*15, "RB:");
        ips200_show_int(140, 16*15, (int)share_data_from_0[S0_MOTOR_RB], 4);
    }
    // 页面2：上下分屏显示 原始图像(上) 与 二值化图像(下)
    else if(display_page_idx == 2)
    {
        // 清除旧页面可能残留的文字行 (30个空格 = 240px 整行)
        ips200_show_string(0, 16*16, "                              ");
        ips200_show_string(0, 16*17, "                              ");
        ips200_show_string(0, 16*18, "                              ");
        ips200_show_string(0, 16*19, "                              ");
        // 标注两幅图像 (RAW在图像间隙, BIN在底部文字区)
        ips200_show_string(0, MT9V03X_H, "RAW");
        ips200_show_string(0, 16*16, "BIN");
        // 页面号与合并标记
        ips200_show_string(160, 16*18, "P:");
        ips200_show_int(176, 16*18, display_page_idx, 1);
        ips200_show_string(160, 16*19, "M:");
        ips200_show_int(176, 16*19, merge_mark_active, 1);
    }
}
