#include "image.h"
#include "zf_common_headfile.h"
// --- 1. 内存分配 ---
// 定义二值化图像缓冲区 (只定义一个)
uint8_t buffer_bin_down[MT9V03X_H][MT9V03X_W];
// 定义 DFS 访问标记数组 (静态分配以防栈溢出)
uint8_t visited_buffer[MT9V03X_H * MT9V03X_W];
uint8 image_copy[MT9V03X_H][MT9V03X_W];
// 定义全局实例
CameraObject cam_down;
extern float m7_1_data[M7_1_DATA_LENGTH];

// --- 2. 初始化函数 ---
void camera_init(void) {
    while(1)
    {
        if(mt9v03x_init())
            gpio_toggle_level(P19_0);                                            // 翻转 LED 引脚输出电平 控制 LED 亮灭 初始化出错这个灯会闪的很慢
        else
            break;
        system_delay_ms(500);                                                   // 闪灯表示异常
    }
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_GRAY, image_copy[0], MT9V03X_W, MT9V03X_H);
    seekfree_assistant_camera_boundary_config(NO_BOUNDARY, 0 ,NULL , NULL ,NULL , NULL , NULL , NULL);
    // 初始化下视摄像头
    cam_down.width = MT9V03X_W;
    cam_down.height = MT9V03X_H;
    
    // 指向库文件的图像数组 (直接使用逐飞库的 DMA 缓冲区)
    cam_down.raw_image = (uint8_t *)mt9v03x_image;      
    cam_down.binarized_image = (uint8_t *)buffer_bin_down;
    
    // 参数配置
    cam_down.threshold = THRESHOLD;   // 二值化阈值 (需根据实际场地光照调整)
    cam_down.margin_cut = 1;    // 四周裁剪 5 像素
    cam_down.debug_max_ratio = 0.0f;
    cam_down.debug_min_ratio = 999.0f;
}

// --- 3. 内部辅助函数 ---
/**
 * @brief 简单估计目标在地面上的相对物理坐标（忽略机体倾角）
 * * @param u        目标在图像中的 Col (X像素)
 * @param v        目标在图像中的 Row (Y像素)
 * @param height   当前无人机的飞行高度 (cm)
 * @param out_x    输出: 目标相对于无人机正下方的【前向】距离 (cm)
 * @param out_y    输出: 目标相对于无人机正下方的【右向】距离 (cm)
 * @param out_dist 输出: 目标离机头正下方的直线总距离 (cm)
 */
void Estimate_Distance_Simple(float u, float v, float height, float *out_x, float *out_y, float *out_dist) {
    
    // 1. 计算以画面中心为原点的像素坐标
    // 图像 Row(v) 往下是正，但在物理世界前方是正，所以 Y 轴取反
    float px = u - CAM_CX;          // X轴：向右为正
    float py = -(v - CAM_CY);       // Y轴：向前为正

    // 2. 计算像素距离平方 (rho2) 和像素距离 (rho)
    float rho2 = px * px + py * py;
    float rho = sqrtf(rho2);

    // 3. 计算畸变多项式 Z 轴 (相当于该像素点处的“虚拟焦距”)
    // 公式: z = A0 + A2*rho^2 + A3*rho^3 + A4*rho^4
    float z_poly = CAM_A0 + CAM_A2 * rho2 + CAM_A3 * rho2 * rho + CAM_A4 * rho2 * rho2;

    // 4. 相似三角形投影计算比例系数 scale
    // 物理距离与像素距离的比例 = 当前高度 / 虚拟焦距
    float scale = height / z_poly;

    // 5. 算出具体的物理坐标和距离
    *out_x = py * scale;                     // 前向距离 (cm)
    *out_y = px * scale;                     // 右向距离 (cm)
    if (out_dist != NULL) {
        *out_dist = rho * scale;             // 目标到正下方的直线距离 (cm)
    }
}
// 用于保存单一连通域统计特征的结构体
typedef struct {
    uint32_t sum_r;
    uint32_t sum_c;
    uint32_t sum_rr;
    uint32_t sum_cc;
    uint32_t sum_rc;
    uint32_t dot_num;
} BlobStats;

// 深度优先搜索 (DFS) - 迭代版 (防止栈溢出)
// 【优化】同步累加计算质心和二阶矩所需的坐标数据，消除对图像的二次遍历
static void dfs_iterative(CameraObject *cam, uint8_t *visited, uint8_t label, uint16_t start_r, uint16_t start_c, BlobStats *stats) {
    typedef struct { uint16_t r; uint16_t c; } Node;
    static Node stack[STACK_SIZE]; // 静态局部栈
    int top = -1;

    // 初始化本连通域统计数据，先记录起点
    stats->sum_r = start_r;
    stats->sum_c = start_c;
    stats->sum_rr = (uint32_t)start_r * start_r;
    stats->sum_cc = (uint32_t)start_c * start_c;
    stats->sum_rc = (uint32_t)start_r * start_c;
    stats->dot_num = 1;

    // 起点入栈
    stack[++top] = (Node){start_r, start_c};
    // 压栈即标记，防止重复入栈
    visited[start_r * cam->width + start_c] = label;
    
    // 方向数组：上右下左
    const int dr[] = {-1, 0, 1, 0};
    const int dc[] = {0, 1, 0, -1};

    while (top >= 0) {
        Node curr = stack[top--];
        
        // 搜索 4 邻域
        for (int i = 0; i < 4; i++) {
            uint16_t nr = curr.r + dr[i];
            uint16_t nc = curr.c + dc[i];
            // 简单的边界预判
            if (nr >= cam->margin_cut && nr < (cam->height - cam->margin_cut) &&
                nc >= cam->margin_cut && nc < (cam->width - cam->margin_cut)) {
                
                uint32_t nidx = nr * cam->width + nc;
                // 只有亮点且未访问才入栈，减少栈占用
                if (cam->binarized_image[nidx] == 1 && visited[nidx] == 0) {
                    if (top < STACK_SIZE - 1) {
                        stack[++top] = (Node){nr, nc};
                        visited[nidx] = label; // 【核心修复】压栈时必须立刻标记
                        
                        // 同步累加该像素点的坐标和平方特征
                        stats->sum_r += nr; 
                        stats->sum_c += nc;
                        stats->sum_rr += (uint32_t)nr * nr; 
                        stats->sum_cc += (uint32_t)nc * nc; 
                        stats->sum_rc += (uint32_t)nr * nc;
                        stats->dot_num++;
                    }
                }
            }
        }
    }
}

// 【合并优化】一次遍历同时完成二值化与 8 邻域膨胀
static void binarize_and_dilate(CameraObject *cam) {
    // 清空二值化缓冲区
    memset(cam->binarized_image, 0, cam->width * cam->height);

    // 为了安全进行 8 邻域膨胀，确保最少有 1 个像素的物理边界
    uint16_t safe_margin = cam->margin_cut > 1 ? cam->margin_cut : 1;

    // 遍历图像 
    for (uint16_t r = safe_margin; r < cam->height - safe_margin; r++) {
        for (uint16_t c = safe_margin; c < cam->width - safe_margin; c++) {
            uint32_t idx = r * cam->width + c;
            
            // 阈值判断
            if (cam->raw_image[idx] > cam->threshold) {
                // 直接点亮自身与 8 邻域
                cam->binarized_image[idx] = 1;
                cam->binarized_image[idx - 1] = 1;
                cam->binarized_image[idx + 1] = 1;
                cam->binarized_image[idx - cam->width] = 1;
                cam->binarized_image[idx + cam->width] = 1;
                cam->binarized_image[idx - cam->width - 1] = 1;
                cam->binarized_image[idx - cam->width + 1] = 1;
                cam->binarized_image[idx + cam->width - 1] = 1;
                cam->binarized_image[idx + cam->width + 1] = 1;
            }
        }
    }
}

// 【合并优化】一次全图扫描，同时完成连通域提取、质心计算与二阶矩长宽比特征提取
static void extract_components(CameraObject *cam, uint8_t *visited) {
    // 清空历史状态
    memset(visited, 0, cam->width * cam->height);
    memset(cam->dot_num, 0, sizeof(cam->dot_num));
    memset(cam->centers, 0, sizeof(cam->centers));
    memset(cam->aspect_ratio, 0, sizeof(cam->aspect_ratio));

    uint8_t label = 1;
    uint8_t valid_idx = 0;

    for (uint16_t r = cam->margin_cut; r < cam->height - cam->margin_cut; r++) {
        for (uint16_t c = cam->margin_cut; c < cam->width - cam->margin_cut; c++) {
            uint32_t idx = r * cam->width + c;
            
            // 发现未访问的亮点 (新连通域的种子点)
            if (cam->binarized_image[idx] == 1 && visited[idx] == 0) {
                BlobStats stats;
                // 跑一遍 DFS，获取该连通域的全部数学特征
                dfs_iterative(cam, visited, label, r, c, &stats);
                
                if (label < 255) label++; // 最大支持254个连通域
                
                // 立即判断该连通域并解算，无需第二次遍历整幅图像
                // [新增] 提取前先计算该连通域的质心位置
                float cy = (float)stats.sum_r / stats.dot_num; // Row (Y)
                float cx = (float)stats.sum_c / stats.dot_num; // Col (X)
                
                // 计算距离画面中心的平方
                float img_cx = cam->width / 2.0f;
                float img_cy = cam->height / 2.0f;
                float dx = cx - img_cx;
                float dy = cy - img_cy;
                float dist_sq = dx * dx + dy * dy;
                
                // 动态计算该位置的最小面积门槛 (越靠边缘要求越低)
                float dynamic_min_area = 1;
                // if (dynamic_min_area < ABS_MIN_AREA) {
                //     dynamic_min_area = ABS_MIN_AREA; // 兜底绝对下限
                // }

                // 立即判断该连通域并解算 (使用动态面积门槛)
                if (stats.dot_num > dynamic_min_area) {
                    if (valid_idx < MAX_LIGHTS) {
                        cam->centers[valid_idx][0] = cy;
                        cam->centers[valid_idx][1] = cx; 
                        cam->dot_num[valid_idx] = stats.dot_num; 
                        
                        // 计算协方差矩阵特征值及长宽比
                        float mu20 = (float)stats.sum_cc / stats.dot_num - cx * cx;
                        float mu02 = (float)stats.sum_rr / stats.dot_num - cy * cy;
                        float mu11 = (float)stats.sum_rc / stats.dot_num - cx * cy;
                        
                        mu02 = mu02 * (K_Y * K_Y);
                        mu11 = mu11 * K_Y;
                        
                        // 计算特征值 (散布程度)
                        float delta = sqrtf((mu20 - mu02)*(mu20 - mu02) + 4.0f * mu11 * mu11);
                        float lambda1 = (mu20 + mu02 + delta) / 2.0f;
                        float lambda2 = (mu20 + mu02 - delta) / 2.0f;
                        
                        float ratio = 1.0f;
                        if (lambda2 > 0.1f) ratio = lambda1 / lambda2;
                        else ratio = 100.0f; // 如果次轴极小(纯直线)，赋予安全极值防除零
                        
                        cam->aspect_ratio[valid_idx] = ratio;
                        valid_idx++;
                    }
                }
            }
        }
    }
    cam->components_count = label - 1;
    cam->light_number = valid_idx;
}


static void sort_lights(CameraObject *cam) {
    // 默认清除上一帧的锁定状态
    cam->car_valid = 0;
    cam->car_dot_num = 0;
    cam->car_ratio = 0.0f;
    cam->car_center_x = 0.0f;
    cam->car_center_y = 0.0f;

    cam->target_valid = 0;
    cam->target_dot_num = 0;
    cam->target_ratio = 0.0f;
    cam->target_center_x = 0.0f;
    cam->target_center_y = 0.0f;

    if (cam->light_number == 0) return;

    // =======================================================
    // 1. 获取当前无人机高度 (用于简单距离估算)
    // =======================================================
    extern volatile float share_data_from_0[];
    float current_height = share_data_from_0[3];
    if (current_height < 30.0f) current_height = 30.0f; // 防除零及贴地保护

    float phys_dist_sq[MAX_LIGHTS] = {0};
    uint8_t is_valid_blob[MAX_LIGHTS] = {0};

    // =======================================================
    // 2. 计算简单物理距离，并做【面积动态过滤】
    // =======================================================
    for (int i = 0; i < cam->light_number && i < MAX_LIGHTS; i++) {
        float out_x, out_y, out_dist;
        
        // 调用简单估算接口
        // 注意传参：centers[i][1] 是 Col(X), centers[i][0] 是 Row(Y)
        Estimate_Distance_Simple(cam->centers[i][1], cam->centers[i][0], 
                                 current_height, &out_x, &out_y, &out_dist);
        
        // 记录物理距离的平方 (单位：平方厘米)
        phys_dist_sq[i] = out_dist * out_dist;

        // 【核心修改】：使用物理距离计算动态面积下限！
        if(out_dist > 150.0f) cam->dot_num[i] *= (out_dist / 100.0f);
        float dynamic_min_area = BASE_MIN_AREA;
        

        // 只有面积在当前物理距离下达标的点，才允许进入后续竞选
        if (cam->dot_num[i] >= dynamic_min_area) {
            is_valid_blob[i] = 1;
        }
    }
    // =======================================================
    // [新增]：独立测试极值记录逻辑 (只记录画面中面积最大的灯，防噪点)
    // =======================================================
    uint32_t max_area = 0;
    int max_area_idx = -1;
    for (int i = 0; i < cam->light_number && i < MAX_LIGHTS; i++) {
        if (cam->dot_num[i] > max_area) {
            max_area = cam->dot_num[i];
            max_area_idx = i;
        }
    }
    
    // 找到了最大的灯，更新历史极值
    if (max_area_idx != -1) {
        float current_max_ratio = cam->aspect_ratio[max_area_idx];
        if (current_max_ratio > cam->debug_max_ratio) {
            cam->debug_max_ratio = current_max_ratio;
        }
        // 排除掉刚好等于 100.0 的情况 (在 calculate_centroids 里设定的噪点线)
        if (current_max_ratio < cam->debug_min_ratio && current_max_ratio < 99.0f) {
            cam->debug_min_ratio = current_max_ratio;
        }
    }
    // =======================================================

    int car_idx = -1;
    int target_idx = -1;
    float img_cx = cam->width / 2.0f;
    float img_cy = cam->height / 2.0f;
    // =========================================================
    // 1. 寻找小车 (加入动态阈值，边缘门槛自动抬高防信标混淆)
    // =========================================================
    float max_car_ratio_found = 0.0f; // 记录找到的最大长宽比
    
    for (int i = 0; i < cam->light_number && i < MAX_LIGHTS; i++) {
        // 计算目标质心到画面中心的像素距离平方
        float dx = cam->centers[i][1] - img_cx;
        float dy = cam->centers[i][0] - img_cy;
        float dist_sq = dx * dx + dy * dy;
        
        // 动态计算该位置的小车最低长宽比门槛
        float dynamic_car_min_ratio = CAR_BASE_MIN_RATIO + (dist_sq * CAR_RATIO_COMP_COEF);
        
        // 只有大于当前位置的动态门槛，才有资格参与小车竞选
        if (cam->aspect_ratio[i] > dynamic_car_min_ratio 
            && cam->centers[i][1] > EDGE_SAFE_MARGIN_X && cam->centers[i][1] < cam->width - EDGE_SAFE_MARGIN_X
            && cam->centers[i][0] > EDGE_SAFE_MARGIN_Y && cam->centers[i][0] < cam->height - EDGE_SAFE_MARGIN_Y
        ) {
            // 在所有合格的候选者中，选出长宽比最大的那个
            if (cam->aspect_ratio[i] > max_car_ratio_found) {
                max_car_ratio_found = cam->aspect_ratio[i];
                car_idx = i;
            }
        }
    }
    
    // 2. 寻找信标 (排除小车后，在动态畸变长宽比阈值内选距离画面中心最近的)
    float min_dist_sq = 999999.0f; // 记录最小的中心距离平方 (初始给一个极大值)
    target_idx = -1; // 确保重置

    for (int i = 0; i < cam->light_number && i < MAX_LIGHTS; i++) {
        if (i == car_idx) continue; 
        
        // 计算目标质心到画面中心的像素距离平方 
        float dx = cam->centers[i][1] - img_cx;
        float dy = cam->centers[i][0] - img_cy;
        float dist_sq = dx * dx + dy * dy;
        
        // 动态阈值补偿：越靠近边缘，允许的信标形变长宽比上限越大
        float dynamic_target_max_ratio = TARGET_BASE_MAX_RATIO + (dist_sq * TARGET_RATIO_COMP_COEF);
        
        // 限制补偿的绝对上限，防止无限放大后与小车混淆
        if (dynamic_target_max_ratio > TARGET_LIMIT_MAX_RATIO) {
            dynamic_target_max_ratio = TARGET_LIMIT_MAX_RATIO; 
        }

        // 使用算出来的动态长宽比阈值进行形状筛选
        if (cam->aspect_ratio[i] < dynamic_target_max_ratio) {
            
            // 【核心修改：按中心距离打擂台】
            // 只要形状合格，谁离画面中心最贴近，谁就是真正的信标！
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                target_idx = i;
            }
        }
    }

    // 3. 将结果输出到专属的安全变量中 (不破坏原始 centers 数组)
    if (car_idx != -1) {
        cam->car_valid = 1;
        cam->car_center_y = cam->centers[car_idx][0];
        cam->car_center_x = cam->centers[car_idx][1];
        cam->car_dot_num = cam->dot_num[car_idx];
        cam->car_ratio = cam->aspect_ratio[car_idx];
    }
    
    if (target_idx != -1) {
        cam->target_valid = 1;
        cam->target_center_y = cam->centers[target_idx][0];
        cam->target_center_x = cam->centers[target_idx][1];
        cam->target_dot_num = cam->dot_num[target_idx];
        cam->target_ratio = cam->aspect_ratio[target_idx];
    }
}

// --- 4. 外部调用的处理入口 ---
void image_processing_loop(void) {
    // 1. 二值化与膨胀一趟融合处理，免去额外内存拷贝与遍历
    binarize_and_dilate(&cam_down);

    // 2. 连通域提取与质心、特征值计算一次性完成
    extract_components(&cam_down, visited_buffer);

    // 3. 按面积从大到小排序 (需同步交换 aspect_ratio)
    
    sort_lights(&cam_down);

    // 4. 矫正处理 share_data_from_0: [0]=Roll, [1]=Pitch, [3]=Height
    extern float share_data_from_0[];
    calculate_ground_positions(share_data_from_0[3], share_data_from_0[1], share_data_from_0[0]);
} 