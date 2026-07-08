#include "image.h"
#include "zf_common_headfile.h"

// ******************************************************************************
// 图像处理流水线 (原始灰度 → 光斑分类结果)
//
//   image_processing_loop() (CM7_1, 每帧调用)
//        │
//   ┌────┴────────────────────────────────────────────────┐
//   │ 1. binarize_pass()        逐像素动态阈值二值化       │
//   │    ├─ 3×3块共用一个阈值                              │
//   │    └─ 中心THRESHOLD_MAX(130) → 边缘THRESHOLD_MIN(40) │
//   │ 2. 双重闭运算 (桥接线缆造成的连通域断裂)              │
//   │    dilate → erode → dilate → erode (8邻域)         │
//   │ 3. extract_components()   连通域提取+特征计算(单遍)   │
//   │    ├─ DFS迭代版 (静态栈, 防栈溢出)                   │
//   │    ├─ 同步计算: 质心 + 协方差矩阵 + 长宽比            │
//   │    └─ FOV圆形掩码 + 动态最小面积过滤                  │
//   │ 4. sort_lights()          光斑分类 (小车 vs 信标)     │
//   │    ├─ 小车: 动态长宽比门槛 → 选ratio最大者            │
//   │    ├─ 信标: 动态畸变补偿 → 排除小车后选中心最近者     │
//   │    └─ 小光斑(≤20px)直接通过形状筛选                  │
//   │ 5. calculate_ground_positions() 地面投影 (见image_process.c) │
//   └────────────────────────────────────────────────────┘
//
// 关键全局: cam_down (CameraObject) — 所有图像状态
// ******************************************************************************

// --- 1. 内存分配 ---
// 定义二值化图像缓冲区 (只定义一个)
uint8_t buffer_bin_down[MT9V03X_H][MT9V03X_W];
// 定义 DFS 访问标记数组 (静态分配以防栈溢出)
uint8_t visited_buffer[MT9V03X_H * MT9V03X_W];
uint8 image_copy[MT9V03X_H][MT9V03X_W];
uint8_t thresh_by_rho2[(int)FOV_RADIUS_SQ + 1]; // 逐像素阈值查找表: rho² → 二值化阈值
// 定义全局实例
CameraObject cam_down;
Image_IMU_Snapshot_t img_imu_snap = {0}; // 新增全局快照实例

// =========================================================
// 预计算每行扫描的有效列边界，极大节约主循环算力
// =========================================================
uint16_t fov_left_bound[MT9V03X_H];
uint16_t fov_right_bound[MT9V03X_H];
uint16_t morph_left_bound[MT9V03X_H];
uint16_t morph_right_bound[MT9V03X_H];
uint16_t edge_clean_left_bound[MT9V03X_H];   // 边缘泛光清除圆 (独立于FOV)
uint16_t edge_clean_right_bound[MT9V03X_H];

// =========================================================
// 预计算FOV边沿内侧像素索引 (用于清除触碰边缘的光斑污染)
// =========================================================
#define MAX_BORDER_POINTS 400  // 边缘清除圆周长上限 (~2*PI*63.75 ≈ 400)
static uint16_t border_indices[MAX_BORDER_POINTS];
static int border_pixel_count = 0;

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
    cam_down.debug.max_ratio = 0.0f;
    cam_down.debug.min_ratio = 999.0f;

    // 预计算逐像素阈值查找表: rho² → threshold, 中心高阈值边缘低阈值
    int span = THRESHOLD_MAX - THRESHOLD_MIN;
    for (int i = 0; i <= (int)FOV_RADIUS_SQ; i++) {
        int thr = THRESHOLD_MAX - (i * span) / (int)FOV_RADIUS_SQ;
        if (thr < THRESHOLD_MIN) thr = THRESHOLD_MIN;
        thresh_by_rho2[i] = (uint8_t)thr;
    }

    // 初始化阶段预计算每一行的圆形视野起始和结束列
    for (int r = 0; r < MT9V03X_H; r++) {
        float dy = r - CAM_CY;
        float dy_sq = dy * dy;
        if (dy_sq > FOV_RADIUS_SQ) {
            fov_left_bound[r] = MT9V03X_W; // 本行全在圆外，设为无效区间
            fov_right_bound[r] = 0;
        } else {
            float dx = sqrtf(FOV_RADIUS_SQ - dy_sq);
            int left = (int)(CAM_CX - dx);
            int right = (int)(CAM_CX + dx);
            if (left < 0) left = 0;
            if (right > MT9V03X_W) right = MT9V03X_W;
            fov_left_bound[r] = left;
            fov_right_bound[r] = right;
        }

        // [新增] 形态学中心圆形遮罩 (防止边缘噪点被膨胀放大)
        if (dy_sq > MORPH_MASK_RADIUS_SQ) {
            morph_left_bound[r] = MT9V03X_W;
            morph_right_bound[r] = 0;
        } else {
            float dx = sqrtf(MORPH_MASK_RADIUS_SQ - dy_sq);
            int left = (int)(CAM_CX - dx);
            int right = (int)(CAM_CX + dx);
            if (left < 0) left = 0;
            if (right > MT9V03X_W) right = MT9V03X_W;
            morph_left_bound[r] = left;
            morph_right_bound[r] = right;
        }

        // [新增] 边缘泛光清除圆 (独立于FOV，可单独调整直径)
        if (dy_sq > EDGE_CLEAN_RADIUS_SQ) {
            edge_clean_left_bound[r] = MT9V03X_W;
            edge_clean_right_bound[r] = 0;
        } else {
            float dx = sqrtf(EDGE_CLEAN_RADIUS_SQ - dy_sq);
            int left = (int)(CAM_CX - dx);
            int right = (int)(CAM_CX + dx);
            if (left < 0) left = 0;
            if (right > MT9V03X_W) right = MT9V03X_W;
            edge_clean_left_bound[r] = left;
            edge_clean_right_bound[r] = right;
        }
    }

    // 预计算边缘泛光清除圆边沿内侧像素索引 (触碰边缘的光斑将在二值化前被清除)
    border_pixel_count = 0;
    // 从第1行到倒数第2行遍历 (避免上下邻居检查时越界)
    for (uint16_t r = 1; r < MT9V03X_H - 1; r++) {
        uint16_t c_start = edge_clean_left_bound[r];
        uint16_t c_end   = edge_clean_right_bound[r];
        if (c_start >= c_end) continue; // 该行完全在清除圆外

        // 确保列边界不超出图像1像素 (防止左右邻居检查时越界)
        if (c_start < 1) c_start = 1;
        if (c_end > MT9V03X_W - 1) c_end = MT9V03X_W - 1;

        for (uint16_t c = c_start; c < c_end; c++) {
            // 检查该边缘圆内像素是否与边缘圆外像素相邻 (4邻域)
            int is_edge = 0;

            // 上邻域在边缘圆外?
            if (c < edge_clean_left_bound[r - 1] || c >= edge_clean_right_bound[r - 1])
                is_edge = 1;
            // 下邻域在边缘圆外?
            else if (c < edge_clean_left_bound[r + 1] || c >= edge_clean_right_bound[r + 1])
                is_edge = 1;
            // 左邻域在边缘圆外? (即当前列是本行边缘圆的最左列)
            else if (c == c_start)
                is_edge = 1;
            // 右邻域在边缘圆外? (即当前列是本行边缘圆的最右列)
            else if (c + 1 >= c_end)
                is_edge = 1;

            if (is_edge && border_pixel_count < MAX_BORDER_POINTS) {
                border_indices[border_pixel_count++] = r * MT9V03X_W + c;
            }
        }
    }
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

    // 增加虚拟焦距的安全下限，防除零和异常反向
    if (z_poly < 0.1f) {
        z_poly = 0.1f;
    }

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
            // 简单的边界预判 (利用无符号数溢出特性，小于 0 会变成超大正数从而大于宽高)
            if (nr < cam->height && nc < cam->width) {
                
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

// [新增] 超高速边界光斑清除：查表法 + 一维 Flood-fill
// 遍历所有FOV边沿内侧像素，发现白色即作为种子点进行泛洪填充，
// 将整个触碰FOV边缘的连通域染黑，防止其在后续膨胀中向内污染信标区域
// 清除FOV边缘泛光污染 —— 在二值化前于原始灰度图上运行。
// EDGE_BLOB_THRESHOLD 远比二值化阈值(130→5)敏感，尽早消除边缘光晕向内扩散的风险。
static void clear_edge_blobs(CameraObject *cam) {
    uint16_t w = cam->width;
    uint16_t h = cam->height;
    uint32_t total = (uint32_t)w * h;

    // 一级栈（静态，零额外开销）
    static uint16_t stack[STACK_SIZE];
    int top = -1;

    // 二级栈（借用 binarized_image，此时尚未被二值化 memset 使用）
    // 22560 字节 → 11280 个 uint16_t，总量 4096 + 11280 > FOV 面积，永不溢出
    uint16_t *spill = (uint16_t *)cam->binarized_image;
    uint32_t spill_wr = 0;  // 二级栈写入位置
    uint32_t spill_rd = 0;  // 二级栈读出位置

    // 1. O(N) 查表扫描：发现FOV边沿上灰度值超过阈值的像素，清零并入栈
    for (int i = 0; i < border_pixel_count; i++) {
        uint32_t idx = border_indices[i];
        if (cam->raw_image[idx] > EDGE_BLOB_THRESHOLD) {
            cam->raw_image[idx] = 0;
            if (top < STACK_SIZE - 1)
                stack[++top] = (uint16_t)idx;
            else
                spill[spill_wr++] = (uint16_t)idx;
        }
    }

    // 2. 八方向 Flood-fill 扩散，清除整个连通域 (含对角连接的细斜线/单像素桥接)
    const int32_t d_idx[] = {-w, 1, w, -1,                 // 上、右、下、左
                             -w - 1, -w + 1, w - 1, w + 1}; // 左上、右上、左下、右下

    while (1) {
        uint16_t curr_idx;
        // 优先消费二级栈 (FIFO)，空则回退到一级栈 (LIFO)
        if (spill_rd < spill_wr)
            curr_idx = spill[spill_rd++];
        else if (top >= 0)
            curr_idx = stack[top--];
        else
            break;

        for (int i = 0; i < 8; i++) {
            uint32_t nidx = curr_idx + d_idx[i];

            if (nidx < total && cam->raw_image[nidx] > EDGE_BLOB_THRESHOLD) {
                cam->raw_image[nidx] = 0;
                if (top < STACK_SIZE - 1)
                    stack[++top] = (uint16_t)nidx;
                else
                    spill[spill_wr++] = (uint16_t)nidx;
            }
        }
    }
}

// 逐像素动态阈值二值化: 每3×3像素块共用一个阈值, 离图像中心越远阈值越低
static void binarize_pass(CameraObject *cam) {
    memset(cam->binarized_image, 0, cam->width * cam->height);
    uint16_t h = cam->height;
    uint16_t w = cam->width;

    for (uint16_t r = 0; r < h; r += 3) {
        uint16_t r_end = r + 3;
        if (r_end > h) r_end = h;
        int32_t cy = r + 1;
        if (cy >= (int32_t)h) cy = r;
        int32_t dy = cy - (int32_t)CAM_CY;
        int32_t dy_sq = dy * dy;

        for (uint16_t c = 0; c < w; c += 3) {
            uint16_t c_end = c + 3;
            if (c_end > w) c_end = w;
            int32_t cx = c + 1;
            if (cx >= (int32_t)w) cx = c;
            int32_t dx = cx - (int32_t)CAM_CX;
            int32_t rho2 = dx * dx + dy_sq;
            uint8_t thr;
            if (rho2 > (int32_t)FOV_RADIUS_SQ) thr = THRESHOLD_MIN;
            else thr = thresh_by_rho2[rho2];

            for (uint16_t br = r; br < r_end; br++) {
                uint16_t bc_start = fov_left_bound[br] > c ? fov_left_bound[br] : c;
                uint16_t bc_end   = fov_right_bound[br] < c_end ? fov_right_bound[br] : c_end;
                for (uint16_t bc = bc_start; bc < bc_end; bc++) {
                    uint32_t idx = br * w + bc;
                    if (cam->raw_image[idx] > thr) {
                        cam->binarized_image[idx] = 1;
                    }
                }
            }
        }
    }
}

// 8邻域膨胀: src → dst
static void dilate_pass(uint8_t *src, uint8_t *dst, uint16_t width, uint16_t height) {
    memset(dst, 0, width * height);

    for (uint16_t r = 0; r < height; r++) {
        uint16_t c_start = fov_left_bound[r];
        uint16_t c_end   = fov_right_bound[r];

        for (uint16_t c = c_start; c < c_end; c++) {
            uint32_t idx = r * width + c;
            
            // [修正] 将膨胀改为 "Gather (拉取)" 模式，彻底杜绝边界向外溢出污染
            if (c >= morph_left_bound[r] && c < morph_right_bound[r]) {
                // 在遮罩内：只要自身或周围 8 邻域有一个为 1，当前点就膨胀为 1 (利用 || 短路特性，速度极快)
                if (src[idx] || src[idx - 1] || src[idx + 1] ||
                    src[idx - width] || src[idx + width] ||
                    src[idx - width - 1] || src[idx - width + 1] ||
                    src[idx + width - 1] || src[idx + width + 1]) {
                    dst[idx] = 1;
                }
            } else {
                // 在遮罩外：严格原样保留自身，不接受遮罩内溢出的膨胀
                if (src[idx]) {
                    dst[idx] = 1;
                }
            }
        }
    }
}

// 8邻域腐蚀: src → dst (仅当像素自身及8邻域全为1时保留)
static void erode_pass(uint8_t *src, uint8_t *dst, uint16_t width, uint16_t height) {
    memset(dst, 0, width * height);

    for (uint16_t r = 0; r < height; r++) {
        uint16_t c_start = fov_left_bound[r];
        uint16_t c_end   = fov_right_bound[r];

        for (uint16_t c = c_start; c < c_end; c++) {
            uint32_t idx = r * width + c;
            if (src[idx] == 1) {
                // 仅在中心形态学圆内进行腐蚀过滤
                if (c >= morph_left_bound[r] && c < morph_right_bound[r]) {
                    uint8_t count = src[idx - 1] + src[idx + 1] +
                                    src[idx - width] + src[idx + width] +
                                    src[idx - width - 1] + src[idx - width + 1] +
                                    src[idx + width - 1] + src[idx + width + 1];
                    if (count >= ERODE_MIN_NEIGHBORS) {
                        dst[idx] = 1;
                    }
                } else {
                    dst[idx] = 1; // 在遮罩外的点跳过腐蚀判断，直接保留
                }
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

    for (uint16_t r = 0; r < cam->height; r++) {
        // 查表读取该行的安全列边界
        uint16_t c_start = fov_left_bound[r];
        uint16_t c_end   = fov_right_bound[r];

        for (uint16_t c = c_start; c < c_end; c++) {
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
                float dx = cx - CAM_CX;
                float dy = cy - CAM_CY;
                float dist_sq = dx * dx + dy * dy;
                
                // 动态计算该位置的最小面积门槛 (越靠边缘要求越低)
                float dynamic_min_area = BASE_MIN_AREA;

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
                        else ratio = 80.0f; // 次轴极小(纯直线)，赋予退化极值
                        
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

// 处理信标丢失后的保持逻辑
static void apply_target_hold_logic(CameraObject *cam) {
    static uint8_t target_consecutive_frames = 0;
    static uint8_t target_hold_frames = 0;

    if (cam->target_valid) {
        // 本帧有效锁定到了信标
        if (target_consecutive_frames < 255) {
            target_consecutive_frames++;
        }
        target_hold_frames = 0;
    } else {
        // 本帧没有锁定到信标
        if (target_consecutive_frames >= TARGET_MIN_CONSECUTIVE_FRAMES && target_hold_frames < TARGET_HOLD_FRAMES) {
            // 满足保持条件，强行锁定并沿用上一次的值
            cam->target_valid = 1;
            target_hold_frames++;
        } else {
            // 保持时间结束或未达到保持条件，彻底清除数据
            target_consecutive_frames = 0;
            cam->target_dot_num = 0;
            cam->target_ratio = 0.0f;
            cam->target_center_x = 0.0f;
            cam->target_center_y = 0.0f;
        }
    }
}

// 处理小车丢失后的保持逻辑 (防 locked_lights 单帧从3骤降至0)
static void apply_car_hold_logic(CameraObject *cam) {
    static uint8_t car_consecutive_frames = 0;
    static uint8_t car_hold_frames = 0;

    if (cam->car_valid) {
        // 本帧有效锁定到了小车
        if (car_consecutive_frames < 255) {
            car_consecutive_frames++;
        }
        car_hold_frames = 0;
    } else {
        // 本帧没有锁定到小车
        if (car_consecutive_frames >= CAR_MIN_CONSECUTIVE_FRAMES && car_hold_frames < CAR_HOLD_FRAMES) {
            // 满足保持条件，强行锁定并沿用上一次的值
            cam->car_valid = 1;
            car_hold_frames++;
        } else {
            // 保持时间结束或未达到保持条件，彻底清除数据
            car_consecutive_frames = 0;
            cam->car_dot_num = 0;
            cam->car_ratio = 0.0f;
            cam->car_center_x = 0.0f;
            cam->car_center_y = 0.0f;
        }
    }
}

static void sort_lights(CameraObject *cam) {
    // 默认清除上一帧的锁定状态
    cam->car_valid = 0;
    cam->car_raw_valid = 0;
    cam->car_dot_num = 0;
    cam->car_ratio = 0.0f;
    cam->car_center_x = 0.0f;
    cam->car_center_y = 0.0f;

    cam->target_valid = 0;
    cam->target_raw_valid = 0;
    // 不在此处清除信标坐标等信息，以支持保持最后一次信标位置

    // 信标身份记忆：同一目标获得线性距离优惠，避免双信标场景下频繁翻转。
    static float last_target_ground_x = 0.0f;
    static float last_target_ground_y = 0.0f;
    static uint8_t has_last_target = 0;

    if (cam->light_number == 0) {
        cam->debug.pass_area = 0;
        cam->debug.pass_car = 0;
        cam->debug.pass_target = 0;
        apply_target_hold_logic(cam);
        apply_car_hold_logic(cam);
        return;
    }

    // =======================================================
    // 1. 获取当前无人机高度 (用于简单距离估算)
    // =======================================================
    float current_height = img_imu_snap.height;
    if (current_height < HEIGHT_ESTIMATE_MIN) current_height = HEIGHT_ESTIMATE_MIN;

    float phys_dist_sq[MAX_LIGHTS] = {0};
    float ground_x[MAX_LIGHTS] = {0};
    float ground_y[MAX_LIGHTS] = {0};
    uint8_t is_valid_blob[MAX_LIGHTS] = {0};

    // =======================================================
    // 2. 计算精确物理距离，并做【面积动态过滤】
    // =======================================================
    for (int i = 0; i < cam->light_number && i < MAX_LIGHTS; i++) {
        double out_x, out_y, out_dist;
        
        // 调用精确估算接口
        // 注意传参：centers[i][1] 是 Col(X), centers[i][0] 是 Row(Y)
        get_accurate_ground_distance((double)cam->centers[i][1], (double)cam->centers[i][0], 
                                     (double)current_height, (double)img_imu_snap.pitch, (double)img_imu_snap.roll,
                                     &out_x, &out_y, &out_dist);
        
        // 记录物理距离的平方 (单位：平方厘米)
        ground_x[i] = (float)out_x;
        ground_y[i] = (float)out_y;
        phys_dist_sq[i] = (float)(out_dist * out_dist);

        // 使用局部变量做距离补偿过滤，不改动原始 dot_num
        float area_for_filter = (float)cam->dot_num[i];
        if (out_dist > DIST_COMP_THRESHOLD) area_for_filter *= (float)(out_dist / DIST_COMP_SCALE);
        float dynamic_min_area = BASE_MIN_AREA;

        // 只有面积在当前物理距离下达标的点，才允许进入后续竞选
        if (area_for_filter >= dynamic_min_area) {
            is_valid_blob[i] = 1;
        }
    }
    cam->debug.pass_area = 0;
    for (int i = 0; i < cam->light_number && i < MAX_LIGHTS; i++) {
        if (is_valid_blob[i]) cam->debug.pass_area++;
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
    cam->max_area = max_area; // 保存供状态机做异常检测
    
    // 找到了最大的灯，更新历史极值
    if (max_area_idx != -1) {
        float current_max_ratio = cam->aspect_ratio[max_area_idx];
        if (current_max_ratio > cam->debug.max_ratio) {
            cam->debug.max_ratio = current_max_ratio;
        }
        if (current_max_ratio < cam->debug.min_ratio && current_max_ratio < DEGENERATE_RATIO_MARK) {
            cam->debug.min_ratio = current_max_ratio;
        }
    }
    // =======================================================

    int car_idx = -1;
    int target_idx = -1;
    // =========================================================
    // 1. 寻找小车 (加入动态阈值，边缘门槛自动抬高防信标混淆)
    // =========================================================
    float max_car_score = 0.0f; // [修改] 记录找到的小车最高综合得分
    
    for (int i = 0; i < cam->light_number && i < MAX_LIGHTS; i++) {
        if (!is_valid_blob[i]) continue;
        
        // 限制：找小车距离在2m以内 (200cm)
        if (phys_dist_sq[i] > 150.0f * 150.0f) continue;
        
        // 计算目标质心到画面中心的像素距离平方
        float dx = cam->centers[i][1] - CAM_CX;
        float dy = cam->centers[i][0] - CAM_CY;
        float dist_sq = dx * dx + dy * dy;
        
        // 动态计算该位置的小车最低长宽比门槛
        float dynamic_car_min_ratio = CAR_BASE_MIN_RATIO + (dist_sq * CAR_RATIO_COMP_COEF);
        
        // 只有在动态门槛与绝对红线之间的连通域，才有资格参与小车竞选
        if (cam->aspect_ratio[i] > dynamic_car_min_ratio && cam->aspect_ratio[i] < CAR_ABSOLUTE_MAX_RATIO
            && cam->centers[i][1] > EDGE_SAFE_MARGIN_X && cam->centers[i][1] < cam->width - EDGE_SAFE_MARGIN_X
            && cam->centers[i][0] > EDGE_SAFE_MARGIN_Y && cam->centers[i][0] < cam->height - EDGE_SAFE_MARGIN_Y
            && dist_sq < CAR_MAX_CENTER_DIST_SQ
        ) {
            // [新增] 综合得分算法：Score = Area * sqrt(Ratio)
            // 既保证面积是基本盘，又让形状更好的目标(Ratio大)有加分优势
            float effective_ratio = cam->aspect_ratio[i];
            if (effective_ratio > CAR_IDEAL_MAX_RATIO) {
                effective_ratio = CAR_IDEAL_MAX_RATIO; // 截断：过于细长不再继续加分，防止噪点劫持
            }
            
            float current_score = (float)cam->dot_num[i] * sqrtf(effective_ratio);
            
            if (current_score > max_car_score) {
                max_car_score = current_score;
                car_idx = i;
            }
            cam->debug.pass_car++;
        }
    }
    
    // 2. 寻找信标 (排除小车后，选距离小车最近的作为信标)
    // [修复] 线性迟滞，防止远距离时固定平方优惠衰减过快。

    float min_sort_dist = 999999.0f;
    target_idx = -1; // 确保重置

    for (int i = 0; i < cam->light_number && i < MAX_LIGHTS; i++) {
        if (i == car_idx) continue;
        if (!is_valid_blob[i]) continue;

        // 限制：找信标距离在10m以内 (1000cm)
        if (phys_dist_sq[i] > 1200.0f * 1200.0f) continue;

        // 计算目标质心到画面中心的像素距离平方 (用于边缘畸变补偿)
        float dx_c = cam->centers[i][1] - CAM_CX;
        float dy_c = cam->centers[i][0] - CAM_CY;
        float dist_to_center_sq = dx_c * dx_c + dy_c * dy_c;

        // 动态阈值补偿：越靠近边缘，允许的信标形变长宽比上限越大
        float dynamic_target_max_ratio = TARGET_BASE_MAX_RATIO + (dist_to_center_sq * TARGET_RATIO_COMP_COEF);

        // 限制补偿的绝对上限，防止无限放大后与小车混淆
        if (dynamic_target_max_ratio > TARGET_LIMIT_MAX_RATIO) {
            dynamic_target_max_ratio = TARGET_LIMIT_MAX_RATIO;
        }

        // 动态面积门槛平滑过渡：正上方(0m)要求面积>25，4m(400cm)处降为0
        float out_dist = sqrtf(phys_dist_sq[i]);
        float min_target_area = 15.0f * (1.0f - out_dist / 200.0f);
        if (min_target_area < 0.0f) min_target_area = 0.0f;

        // 面积不达标直接排除
        if (cam->dot_num[i] <= min_target_area) continue;

        // 面积过小的连通域长宽比不可靠, 直接通过形状筛选
        if (cam->dot_num[i] <= SMALL_BLOB_DIRECT_AREA || cam->aspect_ratio[i] < dynamic_target_max_ratio) {
            // 【核心：按地面实际距小车距离打擂台，小车不可见时回退到距无人机地面投影距离】
            float sort_dist_sq;
            if (car_idx != -1) {
                float dx_car = ground_x[i] - ground_x[car_idx];
                float dy_car = ground_y[i] - ground_y[car_idx];
                sort_dist_sq = dx_car * dx_car + dy_car * dy_car;
            } else {
                sort_dist_sq = phys_dist_sq[i];
            }
            float sort_dist = sqrtf(sort_dist_sq);

            // [修复] 迟滞：若候选与上一帧选中信标地面位置接近（同一信标），
            // 给予线性距离优惠，防止远距离时固定平方优惠衰减过快。
            float hysteresis_bonus = 0.0f;
            if (has_last_target && car_idx != -1) {
                float dx_last = ground_x[i] - last_target_ground_x;
                float dy_last = ground_y[i] - last_target_ground_y;
                float dist_to_last_sq = dx_last * dx_last + dy_last * dy_last;
                if (dist_to_last_sq < HYSTERESIS_MATCH_RADIUS_SQ) {
                    hysteresis_bonus = -HYSTERESIS_DIST_BIAS_CM;
                }
            }
            float effective_dist = sort_dist + hysteresis_bonus;

            if (effective_dist < min_sort_dist) {
                min_sort_dist = effective_dist;
                target_idx = i;
            }
            cam->debug.pass_target++;
        }
    }

    // 更新迟滞记忆：记录本帧最终输出的信标地面坐标
    if (target_idx != -1) {
        last_target_ground_x = ground_x[target_idx];
        last_target_ground_y = ground_y[target_idx];
        has_last_target = 1;
    }
    // 注意：target_idx == -1 时不清除 has_last_target，
    // 短暂丢失后恢复时仍能匹配到之前的信标。

    // 3. 将结果输出到专属的安全变量中 (不破坏原始 centers 数组)
    if (car_idx != -1) {
        cam->car_valid = 1;
        cam->car_raw_valid = 1;
        cam->car_center_y = cam->centers[car_idx][0];
        cam->car_center_x = cam->centers[car_idx][1];
        cam->car_dot_num = cam->dot_num[car_idx];
        cam->car_ratio = cam->aspect_ratio[car_idx];
    }
    
    if (target_idx != -1) {
        cam->target_valid = 1;
        cam->target_raw_valid = 1;
        cam->target_center_y = cam->centers[target_idx][0];
        cam->target_center_x = cam->centers[target_idx][1];
        cam->target_dot_num = cam->dot_num[target_idx];
        cam->target_ratio = cam->aspect_ratio[target_idx];
    }
    
    // 应用信标丢失保持逻辑
    apply_target_hold_logic(cam);
    // 应用小车丢失保持逻辑 (防 locked_lights 单帧骤降)
    apply_car_hold_logic(cam);
}

// --- 4. 外部调用的处理入口 ---
void image_processing_loop(void) {
    // [新] 预先清除FOV边缘泛光污染 — 在二值化前的原始灰度图上进行
    clear_edge_blobs(&cam_down);

    // 1. 逐像素动态阈值二值化 (越靠近图像边缘阈值越低)
    binarize_pass(&cam_down);

    // 2. 双重闭运算: dilate → erode → dilate → erode (桥接线缆造成的断裂)
    uint8_t *bin = (uint8_t *)cam_down.binarized_image;
    uint8_t *tmp = (uint8_t *)image_copy;
    uint16_t w = cam_down.width;
    uint16_t h = cam_down.height;

    // 3次膨胀 (交替使用 bin 和 tmp 缓冲区，链式传递)
    dilate_pass(bin, tmp, w, h);  // 1: bin -> tmp
    dilate_pass(tmp, bin, w, h);  // 2: tmp -> bin
    dilate_pass(bin, tmp, w, h);  // 3: bin -> tmp
    // 3次腐蚀 (交替使用 bin 和 tmp 缓冲区，最终输出至 bin)
    erode_pass(tmp, bin, w, h);   // 1: tmp -> bin
    erode_pass(bin, tmp, w, h);   // 2: bin -> tmp
    erode_pass(tmp, bin, w, h);   // 3: tmp -> bin

    // 3. 连通域提取与质心、特征值计算一次性完成
    extract_components(&cam_down, visited_buffer);

    // 4. 按面积从大到小排序 (需同步交换 aspect_ratio)
    sort_lights(&cam_down);

    // 5. 矫正处理 使用锁定快照，保证整个运算链路无时序冲突
    calculate_ground_positions(img_imu_snap.height, img_imu_snap.pitch, img_imu_snap.roll, img_imu_snap.yaw);
} 
