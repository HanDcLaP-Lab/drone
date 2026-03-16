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
    cam_down.margin_cut = 5;    // 四周裁剪 5 像素
    cam_down.debug_max_ratio = 0.0f;
    cam_down.debug_min_ratio = 999.0f;
}

// --- 3. 内部辅助函数 ---


static uint8_t is_valid_pixel(CameraObject *cam, uint16_t r, uint16_t c, uint8_t *visited) {
    uint32_t index = r * cam->width + c;
    
    return (r >= cam->margin_cut && r < (cam->height - cam->margin_cut) && 
            c >= cam->margin_cut && c < (cam->width - cam->margin_cut) && 
            visited[index] == 0 && 
            cam->binarized_image[index] == 1);
}

// ==========================================
// [新增] 8邻域极速膨胀函数 (缝合线缆遮挡造成的裂缝)
// ==========================================
static void fast_dilate_3x3_8_neighbor(CameraObject *cam) {
    // 1. 清空临时缓冲区 image_copy (利用一维数组的形式快速清空)
    memset(image_copy[0], 0, cam->width * cam->height);
    
    // 2. 遍历原始二值化图像 (避开最外层1个像素边界，防止指针越界)
    for (uint16_t r = cam->margin_cut + 1; r < cam->height - cam->margin_cut - 1; r++) {
        for (uint16_t c = cam->margin_cut + 1; c < cam->width - cam->margin_cut - 1; c++) {
            uint32_t idx = r * cam->width + c;
            
            // 只要当前中心点是 1 (白点)
            if (cam->binarized_image[idx] == 1) {
                // 将缓冲区中的自己和周围 8 个邻居全部点亮
                image_copy[0][idx] = 1;                   // 中心
                image_copy[0][idx - 1] = 1;               // 左
                image_copy[0][idx + 1] = 1;               // 右
                image_copy[0][idx - cam->width] = 1;      // 上
                image_copy[0][idx + cam->width] = 1;      // 下
                
                image_copy[0][idx - cam->width - 1] = 1;  // 左上
                image_copy[0][idx - cam->width + 1] = 1;  // 右上
                image_copy[0][idx + cam->width - 1] = 1;  // 左下
                image_copy[0][idx + cam->width + 1] = 1;  // 右下
            }
        }
    }
    
    // 3. 将膨胀后的连续图像覆盖回原数组，供后面的 DFS 搜索使用
    memcpy(cam->binarized_image, image_copy[0], cam->width * cam->height);
}

// 深度优先搜索 (DFS) - 迭代版 (防止栈溢出)
static void dfs_iterative(CameraObject *cam, uint8_t *visited, uint8_t label, uint16_t start_r, uint16_t start_c) {
    typedef struct { uint16_t r; uint16_t c; } Node;
    static Node stack[STACK_SIZE]; // 静态局部栈
    int top = -1;

    // 起点入栈
    stack[++top] = (Node){start_r, start_c};
    
    // 方向数组：上右下左
    const int dr[] = {-1, 0, 1, 0};
    const int dc[] = {0, 1, 0, -1};

    while (top >= 0) {
        Node curr = stack[top--];
        
        // 再次检查 (防止重复入栈处理)
        if (!is_valid_pixel(cam, curr.r, curr.c, visited)) continue;

        // 标记访问
        visited[curr.r * cam->width + curr.c] = label;

        // 搜索 4 邻域
        for (int i = 0; i < 4; i++) {
            uint16_t nr = curr.r + dr[i];
            uint16_t nc = curr.c + dc[i];
            // 这里只做简单的边界预判，具体有效性由下一次循环的 is_valid_pixel 判断
            if (nr >= cam->margin_cut && nr < (cam->height - cam->margin_cut) &&
                nc >= cam->margin_cut && nc < (cam->width - cam->margin_cut)) {
                
                // 只有亮点且未访问才入栈，减少栈占用
                uint32_t nidx = nr * cam->width + nc;
                if (cam->binarized_image[nidx] == 1 && visited[nidx] == 0) {
                    if (top < STACK_SIZE - 1) {
                        stack[++top] = (Node){nr, nc};
                    }
                }
            }
        }
    }
}

// 二值化处理
static void binarize_image(CameraObject *cam) {
    // 清空二值化缓冲区
    memset(cam->binarized_image, 0, cam->width * cam->height);

    // 遍历图像 (避开边缘)
    for (uint16_t i = cam->margin_cut; i < cam->height - cam->margin_cut; i++) {
        for (uint16_t j = cam->margin_cut; j < cam->width - cam->margin_cut; j++) {
            uint32_t idx = i * cam->width + j;
            
            // 阈值判断
            if (cam->raw_image[idx] > cam->threshold) {
                cam->binarized_image[idx] = 1;
            } else {
                cam->binarized_image[idx] = 0;
            }
        }
    }
}

// 连通域标记
static void mark_components(CameraObject *cam, uint8_t *visited) {
    // 清空访问标记
    memset(visited, 0, cam->width * cam->height);

    uint8_t label = 1;
    for (uint16_t i = cam->margin_cut; i < cam->height - cam->margin_cut; i++) {
        for (uint16_t j = cam->margin_cut; j < cam->width - cam->margin_cut; j++) {
            uint32_t idx = i * cam->width + j;
            
            // 发现未访问的亮点
            if (cam->binarized_image[idx] == 1 && visited[idx] == 0) {
                dfs_iterative(cam, visited, label, i, j);
                label++;
                if (label >= 255) break; // label 是 uint8，防溢出
            }
        }
    }
    cam->components_count = label - 1;
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
        if (cam->aspect_ratio[i] > dynamic_car_min_ratio) {
            // 在所有合格的候选者中，选出长宽比最大的那个
            if (cam->aspect_ratio[i] > max_car_ratio_found) {
                max_car_ratio_found = cam->aspect_ratio[i];
                car_idx = i;
            }
        }
    }

    // 2. 寻找信标 (排除小车后，在动态畸变长宽比阈值内选面积最大的)
    uint32_t max_target_area = 0;
    
    // 计算图像物理中心坐标 (用于计算透视偏离度)
    

    for (int i = 0; i < cam->light_number && i < MAX_LIGHTS; i++) {
        if (i == car_idx) continue; 
        
        // 计算目标质心到画面中心的像素距离平方 (代表视角的斜度)
        float dx = cam->centers[i][1] - img_cx;
        float dy = cam->centers[i][0] - img_cy;
        float dist_sq = dx * dx + dy * dy;
        
        // 动态阈值补偿：越靠近边缘，允许的信标形变长宽比上限越大
        float dynamic_target_max_ratio = TARGET_BASE_MAX_RATIO + (dist_sq * TARGET_RATIO_COMP_COEF);
        
        // 限制补偿的绝对上限，防止无限放大后与小车混淆
        if (dynamic_target_max_ratio > TARGET_LIMIT_MAX_RATIO) {
            dynamic_target_max_ratio = TARGET_LIMIT_MAX_RATIO; 
        }

        // 使用算出来的动态阈值进行筛选
        if (cam->aspect_ratio[i] < dynamic_target_max_ratio) {
            if (cam->dot_num[i] > max_target_area) {
                max_target_area = cam->dot_num[i];
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





static void calculate_centroids(CameraObject *cam, uint8_t *visited) {
    uint32_t sum_r[MAX_DOTS] = {0};
    uint32_t sum_c[MAX_DOTS] = {0};
    
    // 【新增】：用于计算二阶矩的平方和
    uint32_t sum_rr[MAX_DOTS] = {0};
    uint32_t sum_cc[MAX_DOTS] = {0};
    uint32_t sum_rc[MAX_DOTS] = {0};
    
    memset(cam->dot_num, 0, sizeof(cam->dot_num));
    memset(cam->centers, 0, sizeof(cam->centers));
    memset(cam->aspect_ratio, 0, sizeof(cam->aspect_ratio)); // 清空上一帧的长宽比

    // 1. 累加坐标与坐标的平方
    for (uint16_t i = cam->margin_cut; i < cam->height - cam->margin_cut; i++) {
        for (uint16_t j = cam->margin_cut; j < cam->width - cam->margin_cut; j++) {
            uint8_t lbl = visited[i * cam->width + j];
            if (lbl > 0 && lbl <= MAX_DOTS) { 
                sum_r[lbl-1] += i;            // y
                sum_c[lbl-1] += j;            // x
                
                sum_rr[lbl-1] += i * i;       // y^2
                sum_cc[lbl-1] += j * j;       // x^2
                sum_rc[lbl-1] += i * j;       // x*y
                
                cam->dot_num[lbl-1]++;
            }
        }
    }

    // 2. 计算质心和协方差矩阵特征值 (真实长宽比)
    uint8_t valid_idx = 0;
    for (int i = 0; i < cam->components_count && i < MAX_DOTS; i++) {
        uint32_t num = cam->dot_num[i];
        if (num > MIN_LIGHT_SIZE) {
            if (valid_idx < MAX_LIGHTS) {
                // 计算质心
                float cy = (float)sum_r[i] / num; // Row (Y)
                float cx = (float)sum_c[i] / num; // Col (X)
                cam->centers[valid_idx][0] = cy; 
                cam->centers[valid_idx][1] = cx; 
                cam->dot_num[valid_idx] = num; 
                
                // ==========================================
                // 【核心算法】：计算二阶中心矩与真实长宽比
                // ==========================================
                // 计算协方差
                float mu20 = (float)sum_cc[i] / num - cx * cx; // X的方差
                float mu02 = (float)sum_rr[i] / num - cy * cy; // Y的方差
                float mu11 = (float)sum_rc[i] / num - cx * cy; // XY的协方差
                
                
                mu02 = mu02 * (K_Y * K_Y); // Y方差需要乘 K 的平方
                mu11 = mu11 * K_Y;         // 协方差需要乘 K 的一次方
                // 计算特征值 (代表该连通域在最长和最短方向的散布程度)
                float delta = sqrtf((mu20 - mu02)*(mu20 - mu02) + 4.0f * mu11 * mu11);
                float lambda1 = (mu20 + mu02 + delta) / 2.0f; // 主轴(长边)方差
                float lambda2 = (mu20 + mu02 - delta) / 2.0f; // 次轴(短边)方差
                
                // 真实长宽比 = sqrt(主轴方差 / 次轴方差)
                float ratio = 1.0f;
                if (lambda2 > 0.1f) {
                    ratio = lambda1 / lambda2;
                } else {
                    ratio = 100.0f; // 如果次轴极其小(如一条1像素宽的纯直线)，赋予一个大数值
                }
                
                cam->aspect_ratio[valid_idx] = ratio; // 记录该灯的长宽比
                // ==========================================
                
                valid_idx++;
            }
        }
    }
    cam->light_number = valid_idx;


}


// --- 4. 外部调用的处理入口 ---
void image_processing_loop(void) {
    // 1. 二值化
    binarize_image(&cam_down);
    //膨胀处理
    fast_dilate_3x3_8_neighbor(&cam_down);
    // 2. 连通域标记
    mark_components(&cam_down, visited_buffer);
    
    // 3. 计算质心
    calculate_centroids(&cam_down, visited_buffer);

    // 3. 按面积从大到小排序 (需同步交换 aspect_ratio)
    
    sort_lights(&cam_down);

    // 4. 矫正处理 share_data_from_0: [0]=Roll, [1]=Pitch, [3]=Height
    extern float share_data_from_0[];
    calculate_ground_positions(share_data_from_0[3], share_data_from_0[1], share_data_from_0[0]);
} 

void image_send(void){
    if(mt9v03x_finish_flag)
    {
        mt9v03x_finish_flag = 0;
        //遍历赋值并放大显示 (1变255)
        for(int i = 0; i < MT9V03X_H * MT9V03X_W; i++)
        {
            // 如果是1则变为255(白)，如果是0保持0(黑)
            image_copy[0][i] = cam_down.raw_image[i]; 
        }

        // 发送图像
        seekfree_assistant_camera_send();
    }
}