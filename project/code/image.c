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
    if (cam->light_number < 2) return; // 少于2个不用排

    for (int i = 0; i < cam->light_number - 1; i++) {
        for (int j = 0; j < cam->light_number - 1 - i; j++) {
            // 如果后一个比前一个大，交换
            if (cam->dot_num[j] < cam->dot_num[j+1]) {
                // 1. 交换像素数
                uint32_t temp_num = cam->dot_num[j];
                cam->dot_num[j] = cam->dot_num[j+1];
                cam->dot_num[j+1] = temp_num;

                // 2. 交换坐标 (Row/Y)
                float temp_row = cam->centers[j][0];
                cam->centers[j][0] = cam->centers[j+1][0];
                cam->centers[j+1][0] = temp_row;

                // 3. 交换坐标 (Col/X)
                float temp_col = cam->centers[j][1];
                cam->centers[j][1] = cam->centers[j+1][1];
                cam->centers[j+1][1] = temp_col;
            }
        }
    }
}







// 提取质心 (已修改)
static void calculate_centroids(CameraObject *cam, uint8_t *visited) {
    // [修复] 数组大小必须匹配最大连通域数量(MAX_DOTS)，否则 lbl > 20 时会越界崩溃
    uint32_t sum_r[MAX_DOTS] = {0};
    uint32_t sum_c[MAX_DOTS] = {0};
    
    // 清空上一帧结果
    memset(cam->dot_num, 0, sizeof(cam->dot_num));
    memset(cam->centers, 0, sizeof(cam->centers));

    // 1. 累加坐标
    for (uint16_t i = cam->margin_cut; i < cam->height - cam->margin_cut; i++) {
        for (uint16_t j = cam->margin_cut; j < cam->width - cam->margin_cut; j++) {
            uint8_t lbl = visited[i * cam->width + j];
            if (lbl > 0 && lbl <= MAX_DOTS) { 
                sum_r[lbl-1] += i;
                sum_c[lbl-1] += j;
                cam->dot_num[lbl-1]++;
            }
        }
    }

    // 2. 计算平均值并初步筛选
    uint8_t valid_idx = 0;
    for (int i = 0; i < cam->components_count && i < MAX_DOTS; i++) {
        if (cam->dot_num[i] > MIN_LIGHT_SIZE) {
            if (valid_idx < MAX_LIGHTS) {
                cam->centers[valid_idx][0] = (float)sum_r[i] / (float)cam->dot_num[i]; // Row (Y)
                cam->centers[valid_idx][1] = (float)sum_c[i] / (float)cam->dot_num[i]; // Col (X)
                cam->dot_num[valid_idx] = cam->dot_num[i]; 
                
                valid_idx++;
            }
        }
    }
    cam->light_number = valid_idx;

    // 3. [新增] 按面积从大到小排序
    sort_lights(cam);
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

    // 4. 矫正处理
    //calculate_ground_positions(m7_1_data[6], m7_1_data[4], m7_1_data[3]); //此步骤移至main_cm7_1.c
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