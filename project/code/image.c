#include "image.h"

// --- 1. 内存分配 ---
// 定义二值化图像缓冲区 (只定义一个)
uint8_t buffer_bin_down[MT9V03X_H][MT9V03X_W];
// 定义 DFS 访问标记数组 (静态分配以防栈溢出)
uint8_t visited_buffer[MT9V03X_H * MT9V03X_W];

// 定义全局实例
CameraObject cam_down;

// --- 2. 初始化函数 ---
void camera_init(void) {
    // 初始化下视摄像头
    cam_down.width = MT9V03X_W;
    cam_down.height = MT9V03X_H;
    
    // 指向库文件的图像数组 (直接使用逐飞库的 DMA 缓冲区)
    cam_down.raw_image = (uint8_t *)mt9v03x_image;      
    cam_down.binarized_image = (uint8_t *)buffer_bin_down;
    
    // 参数配置
    cam_down.threshold = 100;   // 二值化阈值 (需根据实际场地光照调整)
    cam_down.margin_cut = 5;    // 四周裁剪 5 像素
}

// --- 3. 内部辅助函数 ---

// 坐标合法性检查
// 检查：1.是否在有效区域(去除边缘) 2.是否未访问 3.是否是亮点
static uint8_t is_valid_pixel(CameraObject *cam, uint16_t r, uint16_t c, uint8_t *visited) {
    uint32_t index = r * cam->width + c;
    
    return (r >= cam->margin_cut && r < (cam->height - cam->margin_cut) && 
            c >= cam->margin_cut && c < (cam->width - cam->margin_cut) && 
            visited[index] == 0 && 
            cam->binarized_image[index] == 1);
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

// 提取质心
static void calculate_centroids(CameraObject *cam, uint8_t *visited) {
    uint32_t sum_r[MAX_LIGHTS] = {0};
    uint32_t sum_c[MAX_LIGHTS] = {0};
    
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

    // 2. 计算平均值并筛选
    uint8_t valid_idx = 0;
    for (int i = 0; i < cam->components_count && i < MAX_DOTS; i++) {
        // 筛选条件：像素点数量 > 5 (对于无人机远距离，灯光可能较小，阈值可调小)
        if (cam->dot_num[i] > 5) {
            if (valid_idx < MAX_LIGHTS) {
                // 计算质心
                cam->centers[valid_idx][0] = sum_r[i] / cam->dot_num[i]; // Row (Y)
                cam->centers[valid_idx][1] = sum_c[i] / cam->dot_num[i]; // Col (X)
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
    
    // 2. 连通域标记
    mark_components(&cam_down, visited_buffer);
    
    // 3. 计算质心
    calculate_centroids(&cam_down, visited_buffer);
}