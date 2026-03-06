#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_

#include "zf_common_headfile.h"

// 宏定义
#define STACK_SIZE 4096     // DFS 栈大小
#define MAX_LIGHTS 20       // 最大识别灯光数量
#define MAX_DOTS 40         // 最大记录的连通域数量
#define THRESHOLD 120      //二值化阈值设置
#define MIN_LIGHT_SIZE  2   //灯最小判定大小

#define UART_DATA_LENGTH 8  // 数组数据长度
#define M7_1_DATA_LENGTH 16
// --- 摄像头对象结构体 ---
typedef struct {
    // --- 基础属性 ---
    uint16_t width;             // 图像宽度
    uint16_t height;            // 图像高度
    uint8_t *raw_image;         // 指向原始灰度图的指针
    uint8_t *binarized_image;   // 指向二值化图的指针

    // --- 算法参数 ---
    uint8_t threshold;          // 二值化阈值 (0-255)
    uint8_t margin_cut;         // 四周裁剪像素 (去除镜头边缘噪点)

    // --- 处理结果 ---
    uint8_t light_number;            // 识别到的有效灯数量
    float centers[MAX_LIGHTS][2];    // [修改] 灯光质心坐标改为 float 以提高精度
                                     // 注意: row对应图像垂直方向，col对应图像水平方向
    uint32_t dot_num[MAX_DOTS];      // 灯光像素点数 (面积)
    uint8_t components_count;        // 连通域数量

} CameraObject;

// 声明全局下视摄像头实例
extern CameraObject cam_down; 


// 函数声明
void camera_init(void);           // 初始化
void image_processing_loop(void); // 图像处理主循环
void image_send(void);  //发送图像
#endif 