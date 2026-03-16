#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_

#include "zf_common_headfile.h"

// 宏定义
#define STACK_SIZE 4096     // DFS 栈大小
#define MAX_LIGHTS 20       // 最大识别灯光数量
#define MAX_DOTS 10         // 最大记录的连通域数量
#define THRESHOLD 60      //二值化阈值设置
#define MIN_LIGHT_SIZE  12   //灯最小判定大小

#define UART_DATA_LENGTH 8  // 数组数据长度
#define M7_1_DATA_LENGTH 16

#define ROI_DISTANCE 10.0
#define RATIO 3.5
// --- 摄像头对象结构体 ---
// --- 摄像头对象结构体 ---
typedef struct {
    // --- 基础属性 ---
    uint16_t width;             
    uint16_t height;            
    uint8_t *raw_image;         
    uint8_t *binarized_image;   

    // --- 算法参数 ---
    uint8_t threshold;          
    uint8_t margin_cut;         

    // --- 处理结果 (原始提取数据) ---
    uint8_t light_number;            
    float centers[MAX_LIGHTS][2];    // 纯净的原始质心数据
    uint32_t dot_num[MAX_DOTS];      
    float aspect_ratio[MAX_LIGHTS];
    uint8_t components_count;        

    // ==========================================
    // [新增] --- 追踪输出结果 (外部代码只读以下变量) ---
    // ==========================================
    uint8_t car_valid;           // 是否识别到小车 (1:有效, 0:丢失)
    float car_center_x;          // 小车 X 坐标 (Col)
    float car_center_y;          // 小车 Y 坐标 (Row)
    uint32_t car_area;           // 小车面积

    uint8_t target_valid;        // 是否识别到信标 (1:有效, 0:丢失)
    float target_center_x;       // 信标 X 坐标 (Col)
    float target_center_y;       // 信标 Y 坐标 (Row)
    uint32_t target_area;        // 信标面积

} CameraObject;

// 声明全局下视摄像头实例
extern CameraObject cam_down; 
extern uint8 image_copy[MT9V03X_H][MT9V03X_W];

// 函数声明
void camera_init(void);           // 初始化
void image_processing_loop(void); // 图像处理主循环
void image_send(void);  //发送图像
#endif 