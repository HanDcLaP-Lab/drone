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

#define ROI_DISTANCE 10.0f
#define CAR_MIN_RATIO 4.5f

// =========================================================
// [新增] 信标 (圆形灯) 动态透视畸变补偿参数
// 逻辑：画面中心卡得很严，越靠近画面边缘容错越大，但有绝对上限
// =========================================================
// 1. 基础上限：信标在画面正中心时允许的最大长宽比 (此时几乎没畸变，卡严一点)
#define TARGET_BASE_MAX_RATIO   1.8f

// 2. 畸变补偿系数：目标质心偏离画面中心距离的平方，每增加 1 个单位，上限放宽多少
// 提示：188x120 屏幕角落距离中心的平方大概是 (94^2 + 60^2) ≈ 12436。
// 如果系数是 0.00005，角落里最大允许长宽比就是 2.0 + 12436*0.00005 ≈ 2.62
#define TARGET_RATIO_COMP_COEF  0.00012f

// 3. 绝对上限：就算偏离到屏幕最边缘，长宽比也不能超过这个值 (防止把真正的小车当成信标)
#define TARGET_LIMIT_MAX_RATIO  3.22f

#define TARGET_MAX_RATIO 2.5f
#define K_Y 1.2f
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

    // --- 处理结果 (纯净的原始数据，绝不覆写) ---
    uint8_t light_number;            
    float centers[MAX_LIGHTS][2];    
    uint32_t dot_num[MAX_LIGHTS];    // 【修正】统一改为 MAX_LIGHTS 防止越界
    float aspect_ratio[MAX_LIGHTS];
    uint8_t components_count;        

    // ==========================================
    // [新增] --- 追踪结果输出 (外部业务只读以下变量) ---
    // ==========================================
    uint8_t car_valid;           // 是否锁定小车 (1:是, 0:否)
    float car_center_y;          // 小车 Row (Y)
    float car_center_x;          // 小车 Col (X)
    uint32_t car_area;           // 小车面积
    float car_ratio;

    uint8_t target_valid;        // 是否锁定信标 (1:是, 0:否)
    float target_center_y;       // 信标 Row (Y)
    float target_center_x;       // 信标 Col (X)
    uint32_t target_area;        // 信标面积
    float target_ratio;

    float debug_max_ratio;       // 历史记录的最大长宽比
    float debug_min_ratio;       // 历史记录的最小长宽比

} CameraObject;

// 声明全局下视摄像头实例
extern CameraObject cam_down; 
extern uint8 image_copy[MT9V03X_H][MT9V03X_W];

// 函数声明
void camera_init(void);           // 初始化
void image_processing_loop(void); // 图像处理主循环
void image_send(void);  //发送图像
#endif 