#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_

#include "zf_common_headfile.h"

#define IMG_CENTER_X (MT9V03X_W / 2.0f)
#define IMG_CENTER_Y (MT9V03X_H / 2.0f)
#define CAM_CX 95.5754542f
#define CAM_CY 56.0345163f
#define CAM_A0 53.5431129f
#define CAM_A2 -0.0157824f
#define CAM_A3 0.00027861f
#define CAM_A4 -0.0000044f
// 宏定义
#define STACK_SIZE 4096     // DFS 栈大小
#define MAX_LIGHTS 20       // 最大识别灯光数量
#define THRESHOLD 130      //二值化阈值设置
#define THRESHOLD_MAX 130   // 动态阈值上限 (近距离)
#define THRESHOLD_MIN  40   // 动态阈值下限 (5m水平距离)

// =========================================================
// [新增] 广角全景摄像头有效成像圆形区域配置
// =========================================================
#define FOV_DIAMETER 126.0f     // 视野有效圆直径
#define FOV_RADIUS (FOV_DIAMETER / 2.0f)
#define FOV_RADIUS_SQ (FOV_RADIUS * FOV_RADIUS)

// =========================================================
// [新增] 面积动态补偿参数 (解决边缘灯光变小的问题)
// =========================================================
// 1. 最小面积 (灯必须大于这个面积才算有效)
#define BASE_MIN_AREA 2.0f

// =========================================================
// [新增] 信标 (圆形灯) 动态透视畸变补偿参数
// 逻辑：画面中心卡得很严，越靠近画面边缘容错越大，但有绝对上限
// =========================================================
// 1. 基础上限：信标在画面正中心时允许的最大长宽比 (此时几乎没畸变，卡严一点)
#define TARGET_BASE_MAX_RATIO   2.0f

// 2. 畸变补偿系数：目标质心偏离画面中心距离的平方，每增加 1 个单位，上限放宽多少
// 提示：188x120 屏幕角落距离中心的平方大概是 (94^2 + 60^2) ≈ 12436。
// 如果系数是 0.00005，角落里最大允许长宽比就是 2.0 + 12436*0.00005 ≈ 2.62
#define TARGET_RATIO_COMP_COEF  0.00025f

// 3. 绝对上限：就算偏离到屏幕最边缘，长宽比也不能超过这个值 (防止把真正的小车当成信标)
#define TARGET_LIMIT_MAX_RATIO  5.0f

#define CAR_BASE_MIN_RATIO      3.2f     // 中心基础下限：在中心时长宽比大于 3.0 即认为是小车
#define CAR_RATIO_COMP_COEF     0.00006f  // 补偿系数：假设边缘距离平方约 12000，12000*0.0002=2.4。边缘门槛会提升到 3.0+2.4 = 5.4

#define EDGE_SAFE_MARGIN_X 15.0f
#define EDGE_SAFE_MARGIN_Y 0.0f

#define K_Y 1.11f
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

    uint8_t car_valid;           // 是否锁定小车 (1:是, 0:否)
    float car_center_y;          // 小车 Row (Y)
    float car_center_x;          // 小车 Col (X)
    uint32_t car_dot_num;           // 小车面积
    float car_ratio;

    uint8_t target_valid;        // 是否锁定信标 (1:是, 0:否)
    float target_center_y;       // 信标 Row (Y)
    float target_center_x;       // 信标 Col (X)
    uint32_t target_dot_num;        // 信标面积
    float target_ratio;

    float debug_max_ratio;       // 历史记录的最大长宽比
    float debug_min_ratio;       // 历史记录的最小长宽比

} CameraObject;

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float height;
} Image_IMU_Snapshot_t;

// 声明全局下视摄像头实例
extern CameraObject cam_down; 
extern uint8 image_copy[MT9V03X_H][MT9V03X_W];
extern Image_IMU_Snapshot_t img_imu_snap;

// 函数声明
void camera_init(void);           // 初始化
void image_processing_loop(void); // 图像处理主循环
#endif 