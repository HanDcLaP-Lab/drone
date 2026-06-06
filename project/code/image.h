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
#define THRESHOLD_MIN  5    // 动态阈值下限 (5m水平距离)

// =========================================================
// [新增] 广角全景摄像头有效成像圆形区域配置
// =========================================================
#define FOV_DIAMETER 126.0f     // 视野有效圆直径
#define FOV_RADIUS (FOV_DIAMETER / 2.0f)
#define FOV_RADIUS_SQ (FOV_RADIUS * FOV_RADIUS)

#define CAR_MAX_DISTANCE  180.0f  //小车最大距离，超过不认为是小车
#define TARGET_MAX_DISTANCE  1000.0f  //信标最大距离，超过不认为是小车
// =========================================================
// [新增] 面积动态补偿参数 (解决边缘灯光变小的问题)
// =========================================================
// 1. 最小面积 (灯必须大于这个面积才算有效)
#define BASE_MIN_AREA 1.0f

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

// ================= 形态学参数 =================
#define ERODE_MIN_NEIGHBORS     6       // 腐蚀: 8邻域至少保留此数亮像素

// ================= 距离估算 =================
#define HEIGHT_ESTIMATE_MIN     30.0f   // 距离估算最低高度 (cm)
#define DIST_COMP_THRESHOLD     150.0f  // 距离补偿起效距离 (cm)
#define DIST_COMP_SCALE         100.0f  // 距离补偿基准距离 (cm)

// ================= 小车识别 =================
#define CAR_MAX_CENTER_DIST_SQ  3600.0f // 小车距画面中心最大距离平方 (60²)

// ================= 信标识别 =================
#define SMALL_BLOB_DIRECT_AREA  20      // 小光斑面积上限 (≤此值直接通过形状筛选)
#define DEGENERATE_RATIO_MARK   99.0f   // 退化标记排除值 (ratio==100视为无效)
#define TARGET_MIN_CONSECUTIVE_FRAMES 5 // 连续检测到信标多少帧后允许保持
#define TARGET_HOLD_FRAMES      5       // 丢失后保持最后位置的帧数
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

    // --- 调试字段 (ImageDebug_t, 不参与控制逻辑) ---
    struct {
        uint8_t raw_blobs;          // 原始检测到的连通域数 (components_count 快照)
        uint8_t pass_area;          // 通过面积+物理距离过滤的光斑数
        uint8_t pass_car;           // 满足小车长宽比门槛的候选项数
        uint8_t pass_target;        // 满足信标长宽比门槛的候选项数
        float   max_ratio;          // 本帧最大长宽比
        float   min_ratio;          // 本帧最小长宽比 (排除退化标记值)
    } debug;

    uint32_t max_area;           // [新增] 本帧最大连通域面积，用于融合检测
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

// 预计算的圆周内壁索引表（经过物理距离 > 10m 过滤，且 2 倍稀疏降采样）
// 供 clear_edge_blobs 极速清除边界污染使用
static const uint16_t border_indices[182] = {
    259, 261, 263, 265, 267, 302, 445, 492, 631, 682,
    817, 872, 1003, 1062, 1189, 1252, 1375, 1442, 1562, 1749,
    1935, 2010, 2011, 2122, 2309, 2496, 2682, 2767, 2869, 2957,
    3056, 3243, 3431, 3618, 3805, 3901, 3992, 4179, 4367, 4554,
    4741, 4845, 4928, 5116, 5303, 5491, 5678, 5788, 5866, 6053,
    6241, 6428, 6616, 6730, 6803, 6991, 7178, 7366, 7554, 7672,
    7741, 7929, 8117, 8305, 8492, 8614, 8680, 8868, 9056, 9243,
    9431, 9555, 9619, 9807, 9995, 10183, 10371, 10495, 10559, 10747,
    10935, 11123, 11311, 11436, 11499, 11687, 11875, 12063, 12251, 12375,
    12439, 12627, 12815, 13003, 13191, 13315, 13379, 13568, 13756, 13944,
    14132, 14254, 14321, 14509, 14697, 14885, 15074, 15192, 15262, 15450,
    15639, 15827, 16016, 16130, 16204, 16393, 16581, 16770, 16958, 17068,
    17147, 17335, 17524, 17712, 17901, 18005, 18090, 18279, 18467, 18656,
    18845, 18941, 19034, 19223, 19411, 19600, 19789, 19877, 19978, 20063,
    20168, 20357, 20546, 20735, 20810, 20925, 21114, 21303, 21370, 21493,
    21556, 21683, 21742, 21873, 21928, 22063, 22114, 22253, 22255, 22257,
    22259, 22261, 22263, 22265, 22267, 22269, 22271, 22273, 22275, 22277,
    22279, 22281, 22283, 22285, 22287, 22289, 22291, 22293, 22295, 22297,
    22299, 22301,
};
static const int border_pixel_count = 182;

#endif 