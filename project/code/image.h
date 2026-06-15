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
#define FOV_DIAMETER 127.5f     // 视野有效圆直径
#define FOV_RADIUS (FOV_DIAMETER / 2.0f)
#define FOV_RADIUS_SQ (FOV_RADIUS * FOV_RADIUS)

#define MORPH_MASK_DIAMETER 100.0f  // [新增] 形态学有效区域圆直径(限制边缘噪声膨胀)
#define MORPH_MASK_RADIUS (MORPH_MASK_DIAMETER / 2.0f)
#define MORPH_MASK_RADIUS_SQ (MORPH_MASK_RADIUS * MORPH_MASK_RADIUS)

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
#define CAR_IDEAL_MAX_RATIO     25.0f    // [新增] 小车理想长宽比上限 (用于得分截断，防线缆高倍率加分)
#define CAR_ABSOLUTE_MAX_RATIO  1000.0f    // [未启用] 小车绝对长宽比红线 (超过此值直接视为细长线缆剔除)

#define EDGE_SAFE_MARGIN_X 15.0f
#define EDGE_SAFE_MARGIN_Y 0.0f

#define K_Y 1.11f

// ================= 形态学参数 =================
#define ERODE_MIN_NEIGHBORS     7       // 腐蚀: 8邻域至少保留此数亮像素

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

// ================= 信标选取迟滞 (防双信标震荡) =================
// 原理：若当前候选与上一帧选中信标的地面位置接近（同一信标），在擂台比较时
// 获得等效距离优惠，避免因微小距离变化（drone 微动/图像噪声）导致帧间选取翻转。
#define HYSTERESIS_MATCH_RADIUS_SQ 900.0f   // 30cm² — 同一信标的匹配半径平方
#define HYSTERESIS_DIST_BIAS       50.0f    // 等效距离优惠(cm) — 旧目标获得 50cm 容差
// --- 摄像头对象结构体 ---
typedef struct {
    // --- 基础属性 ---
    uint16_t width;             
    uint16_t height;            
    uint8_t *raw_image;         
    uint8_t *binarized_image;   

    // --- 算法参数 ---
    uint8_t threshold;          

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
#endif 