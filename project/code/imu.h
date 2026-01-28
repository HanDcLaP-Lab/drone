#ifndef _IMU_H
#define _IMU_H

#include "zf_common_headfile.h"
#include <math.h>
#include <stdint.h>

// ================= 配置参数 =================
#define KP 0.93f          // 互补滤波比例增益
#define KI 0.0015f        // 互补滤波积分增益
#define DT 0.001f        // 运行周期 1ms (1000Hz)
#define GRAVITY_MSS 9.789f // 标准重力加速度
#define VALID_G_MIN 0.2f

// ================= 硬件安装误差补偿 (度) =================
#define IMU_MOUNT_ADJUST_ROLL   0.0f //直接填写水平放置的读数
#define IMU_MOUNT_ADJUST_PITCH  0.0f

// ================= Z轴融合参数 =================
#define Z_CORRECT_POS_GAIN  0.3f   // 位置修正系数
#define Z_CORRECT_VEL_GAIN  0.3f   // 速度修正系数

#ifndef PI
#define PI 3.1415926535f
#endif

// ================= 核心结构体定义 =================
typedef struct {
    // --- 姿态角 单位: 度、地球系
    float roll;
    float pitch;
    float yaw;
    //姿态角速度 单位：度/秒、飞机系
    float groll;
    float gpitch;
    float gyaw;

    // --- 空间位置 (单位: cm) ---
    float x;
    float y;
    float z; // 仅 Z 轴有效

    // --- 空间速度  ---
    float vx;
    float vy;
    float vz; // cm/s

    // --- 运动加速度 (单位: m/s^2) ---
    float world_ax; // 用于判断方向
    float world_ay;
    float world_az;

    // --- 状态标志 ---
    uint8_t is_calibrated; 
} IMU_Data_t;

extern IMU_Data_t imu_data;

// ================= 函数声明 =================
void IMU_Update_Loop(void);
void imu_init(void);
void tof_init(void);
// [新增] 专门用于确认 IMU 方向和数据的打印函数
//void IMU_Check_Data_Print(void);
void IMU_Check_Data_Print(void);
#endif