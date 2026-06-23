#ifndef _IMU_H
#define _IMU_H

#include "zf_common_headfile.h"
#include <math.h>
#include <stdint.h>
#include "filters.h"

// ================= 配置参数 =================
#define KP 0.93f          // 互补滤波比例增益
#define KI 0.0015f        // 互补滤波积分增益
#define DT 0.001f        // 运行周期 1ms (1000Hz)
#define GRAVITY_MSS 9.789f // 标准重力加速度
#define VALID_G_MIN 0.2f

// ================= 坐标系映射宏定义 =================
// 目标: NED坐标系 (X前, Y右, Z下)
// 传感器原始数据: x, y, z
// 请根据实际安装方向修改以下宏
// 当前安装: 传感器X向前, Y向右, Z向下

// 陀螺仪映射 (机体角速度)
// 规则: 符合右手定则直接映射，轴向相反则取负
#define IMU_MAP_GX(x, y, z)  (x)       // Roll  = Gz
#define IMU_MAP_GY(x, y, z)  (y)    // Pitch = Gy
#define IMU_MAP_GZ(x, y, z)  (z)       // Yaw   = Gx

// 加速度计映射 (重力向量, 即 -1 * 机体加速度)
// 注意: Mahony算法需要重力向量方向(指向地心)
// 规则: 重力向量 = -1 * 加速度计读数 (因为加速度计测量的是支撑力)
// 推导: G_body = R_sensor_to_body * (-Acc_sensor)
#define IMU_MAP_AX(x, y, z)  (-(x))
#define IMU_MAP_AY(x, y, z)  (-(y))
#define IMU_MAP_AZ(x, y, z)  (-(z))

// ================= 硬件安装误差补偿 (度) =================
#define IMU_MOUNT_ADJUST_ROLL   0.0f //直接填写水平飞行的读数
#define IMU_MOUNT_ADJUST_PITCH  0.0f

// ================= 陷波滤波参数 (电机振动抑制) =================
#define NOTCH_ENABLE            0       // 1: 使能陷波滤波, 0: 关闭
#define NOTCH_FS                1000.0f // 采样频率 (Hz) = 1/DT
#define NOTCH_Q                 5.0f    // 品质因数 (越高越窄, 建议3~10)
#define NOTCH_MIN_FREQ          5.0f    // 最低陷波频率 (Hz), 低于此值自动旁通
#define NOTCH_MAX_FREQ          (NOTCH_FS * 0.48f) // 最高陷波频率 (Hz), 超过防Nyquist折叠
#define NOTCH_MOTOR_COUNT       4       // 电机数量
#define NOTCH_HARMONIC_COUNT    3       // 每电机谐波数 (1×/2×/6×), 级联 4×3=12 个biquad/通道
#define NOTCH_TIMEOUT_MS        20      // 转速数据超时 (ms), 超时后旁通所有陷波

// 通道掩码: bit0=gx, bit1=gy, bit2=gz, bit3=ax, bit4=ay, bit5=az
#define NOTCH_CHANNEL_GX    0x01
#define NOTCH_CHANNEL_GY    0x02
#define NOTCH_CHANNEL_GZ    0x04
#define NOTCH_CHANNEL_AX    0x08
#define NOTCH_CHANNEL_AY    0x10
#define NOTCH_CHANNEL_AZ    0x20
#define NOTCH_CHANNEL_MASK  (NOTCH_CHANNEL_GX | NOTCH_CHANNEL_GY | NOTCH_CHANNEL_GZ)  // 仅陀螺

#include "filters.h"   // KalmanFilter1, NotchFilter_t, NotchConfig_t 等通用滤波器定义

extern const NotchConfig_t notch_cfg;       // IMU 专用陷波配置 (fs=1000, q=5, min=5Hz, max=480Hz)
extern uint8_t notch_active_count;          // 调试: 当前生效的陷波切片数 (NOTCH_ENABLE时有效)

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
// [新增] 专门用于确认 IMU 方向和数据的打印函数
//void IMU_Check_Data_Print(void);
void IMU_Check_Data_Print(void);
#endif