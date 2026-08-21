#ifndef _FLY_CTRL_H
#define _FLY_CTRL_H

#include "zf_common_headfile.h"


// =================== 飞行参数配置 ===================
#define TARGET_HEIGHT_CM 130.0f  // 目标高度
#define LANDING_DESCENT_TIME_MS 7000U // 目标高度从当前值线性降至0的时间
#define FLIGHT_TIMEOUT_MS       70000U // [新增] 全局飞行超时 (ms)，超时自动降落
#define LANDING_CUTOFF_HEIGHT_CM 30.0f  // 新ToF帧低于此高度时触发触地关停
#define LANDING_CUTOFF_RAMP_MS  4000U  // 触地后 PWM scale 线性缩小到 0 的时间 (ms)
#define MAX_PWM 8500
#define MIN_PWM 800
#define MAX_TILT_ANGLE 15.0f  // 最大计算倾角限制 (度)
#define MAX_REAL_ANGLE 40.0f // 最大实际倾角限制 (度) 超过停机
#define CTRL_DT_CTLOOP 0.00125f  // 飞控控制周期 (1.25ms, 800Hz)

// ================= 最外环积分限幅配置 =================
#define IMAGE_PID_MAX_I_NORMAL       50.0f   // 视觉位置外环正常积分限幅
#define IMAGE_PID_MAX_I_CALIB        150.0f  // 姿态校准完成前放大的积分限幅 (用于抗稳态偏置定点)

#define MIN_ERROR 0.0f  //飞机跟踪小车的死区设置
//扫描旋转
#define SEARCH_YAW_RATE 15.0f  // 搜索角速度 (度/秒)
#define TWO_MAX_YAW_DEV   100.0f  //计算出的最大偏航角

#define YAW_OFFSET 6.0f    //为了防止信标被线挡住，让无人机偏过的角度
#define TARGET_ACC_DISTANCE 100.0f    //为了防止无人机与信标很近时yaw变化大，角度跟踪的最小距离
#define YAW_MIN_ERROR  5.0f    //角度跟踪设置的小死区

// ================= 小车固定悬停点视觉补偿 =================
// X前Y右，单位cm；数值是在无人机IMU yaw等于CAM_OFFSET_MEASURE_YAW_DEG时测得的机体系坐标。
#define CAM_OFFSET_X                  (-1.0f)
#define CAM_OFFSET_Y                  (-2.0f)
#define LANDING_CAM_OFFSET_Y_DELTA   (-30.0f)
// 该yaw以无人机上电朝向为0；使用时会与视觉坐标一同转换到小车固定地面系。
#define CAM_OFFSET_MEASURE_YAW_DEG     0.0f

// ================= 高度保护参数 =================
#define VISION_POSITION_MIN_HEIGHT_CM       40.0f // 低于此高度才让视觉坐标失效
#define VISION_POSITION_HYSTERESIS_CM       5.0f  // [新增] 视觉/光流模式切换滞回，防止阈值附近反复切换
#define VISION_LOW_HEIGHT_HOLD_FRAMES       10U   // 图像约100Hz，10帧约100ms
#define CAR_ENABLE_MIN_HEIGHT_CM            70.0f // 低于此高度下传car_en=0
#define MODE_SWITCH_SMOOTH_MS               150U  // 视觉/光流模式切换时目标倾角平滑时间 (ms)

// =================== 飞行状态与模式定义 ===================
typedef enum {
    FLIGHT_STATE_NORMAL = 0, // 正常飞行
    FLIGHT_STATE_PRE_LANDING,// 准备降落
    FLIGHT_STATE_LANDING     // 降落中
} Flight_State_e;

// 兼容旧枚举命名
typedef Flight_State_e STATE;
#define normal       FLIGHT_STATE_NORMAL
#define pre_landing  FLIGHT_STATE_PRE_LANDING
#define landing      FLIGHT_STATE_LANDING

typedef enum {
    ARM_STATE_DISARMED = 0,         // 锁定
    ARM_STATE_ARMED = 1,            // 已解锁
    ARM_STATE_WAITING_IMU_CALIB = 2 // 等待IMU校准后自动解锁
} Arm_State_e;

typedef enum {
    NAV_MODE_ATTITUDE_HOLD = 0, // 基础姿态/回平自稳 (无有效外环或失联)
    NAV_MODE_OPTICAL_FLOW,      // 低高度光流速度定点外环
    NAV_MODE_VISION_HOVER       // 高高度视觉位置/小车站位外环
} Nav_Mode_e;

// =================== 控制目标结构体 ===================
typedef struct {
    // --- 姿态目标 (直接控制量) ---
    float target_roll;   // 期望横滚角 (度) [由视觉/悬停任务设定]
    float target_pitch;  // 期望俯仰角 (度) [由视觉/悬停任务设定]
    float target_yaw;    // 期望偏航角 (度)
    // --- 姿态目标(串级控制量)
    float target_g_roll;  //期望角速度 （度 / 秒）[由角度环PID计算得出]
    float target_g_pitch; //期望角速度 （度 / 秒）[由角度环PID计算得出]
    float target_g_yaw;   //期望角速度 （度 / 秒）[由角度环PID计算得出]

    // --- 高度目标 ---
    float height;         // 内部平滑后的当前高度目标
    float target_height;  // 最终期望高度

    uint8_t is_armed;
    Flight_State_e cur_state;
    float start_up_scale;
    float output_scale;
} Flight_Target_t;

// ===================== 输出结构体 =====================
typedef struct {
    float roll; //由roll带来的最终输出量
    float pitch;
    float yaw;

    int16_t rf;  // 右前
    int16_t rb;  // 右后
    int16_t lb;  // 左后
    int16_t lf;  // 左前
} Motor_Output_t;  //此项于fly_ctrl.c初始化为0

// =================== 全局变量 ===================
extern Flight_Target_t flight_target;
extern Motor_Output_t motor_out;
// =================== 函数声明 ===================
void Flight_Control_Init(void);
void Flight_Control_Angle(void);
void Flight_Control_Loop(void);
// 导航模式更新与查询
Nav_Mode_e Flight_Nav_Mode_Update(float height_cm);
Nav_Mode_e Flight_Get_Nav_Mode(void);

// 控制接口：直接设定目标姿态与带平滑的目标姿态
void Set_Target_Attitude(float roll, float pitch, float yaw);
void Flight_Set_Target_Attitude_Smoothed(float roll, float pitch, float yaw, float dt_sec);
void Flight_Attitude_Smoother_Reset(void);
void Flight_Request_Landing(void);
void Flight_Unlock(void);
void Flight_Lock(void);
void motor_pwm_set(void);

#endif
