#ifndef _FLY_CTRL_H
#define _FLY_CTRL_H

#include "zf_common_headfile.h"


// =================== 飞行参数配置 ===================
#define TARGET_HEIGHT_CM 120.0f  // 目标高度
#define LAND_HEIGHT 10.0f       // 着陆高度
#define HOVER_THROTTLE 5150    // 基础悬停油门 
#define MAX_PWM 8000
#define MIN_PWM 0
#define MAX_TILT_ANGLE 7.5f  // 最大计算倾角限制 (度)
#define MAX_REAL_ANGLE 40.0f // 最大实际倾角限制 (度) 超过停机
#define CTRL_DT_CTLOOP 0.00125f  // 飞控控制周期 (1.25ms, 800Hz)

#define MIN_ERROR 0.0f  //飞机跟踪小车的死区设置
//扫描旋转
#define SEARCH_YAW_RATE 15.0f  // 搜索角速度 (度/秒)
#define TWO_MAX_YAW_DEV   100.0f  //计算出的最大偏航角

#define YAW_OFFSET 6.0f    //为了防止信标被线挡住，让无人机偏过的角度
#define TARGET_ACC_DISTANCE 100.0f    //为了防止无人机与信标很近时yaw变化大，角度跟踪的最小距离
#define YAW_MIN_ERROR  5.0f    //角度跟踪设置的小死区

#define ROLL_OFFSET 338.0f       //补偿重心偏移
#define PITCH_OFFSET -240.0f     //补偿重心偏移
// ================= 小车固定悬停点视觉补偿 =================
// X前Y右，单位cm；数值是在无人机IMU yaw等于CAM_OFFSET_MEASURE_YAW_DEG时测得的机体系坐标。
#define CAM_OFFSET_X                  -2.5f
#define CAM_OFFSET_Y                  -9.5f
// 该yaw以无人机上电朝向为0；使用时会与视觉坐标一同转换到小车固定地面系。
#define CAM_OFFSET_MEASURE_YAW_DEG     0.0f


//--------------------飞行状态----------------------//
typedef enum {
    normal,       // 正常飞行
    pre_landing,  // 准备降落
    landing,      // 降落中
} STATE;

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
    STATE cur_state;
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

typedef struct {
    int16_t rf;  // 右前
    int16_t rb;  // 右后
    int16_t lb;  // 左后
    int16_t lf;  // 左前
} Motor_Offsset_t;  //此项于fly_ctrl.c初始化为0

// =================== 全局变量 ===================
extern Flight_Target_t flight_target;
extern Motor_Output_t motor_out;
extern Motor_Offsset_t motor_offset;
// =================== 函数声明 ===================
void Flight_Control_Init(void);
void Flight_Control_Angle(void);
void Flight_Control_Loop(void);
// 新的控制接口：直接设定目标姿态
void Set_Target_Attitude(float roll, float pitch, float yaw);
void Flight_Unlock(void);
void Flight_Lock(void);
void motor_pwm_set(void);

#endif
