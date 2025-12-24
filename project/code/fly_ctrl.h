#ifndef _FLIGHT_CTRL_H
#define _FLIGHT_CTRL_H

#include "zf_common_headfile.h"

//引脚定义
#define PWM_LF             (TCPWM_CH11_P05_2)
#define PWM_LB             (TCPWM_CH20_P08_1)
#define PWM_RF             (TCPWM_CH30_P10_2)
#define PWM_RB             (TCPWM_CH58_P17_3)
//引脚定义

//--------------------飞行状态----------------------//
typedef enum {
    normal,//正常巡飞
    pre_landing,//降低高度，到达LAND_HEIGHT后转为landing
    landing,//熄火
    brake//紧急停止
} STATE;

// =================== 飞行参数配置 ===================
#define TARGET_HEIGHT_CM 30.0f// 目标高度 (cm)
#define LAND_HEIGHT 10.0f  //着陆时熄火高度
#define HOVER_THROTTLE      2000    // 悬停油门
#define MAX_PWM             3000    // 最大PWM
#define MIN_PWM             0    // 最小PWM
#define MAX_TILT_ANGLE      20.0f   // 最大倾角
#define CTRL_DT             0.02f  // 控制周期 1ms (原代码宏定义为 DT，建议改名防止冲突)
#define CTRL_DT             0.02f  // 控制周期 1ms (原代码宏定义为 DT，建议改名防止冲突)
extern float duty_LF, duty_LB, duty_RF, duty_RB;
// 5 - 10 -> 0% - 100%

// =================== 控制目标结构体 ===================
typedef struct {
    float vel_x_cm_s;  // 目标 X 轴速度
    float vel_y_cm_s;  // 目标 Y 轴速度
    float yaw_rate;    // 目标偏航角速度
    uint8_t is_armed;  // 解锁状态
    STATE cur_state; //飞行状态
} Flight_Target_t;

// =================== 电机输出结构体 ===================
typedef struct {
    int16_t rf; // 右前
    int16_t rb; // 右后
    int16_t lb; // 左后
    int16_t lf; // 左前
} Motor_Output_t;

// =================== 全局变量 ===================
extern Flight_Target_t flight_target;
extern Motor_Output_t motor_out;

// =================== 函数声明 ===================
void Flight_Control_Init(void);
void Flight_Control_Loop(void);
void Set_Target_Velocity(float vx, float vy, float yaw_rate);
void Flight_Unlock(void);
void Flight_Lock(void);
void motor_pwm_set(void);
void motor_pwm_init(void);
void motor_pwm_set(void);
void motor_pwm_init(void);

#endif