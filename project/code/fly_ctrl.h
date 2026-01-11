#ifndef _FLIGHT_CTRL_H
#define _FLIGHT_CTRL_H

#include "zf_common_headfile.h"

//引脚定义
#define PWM_LF             (TCPWM_CH11_P05_2)
#define PWM_LB             (TCPWM_CH20_P08_1)
#define PWM_RF             (TCPWM_CH30_P10_2)
#define PWM_RB             (TCPWM_CH58_P17_3)
//引脚定义

typedef struct {
    float x; // 对应图像的 Col 方向 (无人机右)
    float y; // 对应图像的 Row 方向 (无人机前，注意图像Row0在上面，所以需要取反)
} Vector2D;

//--------------------飞行状态----------------------//
typedef enum {
    normal,//正常巡飞
    pre_landing,//降低高度，到达LAND_HEIGHT后转为landing
    landing,//熄火
    brake//紧急停止
} STATE;

// =================== 飞行参数配置 ===================
#define TARGET_HEIGHT_CM 80.0f// 目标高度 (cm)
#define LAND_HEIGHT 15.0f  //着陆时熄火高度
#define HOVER_THROTTLE      3750    // 悬停油门
#define MAX_PWM             4600    // 最大PWM
#define MIN_PWM             0    // 最小PWM
#define MAX_TILT_ANGLE      20.0f   // 最大倾角
#define CTRL_DT             0.02f  // 控制周期 1ms (原代码宏定义为 DT，建议改名防止冲突)
#define POS_P_GAIN 0.8f     //从像素点误差到速度的乘子


#define IMG_CENTER_X (MT9V03X_W / 2.0f) // 94
#define IMG_CENTER_Y (MT9V03X_H / 2.0f) // 60
// 5 - 10 -> 0% - 100%

// =================== 控制目标结构体 ===================
typedef struct {
    float vel_x_cm_s;
    float vel_y_cm_s;
    float yaw_rate;
    float height;
    float target_height;
    float target_yaw;   // 新增：无人机期望的绝对偏航角 (度)
    
    uint8_t is_armed;
    STATE cur_state;
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
void motor_pwm_init(void);
void Air_Ground_Control_Loop(float car_angle_deg);
#endif