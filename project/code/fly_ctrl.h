#ifndef _FLIGHT_CTRL_H
#define _FLIGHT_CTRL_H

#include "zf_common_headfile.h"

// 引脚定义
#define PWM_LF (TCPWM_CH11_P05_2)
#define PWM_LB (TCPWM_CH20_P08_1)
#define PWM_RF (TCPWM_CH30_P10_2)
#define PWM_RB (TCPWM_CH58_P17_3)

// =================== 飞行参数配置 ===================
#define TARGET_HEIGHT_CM 40.0f  // 目标高度
#define LAND_HEIGHT 10.0f       // 着陆高度
#define HOVER_THROTTLE 3500     // 基础悬停油门 
#define MAX_PWM 5800
#define MIN_PWM 0
#define MAX_TILT_ANGLE 3.0f  // 最大倾角限制 (度) 
#define CTRL_DT_CTLOOP 0.001  // 控制周期
#define CTRL_DT_CTANG 0.02   // 控制周期

// 视觉控制增益
#define ANGLE_COMP_COEF 1.40f  //位姿对像素的补偿

#define IMG_CENTER_X (MT9V03X_W / 2.0f)
#define IMG_CENTER_Y (MT9V03X_H / 2.0f)

#define VALID_MIN_NUM 300
#define ACCEPT_ERROR 6.0f
//视觉传输
#define DATA_LENGTH (3)
extern float comp_row;
extern float comp_col;

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
// 新的控制接口：直接设定目标姿态
void Set_Target_Attitude(float roll, float pitch, float yaw);
void Flight_Unlock(void);
void Flight_Lock(void);
void motor_pwm_set(void);
void motor_pwm_init(void);

// 视觉/上层逻辑
void M7_1_data_send(float* M7_1_data,float* uart_data);
/**
 * @brief 自动悬停控制逻辑封装
 * @note 内部处理视觉补偿、姿态设定及 PID 计算
 */
void Flight_Hover_Control_Task(void);
// [code/fly_ctrl.h] 在 "函数声明" 区域添加
void Fly_Param_Update(uint8_t ch, float val);
#endif