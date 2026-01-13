#ifndef _FLIGHT_CTRL_H
#define _FLIGHT_CTRL_H

#include "zf_common_headfile.h"

//引脚定义
#define PWM_LF             (TCPWM_CH11_P05_2)
#define PWM_LB             (TCPWM_CH20_P08_1)
#define PWM_RF             (TCPWM_CH30_P10_2)
#define PWM_RB             (TCPWM_CH58_P17_3)

// =================== 飞行参数配置 ===================
#define TARGET_HEIGHT_CM    80.0f   // 目标高度
#define LAND_HEIGHT         15.0f   // 着陆高度
#define HOVER_THROTTLE      3750    // 基础悬停油门 (需根据电池电压调整)
#define MAX_PWM             4600    
#define MIN_PWM             0    
#define MAX_TILT_ANGLE      30.0f   // 最大倾角限制 (度) - 既然没速度环，这个限制很重要
#define CTRL_DT_CTLOOP             0.001  // 控制周期 
#define CTRL_DT_CTANG       0.005 //控制周期

// 视觉控制增益
#define VISUAL_POS_P_GAIN   1.0f   // 像素误差 -> 角度 

#define IMG_CENTER_X (MT9V03X_W / 2.0f) 
#define IMG_CENTER_Y (MT9V03X_H / 2.0f) 

#define VALID_MIN_NUM 300

//--------------------飞行状态----------------------//
typedef enum {
    normal,      // 正常飞行
    pre_landing, // 准备降落
    landing,     // 降落中
    brake        // 停机
} STATE;


// ================= 视觉补偿参数 =================
// 基于 MT9V03X + 2.1mm 广角镜头的估算值 (188x120分辨率)
// 理论值约 87-88。如果发现补偿不足（晃动飞机时目标乱跑），调小此值；补偿过度，调大此值。
#define CAM_F_PIXEL        88.0f   

// [方向符号修正] 保持不变，需实测
#define SIGN_PITCH_COMP    1.0f    
#define SIGN_ROLL_COMP     -1.0f
// =================== 控制目标结构体 ===================
typedef struct {
    // --- 姿态目标 (直接控制量) ---
    float target_roll;   // 期望横滚角 (度)
    float target_pitch;  // 期望俯仰角 (度)
    float target_yaw;    // 期望偏航角 (度)
    // --- 姿态目标(串级控制量)
    float target_g_roll;
    float target_g_pitch;
    float target_g_yaw;
      

    // --- 高度目标 ---
    float height;        // 内部平滑后的当前高度目标
    float target_height; // 最终期望高度

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
void Flight_Control_Angle(void);
void Flight_Control_Loop(void);
// 新的控制接口：直接设定目标姿态
void Set_Target_Attitude(float roll, float pitch, float yaw); 
void Flight_Unlock(void);
void Flight_Lock(void);
void motor_pwm_set(void);
void motor_pwm_init(void);

// 视觉/上层逻辑
void Air_Ground_Control_Loop(float car_angle_deg);
void Simple_Hover_Control(void);
void Air_Ground_Control_Loop_New(float car_angle_deg);
//-----调试打印函数----//
void Debug_Motor_Output_Print(void);
void Get_Attitude_Compensated_Error(float raw_row, float raw_col, float *out_err_pitch_deg, float *out_err_roll_deg);
#endif