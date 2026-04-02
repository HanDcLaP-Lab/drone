#ifndef _FLIGHT_CTRL_H
#define _FLIGHT_CTRL_H

#include "zf_common_headfile.h"

// 引脚定义
#define PWM_LF (TCPWM_CH11_P05_2)
#define PWM_LB (TCPWM_CH20_P08_1)
#define PWM_RF (TCPWM_CH58_P17_3)
#define PWM_RB (TCPWM_CH30_P10_2)

// =================== 飞行参数配置 ===================
#define TARGET_HEIGHT_CM 120.0f  // 目标高度
#define LAND_HEIGHT 10.0f       // 着陆高度
#define HOVER_THROTTLE 4350    // 基础悬停油门 
#define MAX_PWM 6000
#define MIN_PWM 0
#define MAX_TILT_ANGLE 6.0f  // 最大倾角限制 (度) 
#define CTRL_DT_CTLOOP 0.001  // 飞控控制周期
#define CTRL_DT_CTANG 0.02   // 视觉控制周期

// 视觉控制增益
// 广角镜头单位角度对应的像素位移较小，过大的系数会导致过度补偿引起绕圈震荡
#define ANGLE_COMP_COEF 1.47f  //位姿对像素的补偿

#define IMG_CENTER_X (MT9V03X_W / 2.0f)
#define IMG_CENTER_Y (MT9V03X_H / 2.0f)

#define VALID_MIN_NUM 300
#define ACCEPT_ERROR 6.0f
#define MIN_ERROR 0.0f
//扫描旋转
#define SEARCH_YAW_RATE 15.0f  // 搜索角速度 (度/秒)
#define MAX_YAW_DEV     90.0f  // 最大扫描范围 (度)
#define TWO_MAX_YAW_DEV   120.0f  //计算出的最大偏航角
#define ROTATE_TIME    500.0f   //在扫描检测到目标后继续转的时间
#define MIN_SEARCH_TIME  120.0f   //用于状态1到3的降噪时间
#define MIN_SWITCH_TIME 1000.0f  //无人机累计没有识别到目标开始旋转的时间，单位：ms
#define WAIT_TIME   1000.0f  //在边缘等待的时间

#define TARGET_ACC_DISTANCE 100.0f
#define YAW_MIN_ERROR  5.0f

#define ROLL_OFFSET 240.0f
#define PITCH_OFFSET -38.5
// ================= 新增：摄像头物理偏心补偿 =================
    // 摄像头位于 IMU 后方 2cm，因此 X 轴补偿为 -2.0f
    #define CAM_OFFSET_X   10.0f  
    #define CAM_OFFSET_Y   -4.0f  // 假设左右居中无偏移
//视觉传输
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
extern float debug_earth_err_x;
extern float debug_earth_err_y;
extern float o_out_yaw;
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

/**
 * @brief 自动悬停控制逻辑封装
 * @note 内部处理视觉补偿、姿态设定及 PID 计算
 */
void Flight_Hover_Control_Task(void);
// [code/fly_ctrl.h] 在 "函数声明" 区域添加
void Fly_Param_Update(uint8_t ch, float val);
void Fly_Param_Update_Visual(uint8_t ch, float val);
void Fly_Param_Update_yaw(uint8_t ch, float val) ;
#endif