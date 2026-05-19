#ifndef _FLY_CTRL_H
#define _FLY_CTRL_H

#include "zf_common_headfile.h"


// =================== 飞行参数配置 ===================
#define TARGET_HEIGHT_CM 120.0f  // 目标高度
#define LAND_HEIGHT 10.0f       // 着陆高度
#define HOVER_THROTTLE 4350    // 基础悬停油门 
#define MAX_PWM 8000
#define MIN_PWM 0
#define MAX_TILT_ANGLE 6.0f  // 最大倾角限制 (度) 
#define CTRL_DT_CTLOOP 0.001f  // 飞控控制周期

#define MIN_ERROR 0.0f  //飞机跟踪小车的死区设置
//扫描旋转
#define SEARCH_YAW_RATE 15.0f  // 搜索角速度 (度/秒)
#define TWO_MAX_YAW_DEV   100.0f  //计算出的最大偏航角

#define YAW_OFFSET 6.0f    //为了防止信标被线挡住，让无人机偏过的角度
#define TARGET_ACC_DISTANCE 100.0f    //为了防止无人机与信标很近时yaw变化大，角度跟踪的最小距离
#define YAW_MIN_ERROR  5.0f    //角度跟踪设置的小死区

#define ROLL_OFFSET 0.0f//240.0f      //补偿重心偏移
#define PITCH_OFFSET 0.0f//-38.5f      //补偿重心偏移
// ================= 新增：摄像头物理偏心补偿 =================
    // 摄像头位于 IMU 后方 2cm，因此 X 轴补偿为 -2.0f
#define CAM_OFFSET_X   10.0f  
#define CAM_OFFSET_Y   -4.0f  // 假设左右居中无偏移

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
extern int LF , LB , RF,RB;
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

#endif