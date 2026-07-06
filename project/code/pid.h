#ifndef _PID_H
#define _PID_H

#include "zf_common_headfile.h"

// =================== PID 配置 ===================
#define PID_PI 3.1415926535f

// =================== PID 结构体定义 ===================
typedef struct {
    // --- 参数 (Parameters) ---
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    
    float max_i;        // 积分限幅 (防止积分饱和/Windup)
    float out_max;      // 总输出限幅
    float d_filter_hz;  // [新增] D项低通滤波截止频率 (Hz)
    
    // --- 运行时状态 (Runtime State) ---
    float integral;     // 积分累加值
    float prev_error;   // 上一次误差 (用于计算微分)
    float prev_derivative; // [新增] 上一次的微分值 (用于低通滤波)
} PID_t;

typedef struct {
    // --- 参数 (Parameters) ---
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float kp2;          //平方项
    
    float max_i;        // 积分限幅 (防止积分饱和/Windup)
    float out_max;      // 总输出限幅
    float d_filter_hz;  // [新增] D项低通滤波截止频率 (Hz)
    
    // --- 运行时状态 (Runtime State) ---
    float integral;     // 积分累加值
    float prev_error;   // 上一次误差 (用于计算微分)
    float prev_derivative; // [新增] 上一次的微分值 (用于低通滤波)
} Nonline_PID_t;

extern PID_t pid_height_vel;
extern PID_t pid_height_pos;
extern Nonline_PID_t pid_roll;
extern Nonline_PID_t pid_pitch;
extern Nonline_PID_t pid_yaw;

extern Nonline_PID_t pid_image_x;
extern Nonline_PID_t pid_image_y;

extern PID_t pid_g_roll;
extern PID_t pid_g_pitch;
extern PID_t pid_g_yaw;
// =================== 函数声明 ===================

/**
 * 初始化 PID 参数
 * @param pid: 指向 PID 结构体的指针
 * @param kp, ki, kd: PID 参数
 * @param max_i: 积分项的最大值 (绝对值)
 * @param out_max: 总输出的最大值 (绝对值)
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_i, float out_max, float d_filter_hz);
void Nonline_PID_Init(Nonline_PID_t *pid, float kp, float ki, float kd, float kp2, float max_i, float out_max, float d_filter_hz);

/**
 * 重置 PID 状态 (清除积分和历史误差)
 * 通常在解锁瞬间或切换模式时调用
 */
void PID_Reset(PID_t *pid);
void Nonline_PID_Reset(Nonline_PID_t *pid);

/**
 * 计算 PID 输出
 * @param pid: 指向 PID 结构体的指针
 * @param error: 当前误差 (目标值 - 测量值)
 * @param dt: 控制周期 (单位: 秒)，例如 1.25ms 传 0.00125f
 * @return: PID 计算出的控制量
 */
float PID_Calculate(PID_t *pid, float error, float dt);
float Nonline_PID_Calculate(Nonline_PID_t *pid, float error, float dt);

#endif
