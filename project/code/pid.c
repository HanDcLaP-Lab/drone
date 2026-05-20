#include "zf_common_headfile.h"

// 初始化函数
void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_i, float out_max, float d_filter_hz) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_i = max_i;
    pid->out_max = out_max;
    pid->d_filter_hz = d_filter_hz; // [新增] 设置滤波频率
    
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_derivative = 0.0f; // [新增] 初始化滤波状态
}

void Nonline_PID_Init(Nonline_PID_t *pid, float kp, float ki, float kd, float kp2, float max_i, float out_max, float d_filter_hz) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kp2 = kp2;
    pid->max_i = max_i;
    pid->out_max = out_max;
    pid->d_filter_hz = d_filter_hz; // [新增] 设置滤波频率
    
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_derivative = 0.0f; // [新增] 初始化滤波状态
}
// 重置函数
void PID_Reset(PID_t *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_derivative = 0.0f; // [新增] 重置滤波状态
}

void Nonline_PID_Reset(Nonline_PID_t *pid){
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_derivative = 0.0f; // [新增] 重置滤波状态
}

// 核心计算函数
float PID_Calculate(PID_t *pid, float error, float dt) {
    if (dt < 1e-7f) return 0.0f;
    // 1. P项计算
    float p_out = pid->kp * error;

    // 2. I项计算
    pid->integral += error * dt;

    // 积分限幅 (Anti-windup)
    if (pid->integral > pid->max_i) {
        pid->integral = pid->max_i;
    } else if (pid->integral < -pid->max_i) {
        pid->integral = -pid->max_i;
    }
    
    float i_out = pid->ki * pid->integral;

    // 3. D项计算
    float d_out = 0.0f;
    if (pid->kd > 0.0f)
    {
        // 简单的误差微分: (当前误差 - 上次误差) / dt
        float derivative = (error - pid->prev_error) / dt;
        
        // D项低通滤波 (PT1 Filter)
        float rc = 1.0f / (2.0f * PID_PI * pid->d_filter_hz);
        float alpha = dt / (dt + rc);
        
        // 更新滤波后的微分值
        pid->prev_derivative = pid->prev_derivative + alpha * (derivative - pid->prev_derivative);
        
        // 使用滤波后的微分值计算 D 项输出
        d_out = pid->kd * pid->prev_derivative;
    }
    pid->prev_error = error; // 更新历史误差

    // 4. 总输出计算
    float output = p_out + i_out + d_out;

    // 5. 输出限幅
    if (output > pid->out_max) {
        output = pid->out_max;
    } else if (output < -pid->out_max) {
        output = -pid->out_max;
    }

    return output;
}

float Nonline_PID_Calculate(Nonline_PID_t *pid, float error, float dt) {
    if (dt < 1e-7f) return 0.0f;
    if (dt < 1e-7f) return 0.0f;
    // 1. P项计算
    float p_out = pid->kp * error;
    float p2_out = pid->kp2 * error * fabsf(error);
    float p2_out = pid->kp2 * error * fabsf(error);

    // 2. I项计算
    pid->integral += error * dt;

    // 积分限幅 (Anti-windup)
    if (pid->integral > pid->max_i) {
        pid->integral = pid->max_i;
    } else if (pid->integral < -pid->max_i) {
        pid->integral = -pid->max_i;
    }
    
    float i_out = pid->ki * pid->integral;

    // 3. D项计算
    float d_out = 0.0f;
    if (pid->kd > 0.0f)
    {
        // 简单的误差微分: (当前误差 - 上次误差) / dt
        float derivative = (error - pid->prev_error) / dt;
        
        // D项低通滤波 (PT1 Filter)
        float rc = 1.0f / (2.0f * PID_PI * pid->d_filter_hz);
        float alpha = dt / (dt + rc);
        
        // 更新滤波后的微分值
        pid->prev_derivative = pid->prev_derivative + alpha * (derivative - pid->prev_derivative);
        
        // 使用滤波后的微分值计算 D 项输出
        d_out = pid->kd * pid->prev_derivative;
    }
    pid->prev_error = error; // 更新历史误差

    // 4. 总输出计算
    float output = p_out + p2_out + i_out + d_out;

    // 5. 输出限幅
    if (output > pid->out_max) {
        output = pid->out_max;
    } else if (output < -pid->out_max) {
        output = -pid->out_max;
    }

    return output;
}