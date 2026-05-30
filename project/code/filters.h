#ifndef _FILTERS_H
#define _FILTERS_H

#include "zf_common_headfile.h"

#ifndef PI
#define PI 3.1415926535f
#endif

// ================= 卡尔曼滤波器 =================
typedef struct {
    float x;  // 状态变量（估计的速度/脉冲数）
    float p;  // 估计协方差
    float q;  // 过程噪声协方差（系统模型的不确定性）
    float r;  // 测量噪声协方差（传感器噪声）
    float k;  // 卡尔曼增益
} KalmanFilter1;

void Kalman_Init(KalmanFilter1* kf, float q, float r, float initial_value);
float Kalman_Update(KalmanFilter1* kf, float measurement);
void main_kalman_init(void);

extern KalmanFilter1 K_w_ax, K_w_ay, K_groll, K_gpitch, K_gyaw, K_ax, K_ay, K_az;
extern KalmanFilter1 K_car_x, K_car_y;
extern KalmanFilter1 K_target_x, K_target_y;

// ================= 陷波滤波器 (通用 biquad) =================
typedef struct {
    float b0, b1, b2;               // 前馈系数 (归一化)
    float a1, a2;                   // 反馈系数 (归一化)
    float x1, x2;                   // 输入延迟线
    float y1, y2;                   // 输出延迟线
    float freq;                     // 当前中心频率 (Hz)
} NotchFilter_t;

typedef struct {
    float fs;                       // 采样频率 (Hz)
    float q;                        // 品质因数
    float min_freq;                 // 最低频率 (Hz), 低于此旁通
    float max_freq;                 // 最高频率 (Hz), 高于此旁通 (防 Nyquist)
} NotchConfig_t;

void Notch_Init(NotchFilter_t *nf);
float Notch_Update(NotchFilter_t *nf, float input);
void Notch_SetFreq(NotchFilter_t *nf, float freq, const NotchConfig_t *cfg);

#endif
