#include "filters.h"

// ================= 卡尔曼滤波器 =================
KalmanFilter1 K_w_ax, K_w_ay, K_groll, K_gpitch, K_gyaw, K_ax, K_ay, K_az;
KalmanFilter1 K_car_x, K_car_y;
KalmanFilter1 K_target_x, K_target_y;

void Kalman_Init(KalmanFilter1* kf, float q, float r, float initial_value) {
    kf->q = q;
    kf->r = r;
    kf->x = initial_value;
    kf->p = 1;
    kf->k = 0;
}

float Kalman_Update(KalmanFilter1* kf, float measurement) {
    kf->p = kf->p + kf->q;                          // 预测
    kf->k = kf->p / (kf->p + kf->r);                // 更新卡尔曼增益
    kf->x = kf->x + kf->k * (measurement - kf->x);  // 更新估计值
    kf->p = (1 - kf->k) * kf->p;                    // 更新误差协方差
    return kf->x;
}

void main_kalman_init(void) {
    Kalman_Init(&K_w_ax, 1e-3f, 0.01f, 0);
    Kalman_Init(&K_w_ay, 1e-3f, 0.01f, 0);
    Kalman_Init(&K_groll, 1e-3f, 1e-3f, 0);
    Kalman_Init(&K_gpitch, 1e-3f, 1e-3f, 0);
    Kalman_Init(&K_gyaw, 1e-3f, 1e-3f, 0);
    Kalman_Init(&K_ax, 0.001f, 0.05f, 0);
    Kalman_Init(&K_ay, 0.001f, 0.05f, 0);
    Kalman_Init(&K_az, 0.001f, 0.05f, 9.8f);
}

// ================= 陷波滤波器 (通用 biquad) =================
void Notch_Init(NotchFilter_t *nf) {
    nf->b0 = 1.0f; nf->b1 = 0.0f; nf->b2 = 0.0f;
    nf->a1 = 0.0f; nf->a2 = 0.0f;
    nf->x1 = 0.0f; nf->x2 = 0.0f;
    nf->y1 = 0.0f; nf->y2 = 0.0f;
    nf->freq = 0.0f;
}

float Notch_Update(NotchFilter_t *nf, float input) {
    float output = nf->b0 * input + nf->b1 * nf->x1 + nf->b2 * nf->x2
                 - nf->a1 * nf->y1 - nf->a2 * nf->y2;
    nf->x2 = nf->x1;
    nf->x1 = input;
    nf->y2 = nf->y1;
    nf->y1 = output;
    return output;
}

void Notch_SetFreq(NotchFilter_t *nf, float freq, const NotchConfig_t *cfg) {
    if (freq == nf->freq) return;

    // 频率突变 >20% → 清零延迟线, 防止旧频率残留造成瞬态畸变
    if (nf->freq > cfg->min_freq && (freq > nf->freq * 1.2f || freq < nf->freq * 0.8f)) {
        nf->x1 = 0.0f; nf->x2 = 0.0f;
        nf->y1 = 0.0f; nf->y2 = 0.0f;
    }
    nf->freq = freq;

    // 频率越界 → 旁通 (低于下限或超出上限)
    if (freq < cfg->min_freq || freq > cfg->max_freq) {
        nf->b0 = 1.0f; nf->b1 = 0.0f; nf->b2 = 0.0f;
        nf->a1 = 0.0f; nf->a2 = 0.0f;
        return;
    }

    float w = 2.0f * PI * freq / cfg->fs;
    float cos_w = cosf(w);
    float alpha = sinf(w) / (2.0f * cfg->q);

    float a0_inv = 1.0f / (1.0f + alpha);
    nf->b0 = a0_inv;
    nf->b1 = -2.0f * cos_w * a0_inv;
    nf->b2 = a0_inv;
    nf->a1 = -2.0f * cos_w * a0_inv;
    nf->a2 = (1.0f - alpha) * a0_inv;
}
