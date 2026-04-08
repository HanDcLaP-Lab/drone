// 初始化函数
#include "kalman_filter.h"
#include "zf_common_headfile.h"

KalmanFilter1 K_w_ax,K_w_ay,K_groll,K_gpitch,K_gyaw,K_ax,K_ay,K_az; // 卡尔曼定义
KalmanFilter1 K_car_x, K_car_y;
KalmanFilter1 K_target_x, K_target_y;
void Kalman_Init(KalmanFilter1* kf, float q, float r, float initial_value) {
    kf->q = q;
    kf->r = r;
    kf->x = initial_value;
    kf->p = 1;
    kf->k = 0;
}


// 卡尔曼滤波更新函数
float Kalman_Update(KalmanFilter1* kf, float measurement) {
    kf->p = kf->p + kf->q;                          // 预测
    kf->k = kf->p / (kf->p + kf->r);                // 更新卡尔曼增益
    kf->x = kf->x + kf->k * (measurement - kf->x);  // 更新估计值
    kf->p = (1 - kf->k) * kf->p;                    // 更新误差协方差
    return kf->x;
}

void main_kalman_init(void){
    // 初始化卡尔曼滤波参数
        Kalman_Init(&K_w_ax, 1e-3f, 0.01f, 0);
        Kalman_Init(&K_w_ay, 1e-3f, 0.01f, 0);
        Kalman_Init(&K_groll, 1e-3f, 1e-3f, 0);
        Kalman_Init(&K_gpitch, 1e-3f, 1e-3f, 0);
        Kalman_Init(&K_gyaw, 1e-3f, 1e-3f, 0);
        Kalman_Init(&K_ax, 0.001f, 0.05f, 0);
        Kalman_Init(&K_ay, 0.001f, 0.05f, 0);
        Kalman_Init(&K_az, 0.001f, 0.05f, 9.8f);
}