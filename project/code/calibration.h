#ifndef _CALIBRATION_H
#define _CALIBRATION_H

#include "zf_common_headfile.h"

// ================= 校准功能宏开关 =================
// 1: 启用起飞后自动悬停校准; 0: 完全关闭, 所有校准值回退到默认
#ifndef CALIBRATION_ENABLE
#define CALIBRATION_ENABLE        1
#endif

// 蜂鸣器引脚: 进入校准和完成校准时各鸣叫 100ms
#define BUZZER_PIN                (P19_4)

// 校准流程时间参数 (ms)
#define CALIB_STABLE_WAIT_MS      3000U   // 到达高度后非阻塞等待稳定
#define CALIB_SAMPLE_MS           1000U   // 校准采样时长
#define CALIB_START_HEIGHT_OFFSET_CM 5.0f // 到达 TARGET_HEIGHT_CM - 5 后开始流程

// ================= 函数声明 =================
void Calibration_Init(void);
void Calibration_Update(void);

float   Calibration_Get_Roll_Offset(void);
float   Calibration_Get_Pitch_Offset(void);
float   Calibration_Get_Corrected_Roll(void);
float   Calibration_Get_Corrected_Pitch(void);
int16_t Calibration_Get_Hover_PWM(uint8_t motor_index);
uint8_t Calibration_Is_Complete(void);

#endif