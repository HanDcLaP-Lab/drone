#ifndef _IMAGE_CTRL_H
#define _IMAGE_CTRL_H
#include "zf_common_headfile.h"

// =================== 视觉前馈宏定义 ===================
#define CAR_FF_ENABLE           0       // 开启/关闭视觉前馈 (1:开启, 0:关闭)
#define CAR_FF_DIST_THRESHOLD   50.0f    // 预测触发距离阈值 (cm)
#define CAR_FF_SPEED            40.0f    // 小车运动速度 (cm/s, 即0.4m/s)
#define CAR_FF_PREDICT_TIME     0.25f    // 预测前瞻时间 (s) 结合机械延迟建议0.25s起步
#define CAR_FF_MAX_CHANGE       2.0f     // 每次循环前馈增量的最大变化限制 (cm)

#define ROTATE_RECOVER_TIME 1000
/**
 * @brief 自动悬停控制逻辑封装
 * @note 内部处理视觉补偿、姿态设定及 PID 计算
 */
void Flight_Hover_Control_Task(void);
#endif