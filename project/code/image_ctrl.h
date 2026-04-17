#ifndef _IMAGE_CTRL_H
#define _IMAGE_CTRL_H
#include "zf_common_headfile.h"

#define ROTATE_RECOVER_TIME 1000
/**
 * @brief 自动悬停控制逻辑封装
 * @note 内部处理视觉补偿、姿态设定及 PID 计算
 */
void Flight_Hover_Control_Task(void);
#endif