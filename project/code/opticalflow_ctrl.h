#ifndef _OPTICALFLOW_CTRL_H
#define _OPTICALFLOW_CTRL_H

#include "zf_common_headfile.h"

// ================= 光流速度环参数 =================
#define OPT_FLOW_TIMEOUT_MS       200U   // 光流数据超时: 超过此时长未收到新帧则回平, 避免使用陈旧速度

// ================= 函数声明 =================
// 低高度光流速度环任务
void    Flight_OpticalFlow_Control_Task(uint8_t flow_frame_new);

// 模式切换生命周期钩子
void    OpticalFlow_Mode_Enter(void);
void    OpticalFlow_Mode_Exit(void);
uint8_t OpticalFlow_Mode_Is_Active(void);

#endif