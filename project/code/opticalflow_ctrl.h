#ifndef _OPTICALFLOW_CTRL_H
#define _OPTICALFLOW_CTRL_H

#include "zf_common_headfile.h"

// ================= 光流速度环参数 =================
#define OPT_FLOW_TIMEOUT_MS       200U   // 光流数据超时: 超过此时长未收到新帧则回平, 避免使用陈旧速度
#define MODE_SWITCH_SMOOTH_MS     150U   // 视觉/光流模式切换时目标倾角平滑时间 (ms)

// ================= 函数声明 =================
// 低高度光流速度环任务 (主循环在光流新帧时驱动)
void Flight_OpticalFlow_Control_Task(uint8_t flow_frame_new);

// 供 image_ctrl 调用的模式切换接口
uint8_t OpticalFlow_Mode_Should_Be_Active(void);
void OpticalFlow_Mode_Enter(void);
void OpticalFlow_Mode_Exit(void);
void OpticalFlow_Set_Target_Attitude_Smoothed(float roll, float pitch, float yaw, float dt_sec);

#endif