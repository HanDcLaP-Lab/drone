#ifndef _APP_H
#define _APP_H

#include "zf_common_headfile.h"

// 定义无人机的最高层级运行模式
typedef enum {
    DRONE_STATE_DEBUG = 0,      // 调试模式 (上电默认，电机不转或受限，可串口调参)
    DRONE_STATE_NORMAL_FLIGHT   // 正常飞行模式 (传感器融合，位置环/姿态环全开)
} Drone_State_e;

// 声明全局变量，供其他文件读取当前状态
extern Drone_State_e current_drone_state;

// 应用层对外接口
void app_init(void);
void app_state_machine_update(void);

#endif