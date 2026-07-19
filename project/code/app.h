#ifndef _APP_H
#define _APP_H

#include "zf_common_headfile.h"

// 定义无人机的最高层级运行模式
typedef enum {
    DRONE_STATE_DEBUG = 0,      // 调试模式 (电机不转，可串口调参)
    DRONE_STATE_NORMAL_FLIGHT   // 正常飞行模式 (传感器融合，位置环/姿态环全开)
} Drone_State_e;

// 启动模式：DEBUG 电机不转；NORMAL_FLIGHT 在 IMU 校准完成后自动解锁
#define DRONE_START_MODE DRONE_STATE_NORMAL_FLIGHT

// 声明全局变量，供其他文件读取当前状态
extern Drone_State_e current_drone_state;

// 应用层对外接口
void app_init(void);
void app_state_machine_update(void);

void Fly_Param_Update(uint8_t ch, float val);
void Fly_Param_Update_Visual(uint8_t ch, float val);
void Fly_Param_Update_yaw(uint8_t ch, float val) ;
void Fly_Param_Update_height(uint8_t ch, float val);
void Fly_Param_Update_Debug(uint8_t ch, float val);
#endif
