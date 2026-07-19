#ifndef _APP_H
#define _APP_H

#include "zf_common_headfile.h"

// 定义无人机的最高层级运行模式
typedef enum {
    DRONE_STATE_DEBUG = 0,      // 调试模式 (电机不转，可串口调参)
    DRONE_STATE_NORMAL_FLIGHT   // 正常飞行模式 (传感器融合，位置环/姿态环全开)
} Drone_State_e;

typedef enum {
    DRONE_MODE_USE_SWITCH = 0,  // 按拨码选择，运行期间允许从调试切换到飞行
    DRONE_MODE_FORCE_DEBUG,     // 忽略拨码，强制调试
    DRONE_MODE_FORCE_NORMAL     // 忽略拨码，强制正常飞行
} Drone_Mode_Select_e;

#define DRONE_MODE_SELECT DRONE_MODE_USE_SWITCH

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
