#include "app.h"
#include "key_switch.h" 

Drone_State_e current_drone_state = DRONE_STATE_DEBUG;

void app_init(void) {
    // 【核心修改】：开机瞬间直接读取底层引脚状态（无需消抖）
    // 假设 SWITCH1 拨向 ON (0电平) 为正常飞行模式，拨向 OFF (1电平) 为视觉调试模式
    gpio_init(SWITCH1, GPI, GPIO_HIGH, GPI_PULL_UP);
    if (gpio_get_level(SWITCH1) == 0) {
        current_drone_state = DRONE_STATE_NORMAL_FLIGHT;
        printf("[APP] Boot Mode: NORMAL FLIGHT MODE \r\n");
    } else {
        current_drone_state = DRONE_STATE_DEBUG;
        printf("[APP] Boot Mode: VISUAL DEBUG ONLY \r\n");
    }
}

void app_state_machine_update(void) {
    // 因为模式在开机前由拨码开关物理锁死，这里不再需要长按切模式的逻辑
    // 保留这个函数，供你以后添加其他按键功能（如按键切换屏幕显示页面等）
}