#include "app.h"
#include "key_switch.h" 
#include "imu.h"
#include "fly_ctrl.h"
#include "kalman_filter.h"

void app_flight_start(void) {
    
}

Drone_State_e current_drone_state = DRONE_STATE_DEBUG;

void app_init(void) {
    // 【核心修改】：开机瞬间直接读取底层引脚状态（无需消抖）
    // 假设 SWITCH1 拨向 ON (0电平) 为正常飞行模式，拨向 OFF (1电平) 为视觉调试模式
    gpio_init(SWITCH1, GPI, GPIO_HIGH, GPI_PULL_UP);
    if (gpio_get_level(SWITCH1) == 0) {
        current_drone_state = DRONE_STATE_NORMAL_FLIGHT;
        printf("[APP] Boot Mode: NORMAL FLIGHT MODE \r\n");
        //app_flight_start();
    } else {
        current_drone_state = DRONE_STATE_DEBUG;
        printf("[APP] Boot Mode: VISUAL DEBUG ONLY \r\n");
    }
}

void app_state_machine_update(void) {
    // 允许从调试模式切换到正常飞行模式
    if (current_drone_state == DRONE_STATE_DEBUG) {
        if (gpio_get_level(SWITCH1) == 0) { // 检测到拨码开关拨下
            printf("[APP] Switching to NORMAL FLIGHT MODE...\r\n");
            system_delay_ms(2000);
            current_drone_state = DRONE_STATE_NORMAL_FLIGHT;
            Flight_Unlock(); // 解锁飞行
        }
    }
}