#include "app.h"
#include "key_switch.h" 
#include "imu.h"
#include "fly_ctrl.h"
#include "kalman_filter.h"

void app_flight_start(void) {
    // 1. 初始化卡尔曼滤波参数
    Kalman_Init(&K_w_ax, 1e-3f, 0.01f, 0);
    Kalman_Init(&K_w_ay, 1e-3f, 0.01f, 0);
    Kalman_Init(&K_groll, 1e-3f, 0.01f, 0);
    Kalman_Init(&K_gpitch, 1e-3f, 0.01f, 0);
    Kalman_Init(&K_gyaw, 1e-3f, 0.01f, 0);
    Kalman_Init(&K_ax, 0.001f, 0.1f, 0);
    Kalman_Init(&K_ay, 0.001f, 0.1f, 0);
    Kalman_Init(&K_az, 0.001f, 0.1f, 9.8f);

    // 2. 初始化底层传感器与执行器
    imu_init();
    tof_init();
    motor_pwm_init(); 
    Flight_Control_Init();

    // 3. 启动周期中断 (确保数据结构初始化完毕后再开启中断)
    // PIT_CH1(20ms), PIT_CH2(400ms)
    pit_ms_init(PIT_CH1, 20);
    pit_ms_init(PIT_CH2, 400);
    system_delay_ms(1000);     // 等待传感器数据稳定
    pit_ms_init(PIT_CH0, 1);   // 开启核心飞控中断 (1ms)
}

Drone_State_e current_drone_state = DRONE_STATE_DEBUG;

void app_init(void) {
    // 【核心修改】：开机瞬间直接读取底层引脚状态（无需消抖）
    // 假设 SWITCH1 拨向 ON (0电平) 为正常飞行模式，拨向 OFF (1电平) 为视觉调试模式
    gpio_init(SWITCH1, GPI, GPIO_HIGH, GPI_PULL_UP);
    if (gpio_get_level(SWITCH1) == 0) {
        current_drone_state = DRONE_STATE_NORMAL_FLIGHT;
        printf("[APP] Boot Mode: NORMAL FLIGHT MODE \r\n");
        app_flight_start();
    } else {
        current_drone_state = DRONE_STATE_DEBUG;
        printf("[APP] Boot Mode: VISUAL DEBUG ONLY \r\n");
    }
}

void app_state_machine_update(void) {
    // 允许从调试模式切换到正常飞行模式 (反向切换不建议在运行中进行)
    if (current_drone_state == DRONE_STATE_DEBUG) {
        if (gpio_get_level(SWITCH1) == 0) { // 检测到拨码开关拨下
            printf("[APP] Switching to NORMAL FLIGHT MODE...\r\n");
            current_drone_state = DRONE_STATE_NORMAL_FLIGHT;
            app_flight_start(); // 执行飞行系统初始化并启动中断
        }
    }
}