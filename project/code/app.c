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

// 无线调参映射函数
// ch: 通道号 (1~8), val: 上位机发送的值
void Fly_Param_Update(uint8_t ch, float val) {
    switch (ch) {
        // === 第一组：角度环 (Nonline_PID) ===
        // 包含 kp, ki, kp2
        case 1: // 总体输出缩放系数 (限制在 0.0 ~ 1.0 之间)
            if (val > 1.0f) val = 1.0f;
            if (val < 0.0f) val = 0.0f;
            flight_target.output_scale = val;
            break;
            
        case 2: // 角度环 KI
            pid_roll.ki = val;
            pid_pitch.ki = val;
            //pid_yaw.ki = val * 0.5f;
            break;
            
        case 3: // 角速度环 KP
            // pid_image_x.kp = val;
            // pid_image_y.kp = val;
            //pid_g_yaw.kp = val * 0.5f;
            pid_image_x.kp = val;
            pid_image_y.kp = val;
            break;

        // === 第二组：角速度环 (PID) ===
        // 包含 kp, kd (通常速度环 ki 给 0 或很小，这里只调 kp, kd)
        case 4: // 角速度环 KI
            // pid_g_roll.ki = val;
            // pid_g_pitch.ki = val;
            //pid_g_yaw.ki = val * 0.5f;
            //search_yaw_rate = val;
            //pid_image_x.kp = val;
            pid_image_x.ki = val;
            pid_image_y.ki = val;
            break;
            
        case 5: // 角速度环 KD
            pid_g_roll.kp = val;
            pid_g_pitch.kp = val;

            //pid_g_yaw.kd = val * 0.5f;
            //pid_yaw.kp = val;
            //pid_image_x.kd = val;
            //pid_image_y.kd = val;
            break;
        
        case 6:
            pid_g_roll.ki = val;
            pid_g_pitch.ki = val;
            break;
        case 7:
            pid_g_roll.kd = val;
            pid_g_pitch.kd = val;
            break;
        case 8:
            if(val == 1){
                wireless_uart_send_string("land\r\n");
                flight_target.cur_state = pre_landing; 
                car_en = 0;
            }else if(val == 2){
                wireless_uart_send_string("emergency stop\r\n");
                car_en = 0;
                Flight_Lock();
            }else if(val == 0){
                if (imu_data.is_calibrated) {
                    Flight_Unlock();
                } else {
                    flight_target.is_armed = 2; // 进入等待校准状态
                }
                flight_target.cur_state = normal;
                flight_target.start_up_scale = 0;
            }
        default:
            break;
    }
}

void Fly_Param_Update_Visual(uint8_t ch, float val) {
    switch (ch) {
        case 1: // 视觉环 KP
            pid_image_x.kp = val;
            pid_image_y.kp = val;
            break;
        case 2: // 视觉环 KI
            pid_image_x.ki = val;
            pid_image_y.ki = val;
            break;
        case 3: // 视觉环 KD
            pid_image_x.kd = val;
            pid_image_y.kd = val;
            break;
        case 4: // 视觉环 KP2
            break;
        case 5: // 角速度环 KP
            pid_g_roll.kp = val;
            pid_g_pitch.kp = val;
            pid_g_yaw.kp = val * 0.5f;
            break;
        case 6: // 角速度环 KI
            pid_g_roll.ki = val;
            pid_g_pitch.ki = val;
            pid_g_yaw.ki = val * 0.5f;
            break;
        case 7: // 角速度环 KD
            pid_g_roll.kd = val;
            pid_g_pitch.kd = val;
            pid_g_yaw.kd = val * 0.5f;
            break;
        case 8:
            if(0.5 <= val && val < 1.5){
                wireless_uart_send_string("land\r\n");
                flight_target.cur_state = pre_landing; 
                car_en = 0;
            }else if(val >=1.5 && val <=2.5){
                wireless_uart_send_string("emergency stop\r\n");
                Flight_Lock();
                car_en = 0;
            }else if(val <= 0.5 && val >= -0.5){
                if (imu_data.is_calibrated) {
                    Flight_Unlock();
                } else {
                    flight_target.is_armed = 2; 
                }
                flight_target.cur_state = normal;
                flight_target.start_up_scale = 0;
            }
            break;
        default:
            break;
    }
}


void Fly_Param_Update_yaw(uint8_t ch, float val) {
    switch (ch) {
        case 1: // 视觉环 KP
            pid_yaw.kp = val;
            break;
        case 2: // 视觉环 KI
            pid_yaw.ki = val;
            break;
        case 3: // 视觉环 KD
            pid_yaw.kp2 = val;
            break;
        case 4: // 视觉环 KP2
            break;
        case 5: // 角速度环 KP
            pid_image_x.kp = val;
            pid_image_y.kp = val;
            break;
        case 6: // 角速度环 KI
            pid_image_x.ki = val;
            pid_image_y.ki = val;
            break;
        case 7: // 角速度环 KD
            pid_image_x.kd = val;
            pid_image_y.kd = val;
            break;
        case 8:
            if(0.5 <= val && val < 1.5){
                wireless_uart_send_string("land\r\n");
                flight_target.cur_state = pre_landing; 
                car_en = 0;
            }else if(val >=1.5 && val <=2.5){
                wireless_uart_send_string("emergency stop\r\n");
                Flight_Lock();
                car_en = 0;
            }else if(val <= 0.5 && val >= -0.5){
                if (imu_data.is_calibrated) {
                    Flight_Unlock();
                } else {
                    flight_target.is_armed = 2; 
                }
                flight_target.cur_state = normal;
                flight_target.start_up_scale = 0;
            }
            break;
        default:
            break;
    }
}