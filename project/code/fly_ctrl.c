#include "fly_ctrl.h"

// =================== 全局变量定义 ===================
Flight_Target_t flight_target = {0};
Motor_Output_t motor_out = {0};
float out = 0;
float comp_col = 0;
float comp_row = 0;
// 定义 PID 对象
PID_t pid_height_vel;
PID_t pid_height_pos;
Nonline_PID_t pid_roll;
Nonline_PID_t pid_pitch;
Nonline_PID_t pid_yaw;

Nonline_PID_t pid_image_x;
Nonline_PID_t pid_image_y;

PID_t pid_g_roll;
PID_t pid_g_pitch;
PID_t pid_g_yaw;

extern float m7_1_data[6];
static float start_up_scale = 0.0f;
// =================== 内部辅助函数 ===================
static float Constrain_Float(float val, float min, float max) {
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

// 角度误差处理 (处理 -180 到 180 跳变)
static float Get_Angle_Error(float target, float current) {
    float error = target - current;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

// =================== 核心控制逻辑 ===================

void Flight_Control_Init(void) {
    flight_target.cur_state = normal;
    flight_target.is_armed = 2;  // 2: 锁定, 0: 待机, 1: 解锁
    flight_target.height = 0;

    // ----------- 初始化 PID 参数 -----------
    // 高度环
    PID_Init(&pid_height_pos, 0.0f, 0.0f, 0.0f, 0, 0);
    PID_Init(&pid_height_vel, 0.0f, 0.0f, 0.0f, 0, 0);
    // 角度环
    Nonline_PID_Init(&pid_roll, 1.8f, 0.6f, 0.0f, 0.0f, 2.5, 15);
    Nonline_PID_Init(&pid_pitch, 1.8f, 0.6f, 0.0f, 0.0f, 2.5, 15);
    Nonline_PID_Init(&pid_yaw, 1.0f, 0.0f, 0.0f, 0.0f, 5, 15);
    // 角速度环
    PID_Init(&pid_g_roll, 15.0f, 0.0f, 0.3f, 300, 1200);
    PID_Init(&pid_g_pitch, 15.0f, 0.0f, 0.3f, 300, 1200);
    PID_Init(&pid_g_yaw, 6.0f, 0.0f, 0.12f, 120, 0);
    // 视觉部分
    Nonline_PID_Init(&pid_image_x, 0.03f, 0.0f, 0.00008f, 0.0005f, 1, 15);
    Nonline_PID_Init(&pid_image_y, 0.03f, 0.0f, 0.00008f, 0.0005f, 1, 15);
}

void Flight_Unlock(void) {
    flight_target.is_armed = 1;
    // 解锁瞬间重置积分，防止暴冲
    PID_Reset(&pid_height_vel);
    PID_Reset(&pid_height_pos);
    Nonline_PID_Reset(&pid_roll);
    Nonline_PID_Reset(&pid_pitch);
    Nonline_PID_Reset(&pid_yaw);
    PID_Reset(&pid_g_roll);
    PID_Reset(&pid_g_pitch);
    PID_Reset(&pid_g_yaw);

    // 锁定当前航向为目标航向，防止解锁即转圈
    flight_target.target_yaw = imu_data.yaw;
    flight_target.height = imu_data.z;
}

void Flight_Lock(void) {
    flight_target.is_armed = 0;
    motor_out.rf = 0;
    motor_out.rb = 0;
    motor_out.lb = 0;
    motor_out.lf = 0;
}

// 直接设定目标姿态
void Set_Target_Attitude(float roll, float pitch, float yaw) {
    flight_target.target_roll = Constrain_Float(roll, -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
    flight_target.target_pitch = Constrain_Float(pitch, -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
    flight_target.target_yaw = yaw;
}

void Flight_Control_Angle(void) {
    // 1. 计算误差 (绝对系)
    float roll_error = flight_target.target_roll - imu_data.roll;
    float pitch_error = flight_target.target_pitch - imu_data.pitch;
    float yaw_error = Get_Angle_Error(flight_target.target_yaw, imu_data.yaw);

    // 2. PID 计算 (输出即视为机体角速度目标，基于小角度假设)
    float target_rate_roll_body = Nonline_PID_Calculate(&pid_roll, roll_error, CTRL_DT_CTANG);
    float target_rate_pitch_body = Nonline_PID_Calculate(&pid_pitch, pitch_error, CTRL_DT_CTANG);
    float target_rate_yaw_body = Nonline_PID_Calculate(&pid_yaw, yaw_error, CTRL_DT_CTANG);
    flight_target.target_g_roll = target_rate_roll_body;
    flight_target.target_g_pitch = target_rate_pitch_body;
    flight_target.target_g_yaw = target_rate_yaw_body;
}
void Flight_Control_Loop(void) {
    // 1. 状态机处理
    Flight_Control_Angle();

    if (flight_target.cur_state == pre_landing && imu_data.z < LAND_HEIGHT + 2)
        flight_target.cur_state = landing;

    
    switch (flight_target.cur_state) {
        case normal:
            flight_target.target_height = TARGET_HEIGHT_CM;
            break;
        case pre_landing:
            flight_target.target_height = LAND_HEIGHT;
            break;
        case landing:
            if (start_up_scale > 0)
                start_up_scale -= 0.002;
            break;
        default:
            break;
    }

    if (flight_target.is_armed == 0) {
        Flight_Lock();
        return;
    }

    // ================= 2. 高度控制 =================
    // 平滑目标高度
    flight_target.height = flight_target.height * 0.999f + flight_target.target_height * 0.001f;

    float height_error = flight_target.height - imu_data.z;
    float target_climb_rate = PID_Calculate(&pid_height_pos, height_error, CTRL_DT_CTLOOP);

    float climb_rate_error = target_climb_rate - imu_data.vz;
    float throttle_adj = PID_Calculate(&pid_height_vel, climb_rate_error, CTRL_DT_CTLOOP);

    int16_t base_throttle = HOVER_THROTTLE + (int16_t)throttle_adj;
    // base_throttle = (int16_t)Constrain_Float(base_throttle, MIN_PWM, MAX_PWM);

    // ================= 3. 姿态控制 =================

    // Roll PID
    float roll_err = flight_target.target_g_roll - imu_data.groll;
    float out_roll = PID_Calculate(&pid_g_roll, roll_err, CTRL_DT_CTLOOP);
    out = out_roll;
    // Pitch PID
    float pitch_err = flight_target.target_g_pitch - imu_data.gpitch;
    float out_pitch = PID_Calculate(&pid_g_pitch, pitch_err, CTRL_DT_CTLOOP);

    // Yaw PID (使用角度环)
    float yaw_err = flight_target.target_g_yaw - imu_data.gyaw;
    float out_yaw = -PID_Calculate(&pid_g_yaw, yaw_err, CTRL_DT_CTLOOP);

    if (flight_target.is_armed == 1 && flight_target.cur_state != landing && flight_target.cur_state != pre_landing) {
        if (start_up_scale < 1.0f) {
            start_up_scale += 0.0005f;  // 约2秒加满 (1ms周期)
        }
    } 

    // 应用到电机输出
    motor_out.lf = (int16_t)(base_throttle * start_up_scale + (out_pitch + out_roll + out_yaw) * start_up_scale);

    // RF (右前, CCW): Base + Pitch - Roll - Yaw
    motor_out.rf = (int16_t)((base_throttle + out_pitch - out_roll - out_yaw) * start_up_scale);

    // LB (左后, CCW): Base - Pitch + Roll - Yaw
    motor_out.lb = (int16_t)((base_throttle - out_pitch + out_roll - out_yaw) * start_up_scale);

    // RB (右后, CW): Base - Pitch - Roll + Yaw
    motor_out.rb = (int16_t)((base_throttle - out_pitch - out_roll + out_yaw) * start_up_scale);

    // ================= 5. 输出限幅 =================
    int16_t* motors = (int16_t*)&motor_out;
    for (int i = 0; i < 4; i++) {
        if (motors[i] > MAX_PWM) motors[i] = MAX_PWM;
        if (motors[i] < MIN_PWM) motors[i] = MIN_PWM;
    }
}

// 辅助：电机PWM设置
void motor_pwm_set() {
    if (flight_target.is_armed == 1) {
        pwm_set_duty(PWM_RF, (motor_out.rf * 2 / 5) + 4000);  // 假设你的电调协议需要这样转换
        pwm_set_duty(PWM_RB, (motor_out.rb * 2 / 5) + 4000);
        pwm_set_duty(PWM_LF, (motor_out.lf * 2 / 5) + 4000);
        pwm_set_duty(PWM_LB, (motor_out.lb * 2 / 5) + 4000);
        //  pwm_set_duty(PWM_RF, (4500 * 2 / 5) + 4000);  // 假设你的电调协议需要这样转换
        //  pwm_set_duty(PWM_RB, (4500 * 2 / 5) + 4000);
        //  pwm_set_duty(PWM_LF, (4500 * 2 / 5) + 4000);
        //  pwm_set_duty(PWM_LB, (4500 * 2 / 5) + 4000);
    } else {
        motor_out.rf = 0;
        motor_out.rb = 0;
        motor_out.lf = 0;
        motor_out.lb = 0;
        pwm_set_duty(PWM_RF, 4000);
        pwm_set_duty(PWM_RB, 4000);
        pwm_set_duty(PWM_LF, 4000);
        pwm_set_duty(PWM_LB, 4000);
    }
}

void motor_pwm_init() {
    pwm_init(PWM_LF, 400, 4000);
    pwm_init(PWM_LB, 400, 4000);
    pwm_init(PWM_RF, 400, 4000);
    pwm_init(PWM_RB, 400, 4000);
}


void M7_1_data_send(float* M7_1_data) {
    M7_1_data[0] = (float)cam_down.centers[0][0];
    M7_1_data[1] = (float)cam_down.centers[0][1];
    M7_1_data[2] = (float)cam_down.dot_num[0];
    M7_1_data[3] = (float)cam_down.centers[1][0];
    M7_1_data[4] = (float)cam_down.centers[1][1];
    M7_1_data[5] = (float)cam_down.dot_num[1];

    if (cam_down.light_number == 0) {
        M7_1_data[2] = M7_1_data[5] = 0;
    }
}

void Flight_Hover_Control_Task(void) {
    SCB_CleanInvalidateDCache_by_Addr((void*)&m7_1_data, sizeof(float) * 6);
    cam_down.centers[0][0] = (uint32_t)m7_1_data[0];  // Row
    cam_down.centers[0][1] = (uint32_t)m7_1_data[1];  // Col
    cam_down.dot_num[0] = (uint32_t)m7_1_data[2];     // Area
    if (cam_down.dot_num[0] > MIN_LIGHT_SIZE && cam_down.centers[0][0] > 0 && cam_down.centers[0][1] > 0) {
        cam_down.light_number = 1;
    } else {
        cam_down.light_number = 0;
    }
    if (cam_down.light_number >= 1) {
        // 直接计算像素误差
        float car_row = (float)Kalman_Update(&K_w_ay, cam_down.centers[0][0]);
        float car_col = (float)Kalman_Update(&K_w_ax, cam_down.centers[0][1]);

        comp_row = car_row - (imu_data.pitch * ANGLE_COMP_COEF);

        comp_col = car_col - (imu_data.roll * ANGLE_COMP_COEF);
        float error_row = comp_row - IMG_CENTER_Y;
        float error_col = comp_col - IMG_CENTER_X;
        if(fabs(error_col) < ACCEPT_ERROR) error_col = 0;
        if(fabs(error_row) < ACCEPT_ERROR) error_row = 0;
        // if(cam_down.dot_num[0] > VALID_MIN_NUM) error_row = error_col = 0;

        // PID 控制
        float target_pitch_val = Nonline_PID_Calculate(&pid_image_y, error_row, CTRL_DT_CTANG);
        float target_roll_val = Nonline_PID_Calculate(&pid_image_x, error_col, CTRL_DT_CTANG);

        Set_Target_Attitude(target_roll_val, target_pitch_val, flight_target.target_yaw);
    } else {
        comp_row = IMG_CENTER_Y;
        comp_col = IMG_CENTER_X;
        Set_Target_Attitude(0, 0, flight_target.target_yaw);
    }                                                                                                                                                        

    // 3. 自动解锁与控制循环 
    if (imu_data.is_calibrated && flight_target.is_armed == 2) {
        Flight_Unlock();
    }
    
}

// 无线调参映射函数
// ch: 通道号 (1~8), val: 上位机发送的值
void Fly_Param_Update(uint8_t ch, float val) {
    switch (ch) {
        // === 第一组：角度环 (Nonline_PID) ===
        // 包含 kp, ki, kp2
        case 1: // 角度环 KP
            pid_roll.kp = val;
            pid_pitch.kp = val;
            pid_yaw.kp = val * 0.5f; // Yaw 参数为 Roll 的 0.5 倍
            break;
            
        case 2: // 角度环 KI
            pid_roll.ki = val;
            pid_pitch.ki = val;
            pid_yaw.ki = val * 0.5f;
            break;
            
        case 3: // 角度环 KP2 (非线性项)
            pid_roll.kp2 = val;
            pid_pitch.kp2 = val;
            pid_yaw.kp2 = val * 0.5f;
            break;

        // === 第二组：角速度环 (PID) ===
        // 包含 kp, kd (通常速度环 ki 给 0 或很小，这里只调 kp, kd)
        case 4: // 角速度环 KP
            pid_g_roll.kp = val;
            pid_g_pitch.kp = val;
            pid_g_yaw.kp = val * 0.5f;
            break;
            
        case 5: // 角速度环 KD
            pid_g_roll.kd = val;
            pid_g_pitch.kd = val;
            pid_g_yaw.kd = val * 0.5f;
            break;
        
        case 6:
            flight_target.target_roll = val;
            break;
        case 7:
            flight_target.target_pitch = val;
            break;
        case 8:
            if(val == 1){
                wireless_uart_send_string("land\r\n");
                flight_target.cur_state = pre_landing; 
            }else if(val == 2){
                wireless_uart_send_string("emergency stop\r\n");
                flight_target.cur_state = landing;
            }else if(val == 0){
                Flight_Unlock();
                flight_target.cur_state = normal;
                start_up_scale = 0;
            }
        default:
            break;
    }
}