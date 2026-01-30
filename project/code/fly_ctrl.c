#include "fly_ctrl.h"

// =================== 全局变量定义 ===================
Flight_Target_t flight_target = {0};
Motor_Output_t motor_out = {0};
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
    flight_target.is_armed = 2;  // 0: 锁定, 1: 解锁, 2: 等待校准后解锁
    flight_target.height = 0;
    start_up_scale = 0.0f;

    // ----------- 初始化 PID 参数 -----------
    // 高度环
    PID_Init(&pid_height_pos, 0.5f, 0.15f, 0.0f, 3, 0);
    PID_Init(&pid_height_vel, 24.0f, 0.0f, 0.2f, 80, 0);
    // 角度环a
    Nonline_PID_Init(&pid_roll, 4.27f, 0.26f, 0.0f, 0.05f, 5, 35);
    Nonline_PID_Init(&pid_pitch, 4.27f, 0.26f, 0.0f, 0.05f, 5, 35);
    Nonline_PID_Init(&pid_yaw, 2.1f, 0.26f, 0.0f, 0.025f, 5, 35);
    // 角速度环g
    PID_Init(&pid_g_roll, 26.0f, 0.0f, 0.38f, 300, 2500);
    PID_Init(&pid_g_pitch, 26.0f, 0.0f, 0.38f, 300, 2500);
    PID_Init(&pid_g_yaw, 13.0f, 0.0f, 0.19f, 120, 600);
    // 视觉部分
    Nonline_PID_Init(&pid_image_x, 0.03f, 0.0f, 0.0005f, 0.0005f, 1, 15);
    Nonline_PID_Init(&pid_image_y, 0.03f, 0.0f, 0.0005f, 0.0005f, 1, 15);
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
    Nonline_PID_Reset(&pid_image_x); // [新增] 重置视觉PID，防止积分累积
    Nonline_PID_Reset(&pid_image_y);

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

// =================== 内部功能模块 (Static) ===================

/**
 * @brief 飞行状态机更新
 * 处理自动解锁、飞行模式切换、目标高度设定
 */
static void Flight_State_Update(void) {
    // 1. 自动解锁逻辑 (等待IMU校准完成后自动解锁)
    if (flight_target.is_armed == 2) {
        if (imu_data.is_calibrated) {
            Flight_Unlock();
        }
        // 若未校准，保持 is_armed=2，电机输出为0
    }

    // 2. 降落检测逻辑
    if (flight_target.cur_state == pre_landing && imu_data.z < LAND_HEIGHT + 2) {
        flight_target.cur_state = landing;
    }

    // 3. 根据状态设定目标高度及特殊行为
    switch (flight_target.cur_state) {
        case normal:
            flight_target.target_height = TARGET_HEIGHT_CM;
            if (flight_target.is_armed == 1) {
                if (start_up_scale < 1.0f) {
                    start_up_scale += 0.0005f;  // 约2秒加满 (1ms周期)
                }
            }
            break;
        case pre_landing:
            flight_target.target_height = LAND_HEIGHT;
            break;
        case landing:
            // 降落阶段逐渐减小油门比例
            if (start_up_scale > 0) {
                start_up_scale -= 0.002f;
            }
            break;
        default:
            break;
    }
}

void Flight_Control_Angle(void) {
    // 1. 计算误差 (绝对系)
    float roll_error = flight_target.target_roll - imu_data.roll;
    float pitch_error = flight_target.target_pitch - imu_data.pitch;
    float yaw_error = Get_Angle_Error(flight_target.target_yaw, imu_data.yaw);

    // 2. PID 计算 (输出即视为机体角速度目标，基于小角度假设)
    flight_target.target_g_roll = Nonline_PID_Calculate(&pid_roll, roll_error, CTRL_DT_CTLOOP);
    flight_target.target_g_pitch = Nonline_PID_Calculate(&pid_pitch, pitch_error, CTRL_DT_CTLOOP);
    flight_target.target_g_yaw = Nonline_PID_Calculate(&pid_yaw, yaw_error, CTRL_DT_CTLOOP);
}

/**
 * @brief 高度环控制 (串级PID: 位置 -> 速度 -> 油门)
 * @return 基础油门值 (base_throttle)
 */
static int16_t Flight_Control_Height(void) {
    // 平滑目标高度
    flight_target.height = flight_target.height * 0.999f + flight_target.target_height * 0.001f;

    // 位置环
    float height_error = flight_target.height - imu_data.z;
    float target_climb_rate = PID_Calculate(&pid_height_pos, height_error, CTRL_DT_CTLOOP);

    // 速度环
    float climb_rate_error = target_climb_rate - imu_data.vz;
    float throttle_adj = PID_Calculate(&pid_height_vel, climb_rate_error, CTRL_DT_CTLOOP);

    return HOVER_THROTTLE + (int16_t)throttle_adj;
}

/**
 * @brief 角速度环控制 (PID)
 * @param out_roll/pitch/yaw 输出的控制量指针
 */
static void Flight_Control_Rate(float *out_roll, float *out_pitch, float *out_yaw) {
    // Roll PID
    float roll_err = flight_target.target_g_roll - imu_data.groll;
    *out_roll = PID_Calculate(&pid_g_roll, roll_err, CTRL_DT_CTLOOP);
    
    // Pitch PID
    float pitch_err = flight_target.target_g_pitch - imu_data.gpitch;
    *out_pitch = PID_Calculate(&pid_g_pitch, pitch_err, CTRL_DT_CTLOOP);

    // Yaw PID
    float yaw_err = flight_target.target_g_yaw - imu_data.gyaw;
    *out_yaw = PID_Calculate(&pid_g_yaw, yaw_err, CTRL_DT_CTLOOP);
}

/**
 * @brief 电机混控与输出
 * @param base_throttle 基础油门
 * @param out_roll/pitch/yaw 三轴控制量
 */
static void Flight_Motor_Mix(int16_t base_throttle, float out_roll, float out_pitch, float out_yaw) {

    // 混控算法 (X型四旋翼)
    // LF (左前, CW): Base + Pitch + Roll - Yaw
    motor_out.lf = (int16_t)((base_throttle + out_pitch + out_roll + out_yaw) * start_up_scale);

    // RF (右前, CCW): Base + Pitch - Roll + Yaw
    motor_out.rf = (int16_t)((base_throttle + out_pitch - out_roll - out_yaw) * start_up_scale);

    // LB (左后, CCW): Base - Pitch + Roll + Yaw
    motor_out.lb = (int16_t)((base_throttle - out_pitch + out_roll - out_yaw) * start_up_scale);

    // RB (右后, CW): Base - Pitch - Roll - Yaw
    motor_out.rb = (int16_t)((base_throttle - out_pitch - out_roll + out_yaw) * start_up_scale);

    // 输出限幅
    int16_t* motors = (int16_t*)&motor_out.rf;
    for (int i = 0; i < 4; i++) {
        if (motors[i] > MAX_PWM) motors[i] = MAX_PWM;
        if (motors[i] < MIN_PWM) motors[i] = MIN_PWM;
    }
}

// =================== 主控制循环 ===================
void Flight_Control_Loop(void) {
    // 1. 状态机更新 (包含自动解锁、模式切换、目标高度设定)
    Flight_State_Update();

    // 2. 角度环控制 (计算期望角速度)
    Flight_Control_Angle();

    // 3. 角速度环控制 (计算姿态修正量)
    Flight_Control_Rate(&motor_out.roll, &motor_out.pitch, &motor_out.yaw);

    // 4. 锁定检查
    if (flight_target.is_armed == 0) {
        Flight_Lock();
        return;
    }
    // 注意: is_armed == 2 (等待校准) 时也会继续执行，但 start_up_scale 为 0，电机不转，安全。

    // 5. 高度环控制 (计算基础油门)
    int16_t base_throttle = Flight_Control_Height();

    // 6. 电机混控与输出
    Flight_Motor_Mix(base_throttle, motor_out.roll, motor_out.pitch, motor_out.yaw);
}

// 辅助：电机PWM设置
void motor_pwm_set() {
    if (flight_target.is_armed == 1) {
        pwm_set_duty(PWM_RF, (motor_out.rf * 2 / 5) + 4000);
        pwm_set_duty(PWM_RB, (motor_out.rb * 2 / 5) + 4000);
        pwm_set_duty(PWM_LF, (motor_out.lf * 2 / 5) + 4000);
        pwm_set_duty(PWM_LB, (motor_out.lb * 2 / 5) + 4000);
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


void M7_1_data_send(float* M7_1_data, float* uart_data) {
    M7_1_data[0] = (float)cam_down.centers[0][0];
    M7_1_data[1] = (float)cam_down.centers[0][1];
    M7_1_data[2] = (float)cam_down.dot_num[0];
    uart_data[0] = (float)cam_down.centers[1][0];
    uart_data[1] = (float)cam_down.centers[1][1];
    uart_data[2] = (float)cam_down.dot_num[1];

    if (cam_down.light_number == 0) {
        M7_1_data[2] = uart_data[2] = 0;
    }
}

void Flight_Hover_Control_Task(void) {
    SCB_CleanInvalidateDCache_by_Addr((void*)&m7_1_data, sizeof(float) * DATA_LENGTH);
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
                Flight_Lock();
            }else if(val == 0){
                if (imu_data.is_calibrated) {
                    Flight_Unlock();
                } else {
                    flight_target.is_armed = 2; // 进入等待校准状态
                }
                flight_target.cur_state = normal;
                start_up_scale = 0;
            }
        default:
            break;
    }
}