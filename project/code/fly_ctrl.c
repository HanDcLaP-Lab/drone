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


extern volatile float share_data_from_1[];
extern volatile float share_data_from_0[];
static float start_up_scale = 0.0f;
//float car_pos_sol1_0 = 0.0f,car_pos_sol1_1 = 0.0f;
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
    PID_Init(&pid_height_pos, 0.5f, 0.15f, 0.0f, 30, 30, 40.0f);
    PID_Init(&pid_height_vel, 24.0f, 0.0f, 0.2f, 80, 1000, 40.0f);
    // 角度环a
    Nonline_PID_Init(&pid_roll, 4.5f, 0.8f, 0.0f, 0.05f, 20, 150, 40.0f);
    Nonline_PID_Init(&pid_pitch, 4.5f, 0.8f, 0.0f, 0.05f, 20, 150, 40.0f);
    Nonline_PID_Init(&pid_yaw, 1.0f, 0.4f, 0.0f, 0.03f, 6, 70, 40.0f);
    // 角速度环g
    PID_Init(&pid_g_roll, 2.73f, 1.52f, 0.11f, 100, 3500, 40.0f);
    PID_Init(&pid_g_pitch, 2.73f, 1.52f, 0.11f, 100, 3500, 40.0f);
    PID_Init(&pid_g_yaw, 2.73f, 1.52f, 0.11f, 100, 3500, 40.0f);
    // 视觉部分
    Nonline_PID_Init(&pid_image_x, 0.08f, 0.00f, 0.00f, 0.00f, 100, 15, 6.0f);
    Nonline_PID_Init(&pid_image_y, 0.08f, 0.00f, 0.00f, 0.00f, 100, 15, 6.0f);
}

void Flight_Unlock(void) {
    flight_target.is_armed = 1;
    start_up_scale = 0.0f;

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
    if(current_drone_state == DRONE_STATE_DEBUG) {
        pwm_set_duty(PWM_RF, 4000);
        pwm_set_duty(PWM_RB, 4000);
        pwm_set_duty(PWM_LF, 4000);
        pwm_set_duty(PWM_LB, 4000);
        return;
    }
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

// static void simple_image_process(float* car_row, float* car_col) {
//         *car_row -= imu_data.pitch * ANGLE_COMP_COEF;
//         *car_col -= imu_data.roll * ANGLE_COMP_COEF;
// 
//         float current_height = imu_data.z;
//         if (current_height < 40.0f) current_height = 40.0f; 
//         float height_gain = current_height / 100.0f;        
// 
//         *car_row = IMG_CENTER_Y + (*car_row - IMG_CENTER_Y) * height_gain;
//         *car_col = IMG_CENTER_X + (*car_col - IMG_CENTER_X) * height_gain;
// 
// }

void Flight_Hover_Control_Task(void) {
    // 使用局部变量
    // float car_row = share_data_from_1[0];  
    // float car_col = share_data_from_1[1];  
    float car_pos_x = share_data_from_1[3];
    float car_pos_y = share_data_from_1[4];
    // uint32_t car_area = (uint32_t)share_data_from_1[2]; 
    uint8_t locked_lights = (uint8_t)share_data_from_1[14];    

    static float search_dir = 1.0f;
    if (locked_lights == 1 || locked_lights == 3) {
        // 在局部变量上做补偿运算，不改变原始图像数据
        //simple_image_process(&car_row, &car_col);

        float error_row = -car_pos_x;
        float error_col = car_pos_y;

        // car_pos_sol1_0 = car_row;比较新老算法使用
        // car_pos_sol1_1 = car_col;

        // 在 code/fly_ctrl.c 中修改 Flight_Hover_Control_Task
        extern uint32_t pit0_cnt;
        static uint32_t last_ang_cnt = 0;
        static uint16_t real_dt_ang = 20; // 修改：默认 20ms，避免截断为 0
        if(last_ang_cnt != 0) real_dt_ang = pit0_cnt - last_ang_cnt;

        // 修改：将 last_ang_cnt 改为 real_dt_ang
        float target_pitch_val = Nonline_PID_Calculate(&pid_image_y, error_row, real_dt_ang / 1000.0f);
        float target_roll_val = Nonline_PID_Calculate(&pid_image_x, error_col, real_dt_ang / 1000.0f);

        last_ang_cnt = pit0_cnt;

        if (locked_lights == 1 && imu_data.z > 0.85 * TARGET_HEIGHT_CM) {
            flight_target.target_yaw += search_dir * SEARCH_YAW_RATE * last_ang_cnt / 1000.0f;
            
            if (flight_target.target_yaw > MAX_YAW_DEV) {
                flight_target.target_yaw = MAX_YAW_DEV; 
                search_dir = -1.0f; 
            } else if (flight_target.target_yaw < -MAX_YAW_DEV) {
                flight_target.target_yaw = -MAX_YAW_DEV; 
                search_dir = 1.0f;  
            }
        }
        Set_Target_Attitude(target_roll_val, target_pitch_val, flight_target.target_yaw);
    } else {
        comp_row = IMG_CENTER_Y;
        comp_col = IMG_CENTER_X;
        Nonline_PID_Reset(&pid_image_x);
        Nonline_PID_Reset(&pid_image_y);
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
            
        case 3: // 角速度环 KP
            pid_g_roll.kp = val;
            pid_g_pitch.kp = val;
            pid_g_yaw.kp = val * 0.5f;
            break;

        // === 第二组：角速度环 (PID) ===
        // 包含 kp, kd (通常速度环 ki 给 0 或很小，这里只调 kp, kd)
        case 4: // 角速度环 KI
            pid_g_roll.ki = val;
            pid_g_pitch.ki = val;
            pid_g_yaw.ki = val * 0.5f;
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
            pid_image_x.kp2 = val;
            pid_image_y.kp2 = val;
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
            }else if(val >=1.5 && val <=2.5){
                wireless_uart_send_string("emergency stop\r\n");
                Flight_Lock();
            }else if(val <= 0.5 && val >= -0.5){
                if (imu_data.is_calibrated) {
                    Flight_Unlock();
                } else {
                    flight_target.is_armed = 2; 
                }
                flight_target.cur_state = normal;
                start_up_scale = 0;
            }
            break;
        default:
            break;
    }
}