#include "fly_ctrl.h"

// =================== 全局变量定义 ===================
Flight_Target_t flight_target = {0};
Motor_Output_t motor_out = {0};
float comp_col = 0;
float comp_row = 0;
float debug_earth_err_x = 0;
float debug_earth_err_y = 0;
float search_yaw_rate = SEARCH_YAW_RATE;

// 定义 PID 对象
PID_t pid_height_vel;
PID_t pid_height_pos;
Nonline_PID_t pid_roll;
Nonline_PID_t pid_pitch;
Nonline_PID_t pid_yaw;

Nonline_PID_t pid_image_x;
Nonline_PID_t pid_image_y;
Nonline_PID_t pid_image_yaw;
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
    Nonline_PID_Init(&pid_yaw, 1.5f, 0.33f, 0.0f, 0.0228f, 6, 35, 40.0f);

    Nonline_PID_Init(&pid_image_yaw, 1.0f, 0.00f, 0.0f, 0.0f, 0, 60.0f, 4.0f);
    // 角速度环g
    PID_Init(&pid_g_roll, 2.73f, 1.52f, 0.11f, 100, 3500, 40.0f);
    PID_Init(&pid_g_pitch, 2.73f, 1.52f, 0.11f, 100, 3500, 40.0f);
    PID_Init(&pid_g_yaw, 1.36f, 0.42f, 0.011f, 100, 3500, 40.0f);
    // 视觉部分
    Nonline_PID_Init(&pid_image_x, 0.059f, 0.006f, 0.133f, 0.00f, 1000, 15, 4.0f);
    Nonline_PID_Init(&pid_image_y, 0.059f, 0.006f, 0.133f, 0.00f, 1000, 15, 4.0f);
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
    Nonline_PID_Reset(&pid_image_yaw);
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
    // 1. 获取目标中心坐标与锁定状态
    float car_pos_x = share_data_from_1[3];
    float car_pos_y = share_data_from_1[4];
    uint8_t locked_lights = (uint8_t)share_data_from_1[14];    
    if (locked_lights == 1 || locked_lights == 3) {
        car_pos_x = car_pos_x - CAM_OFFSET_X;
        car_pos_y = car_pos_y - CAM_OFFSET_Y;
    } 

    // 2. 计算真实时间差 dt (防除零)
    extern uint32_t pit0_cnt;
    static uint32_t last_ang_cnt = 0;
    static uint16_t real_dt_ang = 20; 
    if(last_ang_cnt != 0) real_dt_ang = pit0_cnt - last_ang_cnt;
    last_ang_cnt = pit0_cnt;

    // 3. 静态防抖与状态存储变量
    static uint8_t last_locked_lights = 0; 
    static uint32_t search_time_cnt = 0;   
    static float search_dir = 1.0f;        
    static float base_search_yaw = 0.0f;   
    
    // 【新增】：边缘停留相关的状态变量
    static uint8_t is_pausing = 0;         // 是否正在边缘停留
    static uint32_t edge_pause_cnt = 0;    // 边缘停留计时器 (ms)

    // ================== 有目标视野逻辑 ==================
    if (locked_lights == 1 || locked_lights == 3) {
        
        // ================= 位置环解耦 =================
        float yaw_rad = imu_data.yaw * 3.14159265f / 180.0f;
        float cos_yaw = cosf(yaw_rad);
        float sin_yaw = sinf(yaw_rad);

        float earth_err_x = car_pos_x * cos_yaw - car_pos_y * sin_yaw;
        float earth_err_y = car_pos_x * sin_yaw + car_pos_y * cos_yaw;
        if (fabsf(earth_err_x) < MIN_ERROR) earth_err_x = 0.0f;
        if (fabsf(earth_err_y) < MIN_ERROR) earth_err_y = 0.0f;

        debug_earth_err_x = earth_err_x;
        debug_earth_err_y = earth_err_y;

        float target_earth_accel_x = Nonline_PID_Calculate(&pid_image_x, earth_err_x, real_dt_ang / 1000.0f);
        float target_earth_accel_y = Nonline_PID_Calculate(&pid_image_y, earth_err_y, real_dt_ang / 1000.0f);

        float target_body_accel_x = target_earth_accel_x * cos_yaw + target_earth_accel_y * sin_yaw;
        float target_body_accel_y = -target_earth_accel_x * sin_yaw + target_earth_accel_y * cos_yaw;

        float target_pitch_val = -target_body_accel_x;
        float target_roll_val  = target_body_accel_y;
        // ========================================================

        // 逻辑A：当锁定了双目标（看到信标）
        // 逻辑A：当锁定了双目标（看到信标）
        if (locked_lights == 3) {
            
            // 【核心修改】：读取经过物理校正和姿态逆解算的地面坐标 (单位：厘米)
            // share_data_from_1[5] 是 target_ground_pos.x (前方距离)
            // share_data_from_1[6] 是 target_ground_pos.y (右方距离)
            // 减去摄像头的物理安装偏移量，得到信标相对于飞机实际重心的坐标
            float target_pos_x = share_data_from_1[5] - CAM_OFFSET_X; 
            float target_pos_y = share_data_from_1[6] - CAM_OFFSET_Y; 
            
            // 计算目标距离飞机重心的绝对物理直线距离 (厘米)
            float distance = sqrtf(target_pos_x * target_pos_x + target_pos_y * target_pos_y);
            
            // 【修改】：物理死区判断 
            // 这里的死区单位变成了“厘米”，例如设定为 20.0f (即允许信标在机身 20cm 半径内自由活动而不转机头)
            if (distance < TARGET_ACC_DISTANCE) {
                // 处于物理死区内：停止偏航追踪，重置 PID 积分
                Nonline_PID_Reset(&pid_image_yaw); 
            } else {
                // 【真实角度解算】：
                // 在经过 image_process.c 校正后的物理坐标系中，X 是正前方，Y 是正右方。
                // 刚好符合标准极坐标和无人机航向系的定义！
                // atan2f(Y, X) 算出的角度，向右为正，向左为负，完美匹配飞控的 Yaw 逻辑。
                float yaw_error = atan2f(target_pos_y, target_pos_x) * 180.0f / 3.14159265f;
                
                if(yaw_error > 90) yaw_error -= 180;
                if(yaw_error < -90) yaw_error += 180;
                if(fabs(yaw_error) < YAW_MIN_ERROR) yaw_error = 0;

                // 将解算出的真实角度误差送入 PID
                float yaw_pid_out = Nonline_PID_Calculate(&pid_image_yaw, yaw_error, real_dt_ang / 1000.0f) * real_dt_ang / 1000.0f;
                
                flight_target.target_yaw += yaw_pid_out;
            }

            // 无论转不转，都实时更新扫描基准角，且清理防抖计时器
            base_search_yaw = flight_target.target_yaw; 
            search_time_cnt = 0; 
            is_pausing = 0;      
        }

        // 逻辑B：仅看到单目标（只看到小车），执行扫描寻找信标
        if (locked_lights == 1 && imu_data.z > 0.85 * TARGET_HEIGHT_CM) {
            
            search_time_cnt += real_dt_ang; 
            
            if (search_time_cnt > 300) { 
                
                // 【核心修改】：加入边缘停留等待逻辑
                if (is_pausing) {
                    // 1. 如果处于停留状态，锁死 target_yaw，让飞机机体有时间转到位
                    edge_pause_cnt += real_dt_ang;
                    
                    if (edge_pause_cnt > WAIT_TIME) { // 停留  (你可以根据实际情况调大或调小)
                        is_pausing = 0;         // 停留结束
                        search_dir = -search_dir; // 反转方向，开始往回扫
                    }
                } 
                else {
                    // 2. 正常执行扫描旋转
                    flight_target.target_yaw += search_dir * search_yaw_rate * (real_dt_ang / 1000.0f);
                    
                    // 3. 碰壁检测：一旦达到边界，立即触发停留状态，而不是马上反转
                    if (flight_target.target_yaw > base_search_yaw + MAX_YAW_DEV) {
                        flight_target.target_yaw = base_search_yaw + MAX_YAW_DEV; 
                        is_pausing = 1;     // 触发停留
                        edge_pause_cnt = 0; // 重置停留计时
                    } else if (flight_target.target_yaw < base_search_yaw - MAX_YAW_DEV) {
                        flight_target.target_yaw = base_search_yaw - MAX_YAW_DEV; 
                        is_pausing = 1;     // 触发停留
                        edge_pause_cnt = 0; // 重置停留计时
                    }
                }
            }
        }

        last_locked_lights = locked_lights; 
        Set_Target_Attitude(target_roll_val, target_pitch_val, flight_target.target_yaw);

    } 
    // ================== 完全丢失目标逻辑 ==================
    else {
        comp_row = IMG_CENTER_Y;
        comp_col = IMG_CENTER_X;
        Nonline_PID_Reset(&pid_image_x);
        Nonline_PID_Reset(&pid_image_y);
        Nonline_PID_Reset(&pid_image_yaw); 
        
        Set_Target_Attitude(0, 0, flight_target.target_yaw);
        
        // 清理所有扫描与防抖状态
        search_time_cnt = 0;    
        last_locked_lights = 0; 
        is_pausing = 0;         // 【新增】清理停留状态
        edge_pause_cnt = 0;
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
            search_yaw_rate = val;
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
            search_yaw_rate = val;
            break;
        case 5: // 角速度环 KP
            // pid_g_roll.kp = val;
            // pid_g_pitch.kp = val;
            pid_g_yaw.kp = val;
            break;
        case 6: // 角速度环 KI
            // pid_g_roll.ki = val;
            // pid_g_pitch.ki = val;
            pid_g_yaw.ki = val;
            break;
        case 7: // 角速度环 KD
            // pid_g_roll.kd = val;
            // pid_g_pitch.kd = val;
            pid_g_yaw.kd = val;
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