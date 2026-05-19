#include "fly_ctrl.h"

// =================== 全局变量定义 ===================
Flight_Target_t flight_target = {0};
Motor_Output_t motor_out = {0};
int LF=0 , LB=0 , RF=0,RB = 0;
Motor_Offsset_t motor_offset = {0};
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

// =================== 内部辅助函数 ===================
static float Constrain_Float(float val, float min, float max) {
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

// =================== 核心控制逻辑 ===================

void Flight_Control_Init(void) {
    flight_target.cur_state = normal;
    flight_target.is_armed = 2;  // 0: 锁定, 1: 解锁, 2: 等待校准后解锁
    flight_target.height = 0;
    flight_target.start_up_scale = 0.0f;
    flight_target.output_scale = 1.0f; // 默认缩放比例设为1

    // ----------- 初始化 PID 参数 -----------
    // 高度环
    PID_Init(&pid_height_pos, 0.58f, 0.15f, 0.0f, 30, 30, 40.0f);
    PID_Init(&pid_height_vel, 24.0f, 0.0f, 0.2f, 80, 1000, 40.0f);
    // 角度环a
    Nonline_PID_Init(&pid_roll, 9.079f, 0.365f, 0.0f, 0.05f, 20, 150, 40.0f);
    Nonline_PID_Init(&pid_pitch, 9.079f, 0.365f, 0.0f, 0.05f, 20, 150, 40.0f);
    Nonline_PID_Init(&pid_yaw, 1.5f, 0.33f, 0.0f, 0.0228f, 6, 45, 40.0f);

    //Nonline_PID_Init(&pid_image_yaw, 1.0f, 0.00f, 0.0f, 0.0f, 0, 60.0f, 4.0f);
    // 角速度环g
    PID_Init(&pid_g_roll, 2.777f, 3.327f, 0.101f, 120, 3500, 40.0f);
    PID_Init(&pid_g_pitch, 2.777f, 13.327f, 0.101f, 120, 3500, 40.0f);
    PID_Init(&pid_g_yaw, 4.54f, 1.32f, 0.00f, 150, 3500, 40.0f);
    // 视觉部分
    Nonline_PID_Init(&pid_image_x, 0.079f, 0.0137f, 0.0772f, 0.00f, 50, 6.0 , 10.0f);
    Nonline_PID_Init(&pid_image_y, 0.079f, 0.0137f, 0.0772f, 0.00f, 50, 6.0 , 10.0f);

}

void Flight_Unlock(void) {
    flight_target.is_armed = 1;
    flight_target.start_up_scale = 0.0f;

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
                if (flight_target.start_up_scale < 1.0f) {
                    flight_target.start_up_scale += 0.0005f;  // 约2秒加满 (1ms周期)
                }
            }
            break;
        case pre_landing:
            flight_target.target_height = LAND_HEIGHT;
            break;
        case landing:
            // 降落阶段逐渐减小油门比例
            if (flight_target.start_up_scale > 0) {
                flight_target.start_up_scale -= 0.002f;
            }
            break;
        default:
            break;
    }
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

    
    // 倾角补偿
    float roll_rad = imu_data.roll * (PI / 180.0f);
    float pitch_rad = imu_data.pitch * (PI / 180.0f);
    float cos_tilt = cosf(roll_rad) * cosf(pitch_rad);
    float compensation_factor = 1.0f;
    if (cos_tilt > 0.1f) { // 避免除以过小的数
        compensation_factor = 1.0f / cos_tilt;
    }
    // 限制补偿系数
    compensation_factor = Constrain_Float(compensation_factor, 1.0f, 1.5f); 
    return (int16_t)((HOVER_THROTTLE + throttle_adj) * compensation_factor);
}

void Flight_Control_Angle(void) {
    // 1. 计算误差 (绝对系)
    float roll_error = flight_target.target_roll - imu_data.roll;
    float pitch_error = flight_target.target_pitch - imu_data.pitch;
    float yaw_error = flight_target.target_yaw - imu_data.yaw;

    // 2. PID 计算 (输出即视为机体角速度目标，基于小角度假设)
    flight_target.target_g_roll = Nonline_PID_Calculate(&pid_roll, roll_error, CTRL_DT_CTLOOP);
    flight_target.target_g_pitch = Nonline_PID_Calculate(&pid_pitch, pitch_error, CTRL_DT_CTLOOP);
    flight_target.target_g_yaw = Nonline_PID_Calculate(&pid_yaw, yaw_error, CTRL_DT_CTLOOP);
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
    motor_offset.lf = (int16_t)( PITCH_OFFSET + ROLL_OFFSET);
    motor_offset.rf = (int16_t)( PITCH_OFFSET - ROLL_OFFSET);
    motor_offset.lb = (int16_t)(-PITCH_OFFSET + ROLL_OFFSET);
    motor_offset.rb = (int16_t)(-PITCH_OFFSET - ROLL_OFFSET);
    // LF (左前, CW): Base + Pitch + Roll - Yaw
    motor_out.lf = (int16_t)((base_throttle + out_pitch + out_roll + out_yaw + motor_offset.lf) * flight_target.start_up_scale * flight_target.output_scale);

    // RF (右前, CCW): Base + Pitch - Roll + Yaw
    motor_out.rf = (int16_t)((base_throttle + out_pitch - out_roll - out_yaw + motor_offset.rf) * flight_target.start_up_scale * flight_target.output_scale);

    // LB (左后, CCW): Base - Pitch + Roll + Yaw
    motor_out.lb = (int16_t)((base_throttle - out_pitch + out_roll - out_yaw + motor_offset.lb) * flight_target.start_up_scale * flight_target.output_scale);

    // RB (右后, CW): Base - Pitch - Roll - Yaw
    motor_out.rb = (int16_t)((base_throttle - out_pitch - out_roll + out_yaw + motor_offset.rb) * flight_target.start_up_scale * flight_target.output_scale);

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

    // 5. 高度环控制 (计算基础油门)
    int16_t base_throttle = Flight_Control_Height();

    // 6. 电机混控与输出
    Flight_Motor_Mix(base_throttle, motor_out.roll, motor_out.pitch, motor_out.yaw);
}

void motor_pwm_set() {
    if (current_drone_state == DRONE_STATE_DEBUG) {
        small_driver_set_duty(0, 0, 0, 0);
        return;
    }

    if (flight_target.is_armed == 1) {
        // 注意：UART 驱动通常直接接受逻辑占空比（如 0-10000），不再需要 PWM 的 4000 偏置
        //small_driver_set_duty(LF, RF, RB,LB);
        small_driver_set_duty(motor_out.lf, motor_out.rf, motor_out.rb,motor_out.lb);
        // small_driver_set_duty(2000, 2000, 2000, 2000);
    } else {
        motor_out.rf = 0;
        motor_out.rb = 0;
        motor_out.lf = 0;
        motor_out.lb = 0;
        small_driver_set_duty(0, 0, 0, 0);
    }
}

void motor_pwm_init() {
    // 初始化无刷驱动的串口通讯
    small_driver_uart_init();
}
