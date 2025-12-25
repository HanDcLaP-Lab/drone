#include "zf_common_headfile.h"

// =================== 全局变量定义 ===================
Flight_Target_t flight_target = {0};
Motor_Output_t motor_out = {0};
//float duty_LF = 5.0f, duty_LB = 5.0f, duty_RF = 5.0f, duty_RB = 5.0f;
// 定义 PID 对象
static PID_t pid_height_vel;
static PID_t pid_height_pos;
static PID_t pid_roll;
static PID_t pid_pitch;
static PID_t pid_yaw; // 预留
static PID_t pid_vel_x;
static PID_t pid_vel_y;

// =================== 内部辅助函数 ===================
// 简单的数值约束
static float Constrain_Float(float val, float min, float max) {
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

// =================== 核心控制逻辑 ===================

void Flight_Control_Init(void) {
    flight_target.cur_state = normal;
    flight_target.is_armed = 2; 
    flight_target.height = 0; 
    
    // ----------- 初始化 PID 参数 (Kp, Ki, Kd, MaxI, MaxOut) -----------
    // 1. 高度环
    PID_Init(&pid_height_pos,  2.0f,  0.0f,  0.0f,    0,  100); // 外环：位置 -> 速度
    PID_Init(&pid_height_vel, 15.0f,  0.5f,  0.0f, 1000, 3000); // 内环：速度 -> 油门

    // 2. 姿态环 (Roll/Pitch)
    PID_Init(&pid_roll,  4.5f, 0.02f, 1.2f, 500, 1000);
    PID_Init(&pid_pitch, 4.5f, 0.02f, 1.2f, 500, 1000);
    PID_Init(&pid_yaw,   6.0f, 0.05f, 0.0f, 500, 1000);

    // 3. 水平速度环
    PID_Init(&pid_vel_x, 0.5f, 0.00f, 0.0f, 10, 20); // X速度 -> Pitch角度
    PID_Init(&pid_vel_y, 0.5f, 0.00f, 0.0f, 10, 20); // Y速度 -> Roll角度
}

void Flight_Unlock(void) {
    flight_target.is_armed = 1;
    // 解锁时重置 PID 积分，防止起飞瞬间暴冲
    PID_Reset(&pid_height_vel);
    PID_Reset(&pid_height_pos); // 也可以重置外环，虽然影响不大
    PID_Reset(&pid_roll);
    PID_Reset(&pid_pitch);
    PID_Reset(&pid_yaw);
}

void Flight_Lock(void) {
    flight_target.is_armed = 0;
    motor_out.rf = motor_out.rb = motor_out.lf = motor_out.lb = 0;
}

void Set_Target_Velocity(float vx, float vy, float yaw_rate) {
    flight_target.vel_x_cm_s = vx;
    flight_target.vel_y_cm_s = vy;
    flight_target.yaw_rate = yaw_rate;
}

// 飞行控制主循环,调用后直接输出PWM
void Flight_Control_Loop(void) {
    if(flight_target.cur_state == pre_landing && imu_data.z < LAND_HEIGHT + 2) flight_target.cur_state = landing;

    //-------------飞行状态-----------------//
    switch (flight_target.cur_state)
    {
    case normal:
        flight_target.target_height = TARGET_HEIGHT_CM;
        break;
    case pre_landing:
        flight_target.target_height = LAND_HEIGHT;
        break;
    case landing:
        Flight_Lock();
        break;
    case brake:
        Flight_Lock();
        break;
    default:
        break;
    }


    if (flight_target.is_armed == 0) {
        motor_out.rf = 0; motor_out.rb = 0; 
        motor_out.lb = 0; motor_out.lf = 0;
        return;
    }

    // ---------------- 1. 高度控制 ----------------
    // 外环：位置误差 -> 目标上升速度

    flight_target.height = flight_target.height*0.94 + flight_target.target_height * 0.06;
    float height_error = flight_target.height - imu_data.z;
    // 计算位置环 PID
    float target_climb_rate = PID_Calculate(&pid_height_pos, height_error, CTRL_DT);
    
    // 内环：速度误差 -> 油门调整量
    float climb_rate_error = target_climb_rate - imu_data.vz;
    // 计算速度环 PID
    float throttle_adj = PID_Calculate(&pid_height_vel, climb_rate_error, CTRL_DT);

    int16_t base_throttle = HOVER_THROTTLE + (int16_t)throttle_adj;
    //printf("%f" , throttle_adj);
    // 安全限幅
    if (base_throttle > MAX_PWM) base_throttle = MAX_PWM;
    if (base_throttle < MIN_PWM) base_throttle = MIN_PWM;

    // ---------------- 2. 水平速度控制 ----------------
    // X轴速度 (前后) -> Target Pitch
    // 假设：抬头为正，向前飞需要负Pitch
    
    float err_vx = flight_target.vel_x_cm_s - imu_data.vx; 
    //float target_pitch = PID_Calculate(&pid_vel_x, err_vx, CTRL_DT); 
    float target_pitch = 0; 
    
    // Y轴速度 (左右) -> Target Roll
    // 假设：右倾为正，向右飞需要正Roll
    float err_vy = flight_target.vel_y_cm_s - imu_data.vy; 
    //float target_roll = PID_Calculate(&pid_vel_y, err_vy, CTRL_DT);
    float target_roll = 0;

    target_pitch = Constrain_Float(target_pitch, -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
    target_roll  = Constrain_Float(target_roll,  -MAX_TILT_ANGLE, MAX_TILT_ANGLE);

    // ---------------- 3. 姿态控制 ----------------
    // Roll PID (注意 imu_data.roll 的符号需符合控制逻辑: 抬头为正)
    float roll_err = target_roll - imu_data.roll; 
    float out_roll = PID_Calculate(&pid_roll, roll_err, CTRL_DT);

    // Pitch PID (左倾为正)
    float pitch_err = target_pitch - imu_data.pitch;
    float out_pitch = PID_Calculate(&pid_pitch, pitch_err, CTRL_DT);

    // Yaw PID (不能置0)
    float yaw_err = 0 - imu_data.yaw;
    float out_yaw = PID_Calculate(&pid_yaw, yaw_err, CTRL_DT);

    // ---------------- 4. 电机混控 ----------------
    motor_out.rf = (int16_t)(base_throttle + out_roll + out_pitch + out_yaw); // 右前
    motor_out.rb = (int16_t)(base_throttle - out_roll + out_pitch - out_yaw); // 右后
    motor_out.lf = (int16_t)(base_throttle + out_roll - out_pitch - out_yaw); // 左前
    motor_out.lb = (int16_t)(base_throttle - out_roll - out_pitch + out_yaw); // 左后
    //printf("%d %f %f %f ",base_throttle , out_roll , out_pitch , out_yaw);
    // ---------------- 5. 输出限幅 ----------------
    int16_t *motors = (int16_t*)&motor_out;
    for(int i=0; i<4; i++) {
        if (motors[i] > MAX_PWM) motors[i] = MAX_PWM;
        if (motors[i] < MIN_PWM) motors[i] = MIN_PWM;
    }
}

void motor_pwm_set(){
    //------------------ 7.输出到PWM --------------------//
    // 只有解锁状态才输出
        pwm_set_duty(PWM_RF, (motor_out.rf/20) + 500);
        pwm_set_duty(PWM_RB, (motor_out.rb/20) + 500);
        pwm_set_duty(PWM_LF, (motor_out.lf/20) + 500);
        pwm_set_duty(PWM_LB, (motor_out.lb/20) + 500);
    
}

void motor_pwm_init(){
    pwm_init(PWM_LF, 50, 500);
    pwm_init(PWM_LB, 50, 500);
    pwm_init(PWM_RF, 50, 500);
    pwm_init(PWM_RB, 50, 500);
}