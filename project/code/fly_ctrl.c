#include "fly_ctrl.h"
#include "pid.h"      // 假设你有这个头文件
#include "imu.h"      // 包含 imu_data
#include "image.h"    // 包含 cam_down

// =================== 全局变量定义 ===================
Flight_Target_t flight_target = {0};
Motor_Output_t motor_out = {0};

// 定义 PID 对象
static PID_t pid_height_vel;
static PID_t pid_height_pos;
static PID_t pid_roll;
static PID_t pid_pitch;
static PID_t pid_yaw;

// =================== 内部辅助函数 ===================
static float Constrain_Float(float val, float min, float max) {
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

// 角度误差处理 (处理 -180 到 180 跳变)
static float Get_Angle_Error(float target, float current) {
    float error = target - current;
    while (error > 180.0f)  error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

// =================== 核心控制逻辑 ===================

void Flight_Control_Init(void) {
    flight_target.cur_state = normal;
    flight_target.is_armed = 2; // 2: 锁定, 0: 待机, 1: 解锁
    flight_target.height = 0; 
    
    // ----------- 初始化 PID 参数 -----------
    // 高度环 (参数需根据动力微调)
    PID_Init(&pid_height_pos,  2.0f,  0.0f,  0.0f,    0,  150); // Pos -> Vel
    PID_Init(&pid_height_vel, 15.0f,  0.5f,  0.0f, 1000, 3000); // Vel -> Throttle

    // 姿态环 (Roll/Pitch) - 这是最内环，Kp 需要响应快
    // 假设输入是角度误差，输出是电机PWM差值
    PID_Init(&pid_roll,  4.5f, 0.02f, 0.2f, 500, 1000);
    PID_Init(&pid_pitch, 4.5f, 0.02f, 0.2f, 500, 1000);
    
    // Yaw 环
    PID_Init(&pid_yaw,   6.0f, 0.05f, 0.0f, 500, 1000);
}

void Flight_Unlock(void) {
    flight_target.is_armed = 1;
    // 解锁瞬间重置积分，防止暴冲
    PID_Reset(&pid_height_vel);
    PID_Reset(&pid_height_pos);
    PID_Reset(&pid_roll);
    PID_Reset(&pid_pitch);
    PID_Reset(&pid_yaw);
    
    // 锁定当前航向为目标航向，防止解锁即转圈
    flight_target.target_yaw = imu_data.yaw;
}

void Flight_Lock(void) {
    flight_target.is_armed = 0;
    motor_out.rf = 0; motor_out.rb = 0; 
    motor_out.lb = 0; motor_out.lf = 0;
}

// 直接设定目标姿态
void Set_Target_Attitude(float roll, float pitch, float yaw) {
    flight_target.target_roll = Constrain_Float(roll, -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
    flight_target.target_pitch = Constrain_Float(pitch, -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
    flight_target.target_yaw = yaw; // Yaw 通常不限幅，是绝对角度
}

// 飞行控制主循环 (建议 500Hz 或 1000Hz 调用)
void Flight_Control_Loop(void) {
    // 1. 状态机处理
    if(flight_target.cur_state == pre_landing && imu_data.z < LAND_HEIGHT + 2) 
        flight_target.cur_state = landing;

    switch (flight_target.cur_state) {
        case normal:      flight_target.target_height = TARGET_HEIGHT_CM; break;
        case pre_landing: flight_target.target_height = LAND_HEIGHT;      break;
        case landing:     Flight_Lock(); return;
        case brake:       Flight_Lock(); return;
        default: break;
    }

    if (flight_target.is_armed == 0) {
        Flight_Lock();
        return;
    }

    // ================= 2. 高度控制 =================
    // 平滑目标高度
    flight_target.height = flight_target.height * 0.94f + flight_target.target_height * 0.06f;
    
    float height_error = flight_target.height - imu_data.z;
    float target_climb_rate = PID_Calculate(&pid_height_pos, height_error, CTRL_DT);
    
    float climb_rate_error = target_climb_rate - imu_data.vz;
    float throttle_adj = PID_Calculate(&pid_height_vel, climb_rate_error, CTRL_DT);

    int16_t base_throttle = HOVER_THROTTLE + (int16_t)throttle_adj;
    base_throttle = (int16_t)Constrain_Float(base_throttle, MIN_PWM, MAX_PWM);

    // ================= 3. 姿态控制 (核心) =================
    // 这里的目标已经是由视觉或上层逻辑直接给出的角度

    // Roll PID
    float roll_err = flight_target.target_roll - imu_data.roll;
    float out_roll = PID_Calculate(&pid_roll, roll_err, CTRL_DT);

    // Pitch PID
    float pitch_err = flight_target.target_pitch - imu_data.pitch;
    float out_pitch = PID_Calculate(&pid_pitch, pitch_err, CTRL_DT);

    // Yaw PID (使用角度环)
    float yaw_err = Get_Angle_Error(flight_target.target_yaw, imu_data.yaw);
    float out_yaw = PID_Calculate(&pid_yaw, yaw_err, CTRL_DT);
    //if(out_pitch > 500 || out_pitch < -500) printf("pitch: %.1f err: %.1f pre_err: %.1f integral:%.1f/r/n" ,out_pitch , pitch_err, pid_pitch.prev_error , pid_pitch.integral);
    // ================= 4. 电机混控 (Quad-X) =================
    // 定义确认：
    // Pitch Out > 0 -> 需要抬头 -> 前电机(LF, RF)加, 后电机(LB, RB)减
    // Roll Out > 0  -> 需要左高 -> 左电机(LF, LB)加, 右电机(RF, RB)减
    // Yaw Out > 0   -> 需要左转(逆时针力矩) -> 顺时针桨(LF, RB)加, 逆时针桨(RF, LB)减
    
    // LF (左前, CW): Base + Pitch + Roll + Yaw
    motor_out.lf = (int16_t)(base_throttle + out_pitch + out_roll + out_yaw);
    
    // RF (右前, CCW): Base + Pitch - Roll - Yaw
    motor_out.rf = (int16_t)(base_throttle + out_pitch - out_roll - out_yaw);
    
    // LB (左后, CCW): Base - Pitch + Roll - Yaw
    motor_out.lb = (int16_t)(base_throttle - out_pitch + out_roll - out_yaw);
    
    // RB (右后, CW): Base - Pitch - Roll + Yaw
    motor_out.rb = (int16_t)(base_throttle - out_pitch - out_roll + out_yaw);

    // ================= 5. 输出限幅 =================
    int16_t *motors = (int16_t*)&motor_out;
    for(int i=0; i<4; i++) {
        if (motors[i] > MAX_PWM) motors[i] = MAX_PWM;
        if (motors[i] < MIN_PWM) motors[i] = MIN_PWM;
    }
}

// 辅助：电机PWM设置
void motor_pwm_set(){
    if (flight_target.is_armed) {
        pwm_set_duty(PWM_RF, (motor_out.rf/20) + 500); // 假设你的电调协议需要这样转换
        pwm_set_duty(PWM_RB, (motor_out.rb/20) + 500);
        pwm_set_duty(PWM_LF, (motor_out.lf/20) + 500);
        pwm_set_duty(PWM_LB, (motor_out.lb/20) + 500);
    } else {
        pwm_set_duty(PWM_RF, 500);
        pwm_set_duty(PWM_RB, 500);
        pwm_set_duty(PWM_LF, 500);
        pwm_set_duty(PWM_LB, 500);
    }
}

void motor_pwm_init(){
    pwm_init(PWM_LF, 50, 500);
    pwm_init(PWM_LB, 50, 500);
    pwm_init(PWM_RF, 50, 500);
    pwm_init(PWM_RB, 50, 500);
}

// 建议放在 code/fly_ctrl.c 末尾

void Debug_Motor_Output_Print(void) {
    // 打印格式：电机位置缩写 : PWM值
    // LF: 左前, RF: 右前, LB: 左后, RB: 右后
    // 观察重点：
    // 1. 解锁后是否有基础油门 (HOVER_THROTTLE)
    // 2. 晃动飞机时，电机数值是否产生差动变化
    printf("PWM: LF:%4d | RF:%4d | LB:%4d | RB:%4d\r\n", 
           motor_out.lf, 
           motor_out.rf, 
           motor_out.lb, 
           motor_out.rb);
}



void Simple_Hover_Control(void) {
    // 1. 检查目标
    if (cam_down.light_number == 0) {
        // 丢失目标时回正
        Set_Target_Attitude(0, 0, flight_target.target_yaw);
        return;
    }

    // 2. 获取最亮点的坐标
    float target_row = (float)cam_down.centers[0][0]; 
    float target_col = (float)cam_down.centers[0][1];

    // 3. 计算经过姿态补偿的角度误差
    float err_pitch_deg, err_roll_deg;
    Get_Attitude_Compensated_Error(target_row, target_col, &err_pitch_deg, &err_roll_deg);

    // 4. 转换为控制指令
    // 此时的 err 已经是度数了 (例如偏离了 5 度)
    // VISUAL_POS_P_GAIN 变成了 "角度环的外环比例"
    // 建议设置为 1.0f 左右。 
    // 含义：目标偏离 5度，我就倾斜 5度 去追。
    
    float target_pitch_val = err_pitch_deg * VISUAL_POS_P_GAIN;
    float target_roll_val  = err_roll_deg  * VISUAL_POS_P_GAIN;

    // 5. 执行
    Set_Target_Attitude(target_roll_val, target_pitch_val, flight_target.target_yaw);
    
    // [调试打印] 
    // 务必观察 err_roll_deg 在晃动飞机时是否稳定接近 0
    // printf("Comp: P_deg:%.1f R_deg:%.1f\r\n", err_pitch_deg, err_roll_deg);
}


void Air_Ground_Control_Loop_New(float car_angle_deg) {
    // 1. 同步 Yaw (让飞机头一直跟着小车转)
    flight_target.target_yaw = car_angle_deg; 

    // 2. 检查目标
    if (cam_down.light_number == 0) {
        Set_Target_Attitude(0, 0, flight_target.target_yaw);
        return;
    }

    // 3. 视觉位置闭环 (始终追踪最大的灯/小车)
    // 代码逻辑同 Simple_Hover_Control
    float car_row = (float)cam_down.centers[0][0]; 
    float car_col = (float)cam_down.centers[0][1];
    float error_row = IMG_CENTER_Y - car_row; 
    float error_col = car_col - IMG_CENTER_X;
    
    float target_pitch_val = -0.32f * error_row * VISUAL_POS_P_GAIN;
    float target_roll_val  =  0.32f * error_col * VISUAL_POS_P_GAIN;
    
    Set_Target_Attitude(target_roll_val, target_pitch_val, flight_target.target_yaw);

    // 4. 计算小车与信标的相对位置 (如果看到了第二个灯)
    // if (cam_down.light_number >= 2) {
    //     float beacon_row = (float)cam_down.centers[1][0];
    //     float beacon_col = (float)cam_down.centers[1][1];
        
    //     // 计算矢量、距离等逻辑...
    //     // ...
    // }
}


// 输入：原始像素坐标
// 输出：去除姿态影响后的真实物理角度误差
void Get_Attitude_Compensated_Error(float raw_row, float raw_col, float *out_err_pitch_deg, float *out_err_roll_deg) {
    // 1. 归一化坐标 (以图像中心为原点)
    // Row(Y)向下为正，Col(X)向右为正
    float y_dist = raw_row - IMG_CENTER_Y; 
    float x_dist = raw_col - IMG_CENTER_X; 

    // 2. 像素转角度 (小孔成像模型)
    // tan(angle) = pixel / f
    // 这里不做去畸变，直接假设是线性的（在中心区域近似成立，边缘虽有误差但能接受）
    float tan_angle_y = y_dist / CAM_F_PIXEL;
    float tan_angle_x = x_dist / CAM_F_PIXEL;

    // 转为角度 (度)
    float angle_cam_y = atanf(tan_angle_y) * 180.0f / PI; // 观测到的俯仰角
    float angle_cam_x = atanf(tan_angle_x) * 180.0f / PI; // 观测到的横滚角

    // 3. 姿态补偿 (核心)
    // 真实角度 = 观测角度 - 机身姿态偏移
    // 符号说明：需要通过 SIGN_PITCH_COMP / SIGN_ROLL_COMP 实测确定
    
    // [Pitch] 机头抬高(Pitch+) -> 目标在图像上下移(y_dist变大) -> angle_cam_y 变大
    // 为了消除这个变大，我们需要减去 Pitch
    float angle_real_pitch = angle_cam_y - (imu_data.pitch * SIGN_PITCH_COMP);

    // [Roll] 左端抬高(Roll+) -> 摄像头看左边 -> 目标在图像上右移(x_dist变大) -> angle_cam_x 变大
    // 为了消除这个变大，我们需要减去 Roll
    // (如果你的Roll定义不同，可能需要变成加上 Roll，即 SIGN_ROLL_COMP 设为 -1)
    float angle_real_roll = angle_cam_x - (imu_data.roll * SIGN_ROLL_COMP);

    // 4. 输出符合 PID 控制方向的误差
    // 之前的逻辑：目标在上方(y负) -> error_row为正 -> 向前飞
    // 现在的 angle_real_pitch 是几何角度。
    // y_dist 负 -> angle_real 负。
    // 我们希望输出 正 (向前飞)。所以取反。
    *out_err_pitch_deg = -angle_real_pitch; 
    
    // 之前的逻辑：目标在右方(x正) -> error_col为正 -> 向右飞
    // x_dist 正 -> angle_real 正。
    // 我们希望输出 正 (向右飞)。保持不变。
    *out_err_roll_deg  = angle_real_roll;   
}