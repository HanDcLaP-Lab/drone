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

static float Get_Yaw_Error(float target, float current) {
    float error = target - current;
    // 将误差限制在 -180 到 +180 之间，走最短路径
    while (error > 180.0f)  error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
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
    float target_pitch = PID_Calculate(&pid_vel_x, err_vx, CTRL_DT); 
    // float target_pitch = 0; // 
    
    // Y轴速度 (左右) -> Target Roll
    // 假设：右倾为正，向右飞需要正Roll
    float err_vy = flight_target.vel_y_cm_s - imu_data.vy; 
    float target_roll = PID_Calculate(&pid_vel_y, err_vy, CTRL_DT);
    //float target_roll = 0;
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
    // float yaw_err = Get_Yaw_Error(flight_target.target_yaw, imu_data.yaw);
    // float out_yaw = PID_Calculate(&pid_yaw, yaw_err, CTRL_DT);

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


/**
 * @brief 空地协同控制主循环
 * @param car_angle_deg 小车发来的当前自身绝对偏航角 (度)
 */
void Air_Ground_Control_Loop(float car_angle_deg) {
    
    // ================= 1. 状态同步 =================
    // 让无人机的目标偏航角时刻跟随小车，保持机头朝向一致
    // 注意：需要在 Flight_Control_Loop 中使用 target_yaw 进行 PID 控制
    flight_target.target_yaw = car_angle_deg; 

    // ================= 2. 视觉目标搜索 =================
    int car_idx = -1;
    int beacon_idx = -1;
    uint32_t max_size = 0;
    uint32_t second_size = 0;

    // 遍历所有识别到的灯光，通过面积大小区分小车和信标
    // 逻辑：最大的连通域是小车，第二大的是信标
    for (int i = 0; i < cam_down.light_number; i++) {
        // 简单的冒泡逻辑找出第一大和第二大
        if (cam_down.dot_num[i] > max_size) {
            // 原最大变为第二大
            second_size = max_size;
            beacon_idx = car_idx;
            
            // 更新最大为当前
            max_size = cam_down.dot_num[i];
            car_idx = i;
        } else if (cam_down.dot_num[i] > second_size) {
            // 更新第二大
            second_size = cam_down.dot_num[i];
            beacon_idx = i;
        }
    }

    // 如果未找到小车，直接返回，保持当前状态（或悬停）
    if (car_idx == -1) {
        // 可选：丢失目标时原地悬停
        Set_Target_Velocity(0, 0, 0); 
        return; 
    }

    // ================= 3. 获取图像坐标 =================
    // image.c 中: centers[i][0] = Row (Y), centers[i][1] = Col (X)
    float car_row = (float)cam_down.centers[car_idx][0];
    float car_col = (float)cam_down.centers[car_idx][1];
    
    // ================= 4. 无人机位置跟随控制 =================
    // 目标：将小车保持在图像中心
    // 坐标系定义：
    // Row(Y)轴：上小下大。无人机向前飞，景物向下移(Row变大)。
    //           若小车在上方(Row小)，需要无人机向前飞去追。
    //           Error = Center_Row - Car_Row. (60 - 10 = 50 -> 向前)
    // Col(X)轴：左小右大。无人机向右飞，景物向左移(Col变小)。
    //           若小车在右方(Col大)，需要无人机向右飞去追。
    //           Error = Car_Col - Center_Col. (150 - 94 = 56 -> 向右)

    float error_row = IMG_CENTER_Y - car_row; 
    float error_col = car_col - IMG_CENTER_X;

    // 计算目标速度 (cm/s)
    float target_vx = error_row * POS_P_GAIN;
    float target_vy = error_col * POS_P_GAIN;
    
    // 赋值给飞行控制目标 (yaw_rate 传 0，因为我们用 target_yaw 独立控制了)
    Set_Target_Velocity(target_vx, target_vy, 0);


    // ================= 5. 小车导航指令计算 =================
    // 只有当同时看到信标和小车时，才能计算导航路径
    if (beacon_idx != -1) {
        float beacon_row = (float)cam_down.centers[beacon_idx][0];
        float beacon_col = (float)cam_down.centers[beacon_idx][1];

        // 5.1 计算 [无人机坐标系] 下的矢量 (小车 -> 信标)
        // X轴(前): 图像上方为前。若信标在小车前方(Row更小)，dx 应为正。
        //          dx = Car_Row - Beacon_Row
        float vec_x_drone = car_row - beacon_row; 
        
        // Y轴(右): 图像右侧为右。若信标在小车右方(Col更大)，dy 应为正。
        //          dy = Beacon_Col - Car_Col
        float vec_y_drone = beacon_col - car_col;

        // 5.2 计算距离 (像素距离，可根据实际高度换算为物理距离)
        float distance = sqrtf(vec_x_drone * vec_x_drone + vec_y_drone * vec_y_drone);

        // 5.3 计算 [无人机坐标系] 下的角度 (弧度)
        float angle_drone_frame = atan2f(vec_y_drone, vec_x_drone);
        
        // 5.4 坐标系变换: 无人机系 -> 小车系
        // 目标角度(小车系) = 目标角度(无人机系) + (无人机Yaw - 小车Yaw)
        // 因为我们已经在第一步做了 flight_target.target_yaw = car_angle_deg
        // 所以理论上 (yaw_drone - yaw_car) 应该趋近于 0，
        // 但为了动态修正跟随误差，保留这个补偿公式是必要的。

        float yaw_drone_rad = imu_data.yaw * (3.1415926f / 180.0f);
        float yaw_car_rad   = car_angle_deg * (3.1415926f / 180.0f);
        
        float angle_car_frame = angle_drone_frame + (yaw_drone_rad - yaw_car_rad);
        
        // 转换为角度 (-180 ~ 180)
        float send_angle_deg = angle_car_frame * (180.0f / 3.1415926f);
        
        // 归一化到 -180 ~ 180
        while(send_angle_deg > 180.0f)  send_angle_deg -= 360.0f;
        while(send_angle_deg < -180.0f) send_angle_deg += 360.0f;

        // ================= 6. 发送指令 =================
        // 在这里调用你的通信函数，将 distance 和 send_angle_deg 发送给小车
    }
}



//
void Simple_Hover_Control(void) {
    // --- 1. 寻找最大的灯 (视为目标) ---
    int max_idx = -1;
    uint16_t max_size = 0;

    for (int i = 0; i < cam_down.light_number; i++) {
        if (cam_down.dot_num[i] > max_size) {
            max_size = cam_down.dot_num[i];
            max_idx = i;
        }
    }

    // --- 2. 如果没找到灯，原地悬停 ---
    if (max_idx == -1) {
        Set_Target_Velocity(0, 0, 0);
        return;
    }

    // --- 3. 获取目标坐标 ---
    // image.c 中 centers: [0]是Row(Y), [1]是Col(X)
    float target_row = (float)cam_down.centers[max_idx][0];
    float target_col = (float)cam_down.centers[max_idx][1];

    // --- 4. 计算视觉误差 ---
    // Row误差 (代表前后距离): 图像上方Row小。目标在上方(Row<Center) -> error_row > 0
    // Col误差 (代表左右距离): 图像右侧Col大。目标在右侧(Col>Center) -> error_col > 0
    float error_row = IMG_CENTER_Y - target_row; 
    float error_col = target_col - IMG_CENTER_X;

    // --- 5. 交叉映射控制 (关键修改) ---
    
    // [目标：前后移动] 
    // 视觉误差源：error_row
    // 实际控制通道：vel_y (因为底层vel_y控制了Pitch/前后)
    // 极性推导：
    //   目标在前方 (error_row > 0) -> 需要低头前飞 (Pitch Down)
    //   底层逻辑：out_roll (+) 是抬头 (Front+, Back-)。
    //   所以我们需要 out_roll 为负。
    //   Flight_Control_Loop 中：out_roll 来自 target_roll 来自 vel_y。
    //   结论：需要 vel_y 为负。
    float target_vy_output = -1.0f * error_row * HOVER_POS_GAIN;

    // [目标：左右移动]
    // 视觉误差源：error_col
    // 实际控制通道：vel_x (因为底层vel_x控制了Roll/左右)
    // 极性推导：
    //   目标在右侧 (error_col > 0) -> 需要右倾侧飞 (Roll Right)
    //   底层逻辑：out_pitch (+) 是左倾 (Left-, Right+ 还是反的? 需根据混控确认)
    //   根据混控：out_pitch > 0 是 (RF+, LF-) 即左倾/Roll Left。
    //   所以我们需要 out_pitch 为负 (右倾)。
    //   Flight_Control_Loop 中：out_pitch 来自 target_pitch 来自 vel_x。
    //   结论：需要 vel_x 为负。
    float target_vx_output = -1.0f * error_col * HOVER_POS_GAIN;

    // --- 6. 发送指令 ---
    // 注意：这里我们将 视觉计算出的"前后指令" 填入了 vy，"左右指令" 填入了 vx
    Set_Target_Velocity(target_vx_output, target_vy_output, 0);
}