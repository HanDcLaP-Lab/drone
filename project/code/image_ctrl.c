#include "image_ctrl.h"


// =================== 内部静态状态变量 ===================
static uint32_t last_ang_cnt = 0;
static uint32_t real_dt_ang = 20; 
static uint8_t last_locked_lights = 0; 
static uint8_t has_seen_beacon = 0;
static int8_t search_seq_idx = 0;      // 搜索序列索引 (0~3)
static uint32_t search_wait_timer = 0; // 停留计时器 (ms)
static uint8_t is_turning = 0;         // 是否正在转向中 (0:停留计时, 1:转向中)
static const float search_yaw_seq[8] = {35.0f, 70.0f, 35.0f, 0.0f, -35.0f, -70.0f, -35.0f, 0.0f}; // 目标跳变序列

static int32_t car_en_disable_timer = 0; // 控制 car_en 置零的倒计时器 (ms)
static uint8_t was_aligning = 0;         // 标记飞机之前是否正处于“对准”转动状态
// =================== 内部辅助控制函数 ===================

/**
 * @brief 位置环解耦控制
 */
static void Flight_Hover_Position_Control(float car_pos_x, float car_pos_y, float *out_roll, float *out_pitch) {
    float snapshot_yaw = share_data_from_1[8];
    float yaw_rad = snapshot_yaw * 3.14159265f / 180.0f;
    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);

    float earth_err_x = car_pos_x * cos_yaw - car_pos_y * sin_yaw;
    float earth_err_y = car_pos_x * sin_yaw + car_pos_y * cos_yaw;
    if (fabsf(earth_err_x) < MIN_ERROR) earth_err_x = 0.0f;
    if (fabsf(earth_err_y) < MIN_ERROR) earth_err_y = 0.0f;

    dataC.debug_earth_err_x = earth_err_x;
    dataC.debug_earth_err_y = earth_err_y;

    float dt_sec = real_dt_ang / 1000.0f;
    if (dt_sec < 0.015f) dt_sec = 0.015f;
    if (dt_sec > 0.05f) dt_sec = 0.05f;

    float target_earth_accel_x = Nonline_PID_Calculate(&pid_image_x, earth_err_x, dt_sec);
    float target_earth_accel_y = Nonline_PID_Calculate(&pid_image_y, earth_err_y, dt_sec);

    // 将地球系算出的推力转回当前机体去执行时，必须使用此时此刻的即时偏航角
    float cur_yaw_rad = imu_data.yaw * 3.14159265f / 180.0f;
    float cur_cos_yaw = cosf(cur_yaw_rad);
    float cur_sin_yaw = sinf(cur_yaw_rad);

    float target_body_accel_x = target_earth_accel_x * cur_cos_yaw + target_earth_accel_y * cur_sin_yaw;
    float target_body_accel_y = -target_earth_accel_x * cur_sin_yaw + target_earth_accel_y * cur_cos_yaw;

    *out_pitch = -target_body_accel_x;
    *out_roll  = target_body_accel_y;
}

/**
 * @brief 航向扫描与搜寻状态机
 */
static void Flight_Hover_Yaw_Control(uint8_t locked_lights, float snapshot_yaw) {
    // 逻辑A：当锁定了双目标（看到信标）
    if (locked_lights == 3) {
        if (imu_data.z > 0.85f * TARGET_HEIGHT_CM) has_seen_beacon = 1;
        float target_pos_x = share_data_from_1[5] - dataC.camera_offset_x; 
        float target_pos_y = share_data_from_1[6] - dataC.camera_offset_y; 
        
        float distance = sqrtf(target_pos_x * target_pos_x + target_pos_y * target_pos_y);
        
        // 当距离超出物理死区时才进行角度跟踪
        if (distance >= TARGET_ACC_DISTANCE) {
            // 1. 算出目标相对于机头的相对夹角
            float yaw_error = -atan2f(target_pos_x, target_pos_y) * 180.0f / 3.14159265f;
            yaw_error += YAW_OFFSET;
            // 2. 依然保留极其优秀的“机尾就近对准”逻辑
            if (yaw_error > 90.0f) yaw_error -= 180.0f;
            if (yaw_error < -90.0f) yaw_error += 180.0f;
            
            // 3. 角度死区判定：如果偏角大于死区，才更新目标航向
            if (fabs(yaw_error) >= YAW_MIN_ERROR) {
                flight_target.target_yaw = snapshot_yaw + yaw_error; //使用相机曝光一瞬间的snapshot_yaw
                
                // 4. 叠加全局硬限幅保护
                if (flight_target.target_yaw > TWO_MAX_YAW_DEV) {
                    flight_target.target_yaw = TWO_MAX_YAW_DEV;
                } else if (flight_target.target_yaw < -TWO_MAX_YAW_DEV) {
                    flight_target.target_yaw = -TWO_MAX_YAW_DEV;
                }
            } 
            if(fabs(yaw_error) >= 3 * YAW_MIN_ERROR) {
                was_aligning = 1;
            }else {
                // 【新增】：进入偏角死区，说明对准转动刚刚结束
                if (was_aligning == 1) { 
                    car_en_disable_timer = ROTATE_RECOVER_TIME; // 只触发一次 1000ms 置零
                    was_aligning = 0;            // 触发后立即清除标记
                }
            }
        }else{
            if(was_aligning == 1) was_aligning = 0;
        }

        search_seq_idx = 0;       // 重置方向序列
        search_wait_timer = 0;    // 重置计时器
        is_turning = 0;           // 重置为停留状态
    }else{
        if (was_aligning == 1) {
            was_aligning = 0; 
        }
    }

    // 逻辑B：仅看到单目标（小车），执行定时定角停留 + 定向跳变扫描
    if (locked_lights == 1 && has_seen_beacon == 1) {
        if (!is_turning) {
            // 状态1：已到达目标航向，正在原地停留计时
            search_wait_timer += real_dt_ang;
            
            if (search_wait_timer >= 5000) { 
                flight_target.target_yaw = search_yaw_seq[search_seq_idx];
                
                search_seq_idx++;
                if (search_seq_idx >= (sizeof(search_yaw_seq) / sizeof(search_yaw_seq[0]))) {
                    search_seq_idx = 0;
                }
                is_turning = 1; 
            }
        } else {
            // 状态2：正在向新的 target_yaw 旋转
            float yaw_diff = flight_target.target_yaw - imu_data.yaw;
            
            if (fabsf(yaw_diff) < 3.0f) {
                is_turning = 0;          
                search_wait_timer = 0;
                // 【新增】：一次扫描转动刚刚结束
                //car_en_disable_timer = ROTATE_RECOVER_TIME; // 触发 1000ms 置零
            }
        }
    }
}

// =================== 对外公共任务接口 ===================

void Flight_Hover_Control_Task(void) {
    // 1. 获取目标中心坐标与锁定状态
    float car_pos_x = share_data_from_1[9];
    float car_pos_y = share_data_from_1[12];
    uint8_t locked_lights = (uint8_t)share_data_from_1[14];    
    float snapshot_yaw = share_data_from_1[8];
    
    if (locked_lights == 1 || locked_lights == 3) {
        car_pos_x = car_pos_x - dataC.camera_offset_x;
        car_pos_y = car_pos_y - dataC.camera_offset_y;
    } 

    // 2. 计算真实时间差 dt (防除零)
    if (last_ang_cnt != 0) real_dt_ang = dataC.pit0_cnt - last_ang_cnt;
    last_ang_cnt = dataC.pit0_cnt;

    // ---------- 【新增】: 倒计时器处理及 car_en 赋值 ----------
    if (car_en_disable_timer > 0) {
        car_en_disable_timer -= real_dt_ang;
        car_en = 0; // 倒计时期间强制保持为 0
    } else {
        if(was_aligning == 0){
            car_en = 1;
        }else{
            car_en = 0;
        }
    }

    // ================== 有目标视野逻辑 ==================
    if (locked_lights == 1 || locked_lights == 3) {
        float target_roll_val = 0.0f;
        float target_pitch_val = 0.0f;

        // 1. 调用位置环控制计算期望姿态
        Flight_Hover_Position_Control(car_pos_x, car_pos_y, &target_roll_val, &target_pitch_val);
        
        // 2. 调用航向环控制计算目标 Yaw
        Flight_Hover_Yaw_Control(locked_lights, snapshot_yaw);

        last_locked_lights = locked_lights; 
        Set_Target_Attitude(target_roll_val, target_pitch_val, flight_target.target_yaw);

    } 
    // ================== 完全丢失目标逻辑 ==================
    else {
        Nonline_PID_Reset(&pid_image_x);
        Nonline_PID_Reset(&pid_image_y);
        
        Set_Target_Attitude(0.0f, 0.0f, flight_target.target_yaw);
        
        // 清理所有扫描与防抖状态
        last_locked_lights = 0; 
        has_seen_beacon = 0;
        
        search_seq_idx = 0;
        search_wait_timer = 0;
        is_turning = 0;
        was_aligning = 0;
    }                                                                                                                                                                                         
}
