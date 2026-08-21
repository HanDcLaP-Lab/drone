#include "image_ctrl.h"

// ******************************************************************************
// 视觉悬停控制 (位置环 + 偏航搜索状态机)
//
//   Flight_Hover_Control_Task() (CM7_0主循环, ~10ms周期, 100Hz摄像头)
//        │
//   ┌────┴───────────────────────────────────────────────────┐
//   │ 1. 读取Core1原始小车坐标与锁定状态                      │
//   │ 2. 大地系偏心补偿与滤波 → 位置环PID                     │
//   │    ├─ 快照Yaw → 地球系误差 → PID → 目标加速度          │
//   │    └─ 实时Yaw → 转回机体坐标系 → target_roll/pitch     │
//   │ 4. Flight_Hover_Yaw_Control()  偏航搜索状态机          │
//   │    ├─ locked_lights=3 (双目标): 跟踪信标方向            │
//   │    ├─ locked_lights=1 (仅小车): 定时定角跳变搜索        │
//   │    │   └─ 序列: 35→70→35→0→-35→-70→-35→0 (度)       │
//   │    └─ locked_lights=0/2 (全丢/仅信标): 回平等待        │
//   │ 5. Set_Target_Attitude() → fly_ctrl 的PID输入          │
//   └──────────────────────────────────────────────────────┘
// ******************************************************************************

// =================== 前馈偏移状态 (见 Car_Position_Predict_Feedforward) ===================
float ff_event_deg = -1.0f;          // 已应用前馈事件的角度 (deg, -1=尚未收到过, 供打印函数换算修正角)
float ff_off_x = 0.0f;              // 事件时刻的修正后地面系偏移向量X (cm, 供打印函数换算角度)
float ff_off_y = 0.0f;              // 事件时刻的修正后地面系偏移向量Y (cm, 供打印函数换算角度)
static uint32_t ff_event_ms = 0;    // 事件时刻 (dataC.pit0_cnt, ms)
float ff_disp_dir_deg = 0.0f;       // 当前前馈方向 (deg, 机体系 0°=机头), 供 CM7_1 屏幕显示
float ff_disp_remain_cm = 0.0f;     // 当前前馈剩余偏移量 (cm, 0=无前馈; >0 时方向有效), 供 CM7_1 屏幕显示

/**
 * @brief 小车方向前馈: 收到小车前馈角 (上行 ff_deg) 时, 将小车位置向该方向抛出
 *        FF_THROW_DIST_CM (1m), 在 FF_CONVERGE_MS (1s) 内线性收敛回真实小车位置,
 *        残余误差由位置环 KI 承接 (交棒)。
 * 前馈始终只是叠加在实时小车坐标上的一个向量 (地面系方向, 随帧旋入机体系), 不替代小车位置;
 * 启用条件由调用方保证: 仅小车可见 (locked 1/3), 即未触发丢失回平。
 */
static void Car_Position_Predict_Feedforward(float *car_pos_x, float *car_pos_y, float snapshot_yaw) {
#if defined(CY_CORE_CM7_0) && CAR_FF_ENABLE
    // 1. 从双向通信提取最新待处理前馈角 (支持 100ms 保留窗口)
    float raw_ff = Duplex_Get_Pending_Feedforward();

    if (raw_ff >= 0.0f) {
        if (raw_ff != ff_event_deg) {
            // 新前馈事件: 抛一次偏移并记录事件时刻 (小车确认前重传的相同角度不重复触发)
            ff_event_deg = raw_ff;
            ff_event_ms  = dataC.pit0_cnt;

            // 1. 光流速度转地面系 (世界坐标)
            float ev_yaw_rad = VISION_EARTH_YAW_DEG(snapshot_yaw) * 3.14159265f / 180.0f;
            float ev_cos = cosf(ev_yaw_rad);
            float ev_sin = sinf(ev_yaw_rad);
            float flow_earth_x = upixels_data.filt_vel_x * ev_cos - upixels_data.filt_vel_y * ev_sin;
            float flow_earth_y = upixels_data.filt_vel_x * ev_sin + upixels_data.filt_vel_y * ev_cos;

            // 2. 计算当前飞行速度方向与预期前馈方向的夹角 [0, 180度]
            float angle_scale = 1.0f;
            float v_speed = sqrtf(flow_earth_x * flow_earth_x + flow_earth_y * flow_earth_y);
#if defined(FF_REVERSE_GAIN)
            if (FF_REVERSE_GAIN != 1.0f && v_speed > 5.0f) {
                float vel_deg = atan2f(flow_earth_y, flow_earth_x) * (180.0f / 3.14159265f);
                if (vel_deg < 0.0f) vel_deg += 360.0f;
                float angle_diff = fabsf(raw_ff - vel_deg);
                if (angle_diff > 180.0f) angle_diff = 360.0f - angle_diff;

                // 在 90~180 度区间线性增大响应幅值 (90°对应1.0, 180°对应FF_REVERSE_GAIN)
                if (angle_diff > 90.0f) {
                    angle_scale = 1.0f + (FF_REVERSE_GAIN - 1.0f) * ((angle_diff - 90.0f) / 90.0f);
                }
            }
#endif

            // 3. 基础抛出量计算 (含大角度/掉头增益缩放)
            float rad = raw_ff * 3.14159265f / 180.0f;
            float throw_dist = FF_THROW_DIST_CM * angle_scale;
            float base_off_x = cosf(rad) * throw_dist;   // 地面系偏移向量 (事件方向, 世界固定)
            float base_off_y = sinf(rad) * throw_dist;

            // 4. 光流速度修正: kick 事件瞬间扣除当前实际速度折算位移
            ff_off_x = base_off_x - FF_FLOW_CORRECTION_S * flow_earth_x;
            ff_off_y = base_off_y - FF_FLOW_CORRECTION_S * flow_earth_y;
        }
        duplex_ff_deg_received = 1U; // 收到有效前馈角且已采纳: 向小车回发 ACK
    } else {
        duplex_ff_deg_received = 0U; // 无待处理前馈或已过期 (-1): 清除 ACK, 防止下一次前馈触发时误读旧 ACK
    }

    // 收敛: 事件后 FF_CONVERGE_MS 内衰减到 0 (世界方向恒定, 每帧旋入当前机体系);
    // 抛出量先经 FF_THROW_RAMP_MS 斜坡升至峰值, 消除阶跃对位置环/姿态链的冲击 (原地下坠源)
    uint32_t elapsed = dataC.pit0_cnt - ff_event_ms;
    float k = 1.0f - (float)elapsed / (float)FF_CONVERGE_MS;
    if (k < 0.0f) k = 0.0f;
    float ramp = (float)elapsed / (float)FF_THROW_RAMP_MS;
    if (ramp > 1.0f) ramp = 1.0f;
    float throw_scale = k * ramp;
    float earth_off_x = ff_off_x * throw_scale;
    float earth_off_y = ff_off_y * throw_scale;

    // 地面系 → 机体系 (按本帧快照偏航), 叠加到机体系小车坐标
    float yaw_rad = VISION_EARTH_YAW_DEG(snapshot_yaw) * 3.14159265f / 180.0f;
    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);
    *car_pos_x += earth_off_x * cos_yaw + earth_off_y * sin_yaw;
    *car_pos_y += -earth_off_x * sin_yaw + earth_off_y * cos_yaw;

    // 供 CM7_1 屏幕绘制: 方向 = 事件地面系方向旋入机体系 (= 飞行实际施加方向, 屏幕映射为机体系);
    // 上行值仅在车端发送期有效(≥0), 若取上行值, 整个 1s 收敛期几乎都读到 -1=无前馈,
    // 短线会被画成固定 0° 机头方向。
    ff_disp_dir_deg = (k > 0.0f) ? (ff_event_deg - VISION_EARTH_YAW_DEG(snapshot_yaw)) : 0.0f;
    ff_disp_remain_cm = sqrtf(earth_off_x * earth_off_x + earth_off_y * earth_off_y);
#else
    (void)car_pos_x; (void)car_pos_y; (void)snapshot_yaw;   // CM7_1 构建或前馈关闭时空实现
#endif
}

/** @brief 复位前馈偏移状态 (丢失回平/视觉失联时调用, 与 PID/滤波器复位同步) */
void Car_Feedforward_Reset(void) {
    ff_event_deg = -1.0f;
    ff_off_x = 0.0f;
    ff_off_y = 0.0f;
    ff_disp_dir_deg = 0.0f;
    ff_disp_remain_cm = 0.0f;
#if defined(CY_CORE_CM7_0)
    Duplex_Clear_Pending_Feedforward();
    duplex_ff_deg_received = 0;      // 复位后不再对旧前馈回 ack, 让小车在可采纳时重传
#endif
}

// =================== 内部静态状态变量 ===================
static uint32_t last_ang_cnt = 0;
static uint32_t real_dt_ang = 10;   // 初始值 10ms (100Hz 帧周期)
static uint8_t last_locked_lights = 0; 
static uint8_t has_seen_beacon = 0;
static int8_t search_seq_idx = 0;      // 搜索序列索引 (0~3)
static uint32_t search_wait_timer = 0; // 停留计时器 (ms)
static uint8_t is_turning = 0;         // 是否正在转向中 (0:停留计时, 1:转向中)
static const float search_yaw_seq[SEARCH_YAW_SEQ_NUM] = SEARCH_YAW_SEQ_ARRAY; // 目标跳变序列

static uint8_t was_aligning = 0;         // 标记飞机之前是否正处于”对准”转动状态
static uint8_t lost_frames = 0;          // 连续丢失目标帧数 (防单帧噪点误触发回平)
static KalmanFilter1 hover_car_earth_x_filter;
static KalmanFilter1 hover_car_earth_y_filter;
static uint8_t hover_car_filter_initialized = 0;

/**
 * @brief 进入光流速度环时复位视觉侧状态 (由 opticalflow_ctrl 调用)
 */
void Image_Hover_Reset_For_OpticalFlow(void) {
    Nonline_PID_Reset(&pid_image_x);
    Nonline_PID_Reset(&pid_image_y);
    hover_car_filter_initialized = 0;
    Car_Feedforward_Reset();
    lost_frames = 0;
    last_locked_lights = 0;
    has_seen_beacon = 0;
    search_seq_idx = 0;
    search_wait_timer = 0;
    is_turning = 0;
    was_aligning = 0;
}

/**
 * @brief 视觉长时间失联时的安全保护处理 (清理积分、复位前馈、高高度平滑回平)
 */
void Flight_Hover_Lost_Protection(void) {
    Nonline_PID_Reset(&pid_image_x);
    Nonline_PID_Reset(&pid_image_y);
    Car_Feedforward_Reset();
    if (Flight_Get_Nav_Mode() == NAV_MODE_VISION_HOVER) {
        Flight_Set_Target_Attitude_Smoothed(0.0f, 0.0f, flight_target.target_yaw, 0.01f);
    }
    last_locked_lights = 0;
    has_seen_beacon = 0;
    search_seq_idx = 0;
    search_wait_timer = 0;
    is_turning = 0;
    was_aligning = 0;
    lost_frames = LOST_TOLERANCE_FRAMES;
}

/**
 * @brief 在小车固定地面系扣除悬停点偏移并滤波，输出供飞控使用的机体系位置
 */
static void Hover_Car_Position_Filter(float raw_car_x, float raw_car_y, float snapshot_yaw,
                                      float *filtered_car_x, float *filtered_car_y) {
    float yaw_rad = VISION_EARTH_YAW_DEG(snapshot_yaw) * 3.14159265f / 180.0f;
    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);
    float offset_yaw_rad = VISION_EARTH_YAW_DEG(CAM_OFFSET_MEASURE_YAW_DEG) *
                           3.14159265f / 180.0f;
    float cos_offset_yaw = cosf(offset_yaw_rad);
    float sin_offset_yaw = sinf(offset_yaw_rad);

    float raw_earth_x = raw_car_x * cos_yaw - raw_car_y * sin_yaw;
    float raw_earth_y = raw_car_x * sin_yaw + raw_car_y * cos_yaw;
    float offset_earth_x = dataC.camera_offset_x * cos_offset_yaw -
                           dataC.camera_offset_y * sin_offset_yaw;
    float offset_earth_y = dataC.camera_offset_x * sin_offset_yaw +
                           dataC.camera_offset_y * cos_offset_yaw;
    float hover_earth_x = raw_earth_x - offset_earth_x;
    float hover_earth_y = raw_earth_y - offset_earth_y;

    if (!hover_car_filter_initialized) {
        Kalman_Init(&hover_car_earth_x_filter, IMAGE_POS_KALMAN_Q, IMAGE_POS_KALMAN_R, hover_earth_x);
        Kalman_Init(&hover_car_earth_y_filter, IMAGE_POS_KALMAN_Q, IMAGE_POS_KALMAN_R, hover_earth_y);
        hover_car_filter_initialized = 1;
    } else {
        hover_earth_x = Kalman_Update(&hover_car_earth_x_filter, hover_earth_x);
        hover_earth_y = Kalman_Update(&hover_car_earth_y_filter, hover_earth_y);
    }

    *filtered_car_x = hover_earth_x * cos_yaw + hover_earth_y * sin_yaw;
    *filtered_car_y = -hover_earth_x * sin_yaw + hover_earth_y * cos_yaw;
}

/**
 * @brief 位置环解耦控制
 */
static void Flight_Hover_Position_Control(float car_pos_x, float car_pos_y, float *out_roll, float *out_pitch) {
    float snapshot_yaw = vision_snap[S1_SNAPSHOT_YAW];
    float yaw_rad = VISION_EARTH_YAW_DEG(snapshot_yaw) * 3.14159265f / 180.0f;
    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);

    float earth_err_x = car_pos_x * cos_yaw - car_pos_y * sin_yaw;
    float earth_err_y = car_pos_x * sin_yaw + car_pos_y * cos_yaw;
    if (fabsf(earth_err_x) < MIN_ERROR) earth_err_x = 0.0f;
    if (fabsf(earth_err_y) < MIN_ERROR) earth_err_y = 0.0f;

    dataC.debug_earth_err_x = earth_err_x;
    dataC.debug_earth_err_y = earth_err_y;

    float dt_sec = real_dt_ang / 1000.0f;
    // 100Hz 帧周期 10ms: 下限 5ms 容忍帧率抖动/漏帧, 上限 50ms 防 dt 测量异常 (失联保护由 VISION_LOST_TIMEOUT_MS 兜底)
    if (dt_sec < 0.005f) dt_sec = 0.005f;
    if (dt_sec > 0.05f) dt_sec = 0.05f;

    float target_earth_accel_x = Nonline_PID_Calculate(&pid_image_x, earth_err_x, dt_sec);
    float target_earth_accel_y = Nonline_PID_Calculate(&pid_image_y, earth_err_y, dt_sec);

    // 将地球系算出的推力转回当前机体去执行时，必须使用此时此刻的即时偏航角
    float cur_yaw_rad = VISION_EARTH_YAW_DEG(imu_data.yaw) * 3.14159265f / 180.0f;
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
    // 降落阶段不再更新或覆盖偏航目标角，保持设定的360度
    if (flight_target.cur_state == FLIGHT_STATE_PRE_LANDING) return;
    // 逻辑A：当锁定了双目标（看到信标）
#if TARGET_ALIGN_ENABLE
    if (locked_lights == 3) {
        if (imu_data.z > 0.85f * TARGET_HEIGHT_CM) has_seen_beacon = 1;
        float target_pos_x = vision_snap[S1_TARGET_X] - dataC.camera_offset_x;
        float target_pos_y = vision_snap[S1_TARGET_Y] - dataC.camera_offset_y;
        
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
            if (fabsf(yaw_error) >= YAW_MIN_ERROR) {
                flight_target.target_yaw = snapshot_yaw + yaw_error; // 使用相机曝光一瞬间的snapshot_yaw
                // 4. 叠加全局硬限幅保护
                if (flight_target.target_yaw > TWO_MAX_YAW_DEV) {
                    flight_target.target_yaw = TWO_MAX_YAW_DEV;
                } else if (flight_target.target_yaw < -TWO_MAX_YAW_DEV) {
                    flight_target.target_yaw = -TWO_MAX_YAW_DEV;
                }
            } 
            if(fabsf(yaw_error) >= 3 * YAW_MIN_ERROR) {
                was_aligning = 1;
            }else {
                // 【新增】：进入偏角死区，说明对准转动刚刚结束
                if (was_aligning == 1) { 
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
#else
    flight_target.target_yaw = TARGET_ALIGN_DISABLE_YAW; // 当前为特殊值禁用对准

    // float target_yaw_set[12]={0,30,60,90,60,30,0,-30,-60,-90,-60,-30};
    // if (dataC.pit0_cnt > 20000U) {
    //     uint32_t target_yaw_idx = ((dataC.pit0_cnt - 20000U) / 5000U) % 12U;
    //     flight_target.target_yaw = target_yaw_set[target_yaw_idx];
    // }

#endif

    // 逻辑B：仅看到单目标（小车），执行定时定角停留 + 定向跳变扫描
#if SEARCH_YAW_ENABLE
    static uint8_t search_beacon_ready = 0;
    static uint8_t search_loss_active = 0;
    static uint32_t search_loss_start_ms = 0;
    static float search_target_yaw = 0.0f;

    if (locked_lights == 3) {
        if (imu_data.z > 100.0f) search_beacon_ready = 1;
        search_loss_active = 0;
        search_loss_start_ms = 0;
        search_seq_idx = 0;
        search_wait_timer = 0;
        is_turning = 0;
    } else if (locked_lights == 1) {
        // 只要处于state1且搜索尚未激活，就立即开始搜索。
        if (!search_loss_active) {
            search_loss_active = 1;
            search_loss_start_ms = dataC.pit0_cnt;
            search_seq_idx = 0;
            search_target_yaw = search_yaw_seq[search_seq_idx++];
            search_wait_timer = 0;
            is_turning = 1;
        }

        if (search_loss_active) {
            uint32_t search_elapsed_ms = dataC.pit0_cnt - search_loss_start_ms;
            // 仅当曾经见过信标时才启用搜索超时
            if (search_beacon_ready && ff_event_ms > 0 && search_elapsed_ms >= SEARCH_TIMEOUT) {
                search_loss_active = 0;
                search_wait_timer = 0;
                is_turning = 0;
                Flight_Request_Landing();
            } else if (search_elapsed_ms >= SEARCH_START_DELAY) {
                // 对准关闭分支会每帧写入0度，因此搜索期间必须持续恢复当前搜索目标。
                flight_target.target_yaw = search_target_yaw;

                if (!is_turning) {
                    search_wait_timer += real_dt_ang;
                    if (search_wait_timer >= SEARCH_WAIT_TIME) {
                        search_target_yaw = search_yaw_seq[search_seq_idx++];
                        flight_target.target_yaw = search_target_yaw;
                        if (search_seq_idx >= SEARCH_YAW_SEQ_NUM) search_seq_idx = 0;
                        search_wait_timer = 0;
                        is_turning = 1;
                    }
                } else if (fabsf(search_target_yaw - imu_data.yaw) < 3.0f) {
                    search_wait_timer = 0;
                    is_turning = 0;
                }
            }
        }
    }
#endif
}

// =================== 对外公共任务接口 ===================

void Flight_Hover_Control_Task(void) {
    // 1. 获取目标中心坐标与锁定状态 (读主循环的一致性快照, 不直读共享区)
    float car_pos_x = vision_snap[S1_CAR_RAW_X];
    float car_pos_y = vision_snap[S1_CAR_RAW_Y];
    uint8_t locked_lights = (uint8_t)vision_snap[S1_LOCKED_COUNT];
    float snapshot_yaw = vision_snap[S1_SNAPSHOT_YAW];
    
    // 2. 计算真实时间差 dt (防除零)
    if (last_ang_cnt != 0) real_dt_ang = dataC.pit0_cnt - last_ang_cnt;
    last_ang_cnt = dataC.pit0_cnt;

    // 视觉环使用真实帧间隔作为平滑/控制 dt
    float dt_sec = real_dt_ang / 1000.0f;
    if (dt_sec < 0.005f) dt_sec = 0.005f;
    if (dt_sec > 0.05f) dt_sec = 0.05f;

    if (locked_lights == 1 || locked_lights == 3) {
        Hover_Car_Position_Filter(car_pos_x, car_pos_y, snapshot_yaw, &car_pos_x, &car_pos_y);

        // 3. 小车方向前馈: 条件放松, 仅小车可见即可 (未触发丢失回平), 叠加于实时小车坐标之上
        Car_Position_Predict_Feedforward(&car_pos_x, &car_pos_y, snapshot_yaw);
    } else if (lost_frames < LOST_TOLERANCE_FRAMES) {
        // [新增] 丢失容忍期内仍维持飞行: 即使 locked_lights 不为 1/3,
        // 只要还没进入丢失回平, 也尝试采纳前馈, 避免短暂丢失期间把前馈丢掉。
        Car_Position_Predict_Feedforward(&car_pos_x, &car_pos_y, snapshot_yaw);
    }

    if (locked_lights == 1 || locked_lights == 3) {
        dataC.debug_body_track_x = car_pos_x;
        dataC.debug_body_track_y = car_pos_y;
    }

    // car_en 只由飞控锁定/解锁/降落/急停维护，视觉对准不再让小车完全停止。

    // ================== 有目标视野逻辑 ==================
    if (locked_lights == 1 || locked_lights == 3) {
        float target_roll_val = 0.0f;
        float target_pitch_val = 0.0f;

        // 1. 调用位置环控制计算期望姿态
        Flight_Hover_Position_Control(car_pos_x, car_pos_y, &target_roll_val, &target_pitch_val);
        
        // 2. 调用航向环控制计算目标 Yaw
        Flight_Hover_Yaw_Control(locked_lights, snapshot_yaw);

        last_locked_lights = locked_lights;
        lost_frames = 0; // 有目标，清零丢失计数器
        Flight_Set_Target_Attitude_Smoothed(target_roll_val, target_pitch_val, flight_target.target_yaw, dt_sec);

    } 
    // ================== 完全丢失目标逻辑 ==================
    else {
        lost_frames++;
        if (lost_frames >= LOST_TOLERANCE_FRAMES) {
            Nonline_PID_Reset(&pid_image_x);
            Nonline_PID_Reset(&pid_image_y);
            hover_car_filter_initialized = 0;
            Car_Feedforward_Reset(); // [新增] 丢失回平同步清前馈偏移

            Flight_Set_Target_Attitude_Smoothed(0.0f, 0.0f, flight_target.target_yaw, dt_sec);

            // 清理所有扫描与防抖状态
            last_locked_lights = 0;
            has_seen_beacon = 0;

            search_seq_idx = 0;
            search_wait_timer = 0;
            is_turning = 0;
            was_aligning = 0;

            lost_frames = LOST_TOLERANCE_FRAMES; // 钳位，防溢出
        }
    }                                                                                                                                                                                         
}
