#include "data_complex.h"
#include "zf_common_headfile.h"
#include "image.h"

Data_Complex_t dataC = {0};
uint8_t car_en = 1;
//核间通信初始化
#if defined(CY_CORE_CM7_0)
    //Core 0
    
    #pragma location = 0x28001040
    volatile float share_data_from_0[M7_x_DATA_LENGTH] = {0}; // Core 0 定义并负责清零
    
    #pragma location = 0x28001000
    __root __no_init volatile float share_data_from_1[M7_x_DATA_LENGTH]; // 对 Core 1 的数据只读，不初始化

#elif defined(CY_CORE_CM7_1)
    //Core 1
    
    #pragma location = 0x28001000
    volatile float share_data_from_1[M7_x_DATA_LENGTH] = {0}; // Core 1 定义并负责清零
    
    #pragma location = 0x28001040
    __root __no_init volatile float share_data_from_0[M7_x_DATA_LENGTH]; // 对 Core 0 的数据只读，不初始化

#else
    #error "Unknown Core"
#endif

#if defined(CY_CORE_CM7_0)


static uint8_t send_buffer[36]; // 发送缓冲区：2字节帧头 + 32字节(8个float) + 1字节校验和 + 1字节帧尾 = 36字节
// ================= 板间通讯协议配置 =================
#define FRAME_HEADER1 0xAA
#define FRAME_HEADER2 0x55
#define FRAME_TAIL    0x7F
// ================= 通讯初始化 =================
void Board_Comm_Init(void)
{
    // 初始化配置好的串口
    uart_init(BOARD_UART, BOARD_BAUDRATE, BOARD_TX_PIN, BOARD_RX_PIN);
}

// ================= 打包并发送函数 =================
// 只要传入长度为 8 的 float 数组首地址即可
void Board_Comm_Send_Data(volatile float *data_array)
{
    // 1. 填入帧头
    send_buffer[0] = FRAME_HEADER1;
    send_buffer[1] = FRAME_HEADER2;
    
    // 2. 将传入的 float 数组拷贝到发送缓冲区
    // 先读入局部变量以保留 volatile 读取语义，再用 memcpy 处理对齐
    float local_data[8];
    for (int i = 0; i < 8; i++) {
        local_data[i] = data_array[i];
    }
    memcpy(&send_buffer[2], local_data, sizeof(local_data));
    
    // 3. 计算简单的累加校验和 (只校验数据区的32字节)
    uint8_t checksum = 0;
    for (int i = 2; i < 34; i++) {
        checksum += send_buffer[i];
    }
    
    // 4. 填入校验和与帧尾
    send_buffer[34] = checksum;
    send_buffer[35] = FRAME_TAIL;
    
    // 5. 物理发送整包数据 (36字节)
    uart_write_buffer(BOARD_UART, send_buffer, sizeof(send_buffer));
}

//===========================通讯传递数组赋值==============================
//请将所有通讯赋值在下面三个函数完成

void M7_0_data_send(volatile float* data_out) { // Core 0 调用，写入share_data_from_0
    data_out[S0_IMU_ROLL]    = imu_data.roll;
    data_out[S0_IMU_PITCH]   = imu_data.pitch;
    data_out[S0_IMU_YAW]     = imu_data.yaw;

    data_out[S0_IMU_HEIGHT]  = imu_data.z;
    data_out[S0_DRONE_STATE] = (float)current_drone_state;
    data_out[S0_MOTOR_LF]    = motor_out.lf;
    data_out[S0_MOTOR_RF]    = motor_out.rf;
    data_out[S0_MOTOR_LB]    = motor_out.lb;
    data_out[S0_MOTOR_RB]    = motor_out.rb;

    data_out[S0_TARGET_ROLL]  = flight_target.target_roll;
    data_out[S0_TARGET_PITCH] = flight_target.target_pitch;
    data_out[S0_TARGET_YAW]   = flight_target.target_yaw;
    data_out[S0_DEBUG_ERR_X]  = dataC.debug_earth_err_x;
    data_out[S0_DEBUG_ERR_Y]  = dataC.debug_earth_err_y;

}

// **************************** 下传协议映射 (无人机→小车) ****************************
// buffer[8] 索引映射，与小车端 uart_data[8] 一一对应 (协议帧: 0xAA 0x55 + 8×float + 校验和 + 0x7F):
//   [0] car_ground_pos.x    — 卡尔曼滤波后小车X坐标 (cm)     ← S1_K_CAR_X (pos.k_car.x)
//   [1] car_ground_pos.y    — 卡尔曼滤波后小车Y坐标 (cm)     ← S1_K_CAR_Y (pos.k_car.y)
//   [2] target_ground_pos.x — 目标(信标)地面X坐标 (cm)       ← S1_TARGET_X (pos.target.x)
//   [3] target_ground_pos.y — 目标(信标)地面Y坐标 (cm)       ← S1_TARGET_Y (pos.target.y)
//   [4] drone_yaw           — 无人机偏航角 (deg, 顺时针正)    ← S1_SNAPSHOT_YAW
//   [5] locked_state        — 锁定状态 (0=全丢/1=仅小车/2=仅信标/3=都有/4=近距离融合盲冲) ← S1_LOCKED_COUNT
//   [6] car_en              — 急停使能标志 (0=急停, 1=正常)   ← car_en
//   [7] car_target_dist     — 车-信标地面距离 (cm)                           ← S1_CAR_TARGET_DIST
// ******************************************************************************
void Float_Buffer_write(float* buffer, volatile float* share_data) //此处share_data一般传入share_data_from_1
{
    buffer[0] = share_data[S1_K_CAR_X];
    buffer[1] = share_data[S1_K_CAR_Y];
    buffer[2] = share_data[S1_TARGET_X];
    buffer[3] = share_data[S1_TARGET_Y];
    buffer[4] = share_data[S1_SNAPSHOT_YAW]; // [修复]: 使用快照Yaw替代实时Yaw，消灭20ms的时序旋转误差
    buffer[5] = share_data[S1_LOCKED_COUNT];
    buffer[6] = car_en;
    buffer[7] = share_data[S1_CAR_TARGET_DIST];
}

#elif defined(CY_CORE_CM7_1)
void M7_1_data_send(volatile float* data_out) { //Core 1 调用，写入share_data_from_1
    data_out[S1_CAR_CENTER_Y] = cam_down.car_center_y;
    data_out[S1_CAR_CENTER_X] = cam_down.car_center_x;
    data_out[S1_CAR_DOT_NUM]  = (float)cam_down.car_dot_num;
    data_out[S1_CAR_RAW_X]    = pos.car.x;
    data_out[S1_CAR_RAW_Y]    = pos.car.y;
    data_out[S1_TARGET_X]     = pos.target.x;
    data_out[S1_TARGET_Y]     = pos.target.y;
    data_out[S1_SNAPSHOT_YAW] = img_imu_snap.yaw; // 传回 Core0 的是该帧对应的快照 Yaw
    data_out[S1_RAW_CAR_X]    = pos.raw_car.x;
    data_out[S1_K_CAR_X]      = pos.k_car.x;
    data_out[S1_K_CAR_Y]      = pos.k_car.y;
    data_out[S1_CAR_TARGET_DIST] = dataC.car_target_dist;

#define FUSION_DIRECT_DIST_CM 60.0f
#define FUSION_DIRECT_DIST_SQ (FUSION_DIRECT_DIST_CM * FUSION_DIRECT_DIST_CM)
#define FUSION_STATE4_HOLD_FRAMES 3U
#define FUSION_STATE4_COOLDOWN_MS 1000U
#define LOCKED_STATE_MIN_HEIGHT_CM 90.0f
#define LOCKED_STATE_LOW_HEIGHT_HOLD_FRAMES 5U // 图像约50Hz，5帧约100ms

    uint8_t locked_count = 0;
    if (cam_down.car_valid) locked_count++;
    if (cam_down.target_valid) locked_count += 2;

    uint8_t raw_locked_count = 0;
    if (cam_down.car_raw_valid) raw_locked_count++;
    if (cam_down.target_raw_valid) raw_locked_count += 2;

    // --- 近距融合事件检测 ---
    static uint8_t fusion_state4_hold_frames = 0;
    static uint32_t last_fusion_trigger_ms = 0;
    static uint8_t has_last_fusion_trigger_ms = 0;
    static uint8_t low_height_frame_cnt = 0;
    uint8_t trigger_fusion = 0;
    uint32_t now_ms = dataC.pit0_cnt;
    uint8_t fusion_cooldown_elapsed = (!has_last_fusion_trigger_ms ||
        (uint32_t)(now_ms - last_fusion_trigger_ms) >= FUSION_STATE4_COOLDOWN_MS);

    if (img_imu_snap.height < LOCKED_STATE_MIN_HEIGHT_CM) {
        if (low_height_frame_cnt < LOCKED_STATE_LOW_HEIGHT_HOLD_FRAMES) {
            low_height_frame_cnt++;
        }
        if (low_height_frame_cnt >= LOCKED_STATE_LOW_HEIGHT_HOLD_FRAMES) {
            fusion_state4_hold_frames = 0;
            last_fusion_trigger_ms = 0;
            has_last_fusion_trigger_ms = 0;
            data_out[S1_LOCKED_COUNT] = 0.0f;
            if (!cam_down.car_valid) {
                data_out[S1_CAR_DOT_NUM] = 0;
            }
            return;
        }
    } else {
        low_height_frame_cnt = 0;
    }

    if (fusion_state4_hold_frames > 0) {
        trigger_fusion = 1;
        fusion_state4_hold_frames--;
    } else {
        if (raw_locked_count == 3 && fusion_cooldown_elapsed) {
            float current_rel_x = (float)(pos.raw_target.x - pos.raw_car.x);
            float current_rel_y = (float)(pos.raw_target.y - pos.raw_car.y);
            float rel_dist_sq = current_rel_x * current_rel_x + current_rel_y * current_rel_y;

            if (rel_dist_sq <= FUSION_DIRECT_DIST_SQ) {
                trigger_fusion = 1;
                fusion_state4_hold_frames = FUSION_STATE4_HOLD_FRAMES - 1U;
                last_fusion_trigger_ms = now_ms;
                has_last_fusion_trigger_ms = 1;
            }
        }
    }

    if (trigger_fusion) {
        locked_count = 4; // 触发或维持新建状态 4
    }

    data_out[S1_LOCKED_COUNT] = (float)locked_count;

    if (!cam_down.car_valid) {
        data_out[S1_CAR_DOT_NUM] = 0; // 丢失时仅将面积清零通知飞控即可
    }
}
#endif
