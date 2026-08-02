#include "data_complex.h"
#include "zf_common_headfile.h"
#include "image.h"

Data_Complex_t dataC = {0};
uint8_t car_en = 1;
uint8_t car_en_height = 0;

// 视觉帧一致性快照: Core0 主循环整帧拷贝 share_data_from_1 并复核帧序号后消费此数组。
// 所有消费者 (image_ctrl/wireless_uart/下传协议) 一律读 vision_snap，避免跨多 float 读共享区
// 时被 Core1 新帧写穿导致新旧帧混合。定义在两个核的构建中 (image_ctrl.c 同时编译进 CM7_1)，
// CM7_1 侧保持全零且无人消费。
volatile float vision_snap[M7_x_DATA_LENGTH] = {0};

//核间通信初始化
#if defined(CY_CORE_CM7_0)
    //Core 0

    // 两个20-float共享区按32字节缓存行隔开，整区Clean不会触及另一生产者的数据。
    #pragma location = 0x28001060
    volatile float share_data_from_0[M7_x_DATA_LENGTH] = {0}; // Core 0 定义并负责清零

    #pragma location = 0x28001000
    __root __no_init volatile float share_data_from_1[M7_x_DATA_LENGTH]; // 对 Core 1 的数据只读，不初始化

#elif defined(CY_CORE_CM7_1)
    //Core 1
    
    #pragma location = 0x28001000
    volatile float share_data_from_1[M7_x_DATA_LENGTH] = {0}; // Core 1 定义并负责清零
    
    #pragma location = 0x28001060
    __root __no_init volatile float share_data_from_0[M7_x_DATA_LENGTH]; // 对 Core 0 的数据只读，不初始化

#else
    #error "Unknown Core"
#endif

#if defined(CY_CORE_CM7_0)


// ================= 板间通讯协议配置 =================
#define FRAME_HEADER1 0xAA
#define FRAME_HEADER2 0x55
#define FRAME_TAIL    0x7F
#define UART_FLOAT_BYTES     4U
#define UART_PAYLOAD_BYTES   (UART_DATA_LENGTH * UART_FLOAT_BYTES)
#define UART_DATA_OFFSET     2U
#define UART_CHECKSUM_INDEX  (UART_DATA_OFFSET + UART_PAYLOAD_BYTES)
#define UART_TAIL_INDEX      (UART_CHECKSUM_INDEX + 1U)
#define UART_FRAME_LENGTH    (UART_TAIL_INDEX + 1U)

static uint8_t send_buffer[UART_FRAME_LENGTH];
// ================= 通讯初始化 =================
void Board_Comm_Init(void)
{
    // 初始化配置好的串口
    uart_init(BOARD_UART, BOARD_BAUDRATE, BOARD_TX_PIN, BOARD_RX_PIN);
}

// ================= 打包并发送函数 =================
// 只要传入长度为 UART_DATA_LENGTH 的 float 数组首地址即可
void Board_Comm_Send_Data(volatile float *data_array)
{
    // 1. 填入帧头
    send_buffer[0] = FRAME_HEADER1;
    send_buffer[1] = FRAME_HEADER2;
    
    // 2. 将传入的 float 数组拷贝到发送缓冲区
    // 先读入局部变量以保留 volatile 读取语义，再用 memcpy 处理对齐
    float local_data[UART_DATA_LENGTH];
    for (int i = 0; i < UART_DATA_LENGTH; i++) {
        local_data[i] = data_array[i];
    }
    memcpy(&send_buffer[UART_DATA_OFFSET], local_data, sizeof(local_data));
    
    // 3. 计算简单的累加校验和 (只校验float数据区)
    uint8_t checksum = 0;
    for (int i = UART_DATA_OFFSET; i < UART_CHECKSUM_INDEX; i++) {
        checksum += send_buffer[i];
    }
    
    // 4. 填入校验和与帧尾
    send_buffer[UART_CHECKSUM_INDEX] = checksum;
    send_buffer[UART_TAIL_INDEX] = FRAME_TAIL;
    
    // 5. 物理发送整包数据
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
// buffer[12] 索引映射，与小车端 uart_data[12] 一一对应:
//   [0] car_raw_x           — 小车机体系X (cm, 未滤波)       ← S1_CAR_RAW_X (pos.raw_car.x)
//   [1] car_raw_y           — 小车机体系Y (cm, 未滤波)       ← S1_CAR_RAW_Y (pos.raw_car.y)
//   [2] target_raw_x        — 主信标机体系X (cm, 未滤波)     ← S1_RAW_TARGET_X
//   [3] target_raw_y        — 主信标机体系Y (cm, 未滤波)     ← S1_RAW_TARGET_Y
//   [4] drone_yaw           — 无人机地面系偏航角 (deg, 顺时针正) ← VISION_EARTH_YAW_DEG(S1_SNAPSHOT_YAW)
//   [5] locked_state        — 锁定状态 (0=全丢/1=仅小车/2=仅信标/3=都有) ← S1_LOCKED_COUNT
//   [6] car_en              — 小车使能标志 (0=停止, 1=正常)   ← car_en && car_en_height
//   [7] car_target_dist     — 车-信标地面距离 (cm)           ← S1_CAR_TARGET_DIST
//   [8] target2_raw_x       — 第二信标机体系X (cm, 未滤波)    ← S1_RAW_TARGET2_X
//   [9] target2_raw_y       — 第二信标机体系Y (cm, 未滤波)    ← S1_RAW_TARGET2_Y
//   [10] target3_raw_x      — 第三信标机体系X (cm, 未滤波)    ← S1_RAW_TARGET3_X
//   [11] target3_raw_y      — 第三信标机体系Y (cm, 未滤波)    ← S1_RAW_TARGET3_Y
// ******************************************************************************
void Float_Buffer_write(float* buffer, volatile float* share_data) //此处share_data一般传入share_data_from_1
{
    car_en_height = (imu_data.z >= CAR_ENABLE_MIN_HEIGHT_CM);

    buffer[0] = share_data[S1_CAR_RAW_X];
    buffer[1] = share_data[S1_CAR_RAW_Y];
    buffer[2] = share_data[S1_RAW_TARGET_X];
    buffer[3] = share_data[S1_RAW_TARGET_Y];
    buffer[4] = VISION_EARTH_YAW_DEG(share_data[S1_SNAPSHOT_YAW]);
    buffer[5] = share_data[S1_LOCKED_COUNT];
    buffer[6] = (float)(car_en && car_en_height);
    buffer[7] = share_data[S1_CAR_TARGET_DIST];
    buffer[8] = share_data[S1_RAW_TARGET2_X];
    buffer[9] = share_data[S1_RAW_TARGET2_Y];
    buffer[10] = share_data[S1_RAW_TARGET3_X];
    buffer[11] = share_data[S1_RAW_TARGET3_Y];
}

#elif defined(CY_CORE_CM7_1)
void M7_1_data_send(volatile float* data_out) { //Core 1 调用，写入share_data_from_1
    static uint8_t vision_low_height_frame_cnt = 0;

    data_out[S1_CAR_CENTER_Y] = cam_down.car_center_y;
    data_out[S1_CAR_CENTER_X] = cam_down.car_center_x;
    data_out[S1_CAR_DOT_NUM]  = (float)cam_down.car_dot_num;
    data_out[S1_CAR_RAW_X]    = pos.raw_car.x;
    data_out[S1_CAR_RAW_Y]    = pos.raw_car.y;
    data_out[S1_TARGET_X]     = pos.k_target.x;
    data_out[S1_TARGET_Y]     = pos.k_target.y;
    data_out[S1_SNAPSHOT_YAW] = img_imu_snap.yaw; // 传回 Core0 的是该帧对应的快照 Yaw
    data_out[S1_RAW_TARGET_X] = pos.raw_target[0].x;
    data_out[S1_RAW_TARGET_Y] = pos.raw_target[0].y;
    data_out[S1_RAW_TARGET2_X] = pos.raw_target[1].x;
    data_out[S1_RAW_TARGET2_Y] = pos.raw_target[1].y;
    data_out[S1_RAW_TARGET3_X] = pos.raw_target[2].x;
    data_out[S1_RAW_TARGET3_Y] = pos.raw_target[2].y;
    data_out[S1_K_CAR_X]      = pos.k_car.x;
    data_out[S1_K_CAR_Y]      = pos.k_car.y;
    data_out[S1_CAR_TARGET_DIST] = dataC.car_target_dist;

    uint8_t locked_count = 0;
    if (cam_down.car_valid) locked_count++;
    if (cam_down.target_valid) locked_count += 2;
    // locked_count: 0=全丢, 1=仅小车, 2=仅信标, 3=都有

    if (img_imu_snap.height < VISION_POSITION_MIN_HEIGHT_CM) {
        if (vision_low_height_frame_cnt < VISION_LOW_HEIGHT_HOLD_FRAMES) {
            vision_low_height_frame_cnt++;
        }
        if (vision_low_height_frame_cnt >= VISION_LOW_HEIGHT_HOLD_FRAMES) {
            ground_position_history_reset();
            locked_count = 0;
            data_out[S1_CAR_DOT_NUM] = 0;
        }
    } else {
        vision_low_height_frame_cnt = 0;
    }

    data_out[S1_LOCKED_COUNT] = (float)locked_count;

    if (!cam_down.car_valid) {
        data_out[S1_CAR_DOT_NUM] = 0; // 丢失时仅将面积清零通知飞控即可
    }
}
#endif
