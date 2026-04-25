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
    data_out[0] = imu_data.roll; 
    data_out[1] = imu_data.pitch;
    data_out[2] = imu_data.yaw;

    data_out[3] = imu_data.z;
    data_out[4] = (float)current_drone_state;
    data_out[5] = motor_out.lf;
    data_out[6] = motor_out.rf;
    data_out[7] = motor_out.lb;
    data_out[8] = motor_out.rb;

    data_out[9] = flight_target.target_roll; 
    data_out[10] = flight_target.target_pitch;
    data_out[11] = flight_target.target_yaw;
    data_out[12] = dataC.debug_earth_err_x;
    data_out[13] = dataC.debug_earth_err_y;
    
}

void Float_Buffer_write(float* buffer, volatile float* share_data) //此处share_data一般传入share_data_from_1
{
    buffer[0] = share_data[9];
    buffer[1] = share_data[12];
    buffer[2] = share_data[5];
    buffer[3] = share_data[6];
    buffer[4] = imu_data.yaw;
    buffer[5] = share_data[14];
    buffer[6] = car_en;
}

#elif defined(CY_CORE_CM7_1)
void M7_1_data_send(volatile float* data_out) { //Core 1 调用，写入share_data_from_1
    data_out[0] = cam_down.car_center_y; 
    data_out[1] = cam_down.car_center_x;
    data_out[2] = (float)cam_down.car_dot_num;
    data_out[3] = pos.car.x;
    data_out[4] = pos.car.y;  
    data_out[5] = pos.target.x;
    data_out[6] = pos.target.y;
    data_out[8] = img_imu_snap.yaw; // 传回 Core0 的是该帧对应的快照 Yaw
    data_out[7] = pos.raw_car.x;
    data_out[9] = pos.k_car.x;
    data_out[12] = pos.k_car.y;
    data_out[13] = dataC.car_target_dist;

    uint8_t locked_count = 0;
    if (cam_down.car_valid) locked_count++;
    if (cam_down.target_valid) locked_count += 2;
    data_out[14] = (float)locked_count;
    
    if (!cam_down.car_valid) {
        data_out[2] = 0; // 丢失时仅将面积清零通知飞控即可
    }
}
#endif
