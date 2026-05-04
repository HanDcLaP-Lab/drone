#include "upixel.h"

upixels_data_t upixels_data = {0};
static uart_index_enum upixels_uart_idx;
float opt_vel_x = 0.0f;
float opt_vel_y = 0.0f;
/**
 * @brief 光流模块初始化
 * @param uart_n   使用的UART模块号
 * @param tx_pin   TX引脚 (对应单片机TX)
 * @param rx_pin   RX引脚 (对应单片机RX，接光流TX)
 */
void upixels_init(void)
{
    // 初始化UART：波特率固定为 19200，1个起始位，8个数据位，1个停止位，无校验位[cite: 1, 2]
    uart_init(UPIXEL_UART, 19200, UPIXEL_UART_TX, UPIXEL_UART_RX);
    
    // 开启串口接收中断，确保数据能被 zf_driver_uart 接收
    uart_rx_interrupt(UPIXEL_UART, 1);
}

/**
 * @brief 状态机解析逐个接收到的字节
 * @param data  最新接收到的单字节
 * @return      1表示成功解析出一帧完整数据，0表示解析中或校验失败
 */
uint8 upixels_parse_byte(uint8 data)
{
    static uint8 state = 0;
    static uint8 buffer[14];
    static uint8 checksum = 0;

    // 协议包头判断：0xFE[cite: 1]
    if (state == 0 && data == 0xFE) 
    {
        buffer[0] = data;
        state = 1;
    } 
    // 字节数判断：0x0A[cite: 1]
    else if (state == 1 && data == 0x0A) 
    {
        buffer[1] = data;
        state = 2;
        checksum = 0; // 重置校验和
    } 
    // 接收光流数据结构体内容 (Byte 3 ~ Byte 12，共10个字节参与异或校验)[cite: 1]
    else if (state >= 2 && state < 12) 
    {
        buffer[state] = data;
        checksum ^= data; 
        state++;
    } 
    // 校验位比对[cite: 1]
    else if (state == 12) 
    {
        buffer[12] = data;
        if (checksum != data) 
        {
            state = 0; // 校验失败，状态机复位
            return 0;
        }
        state++;
    } 
    // 包尾判断：0x55[cite: 1]
    else if (state == 13) 
    {
        buffer[13] = data;
        state = 0; // 无论成功与否，状态机复位迎接下一帧
        if (data == 0x55) 
        {
            // 解析成功，按低字节在前、高字节在后的方式拼接数据[cite: 1]
            upixels_data.flow_x_integral      = (int16)((buffer[3] << 8) | buffer[2]);
            upixels_data.flow_y_integral      = (int16)((buffer[5] << 8) | buffer[4]);
            upixels_data.integration_timespan = (uint16)((buffer[7] << 8) | buffer[6]);
            upixels_data.ground_distance      = (uint16)((buffer[9] << 8) | buffer[8]);
            upixels_data.valid                = buffer[10];
            upixels_data.version              = buffer[11];
            return 1; 
        }
    } 
    else 
    {
        state = 0; // 异常情况复位
    }
    return 0;
}

/**
 * @brief  解算光流物理速度
 * @param  current_height_cm 当前无人机的离地高度(cm)
 */
void upixels_calc_velocity(float current_height_cm)
{
    // 数据不可用或时间为0时，跳过解算防异常
    if (upixels_data.valid != 0xF5 || upixels_data.integration_timespan == 0) {
        return;
    }
    
    // 贴地保护：防止起飞前或降落时高度为0导致速度异常发散
    if (current_height_cm < 10.0f) {
        current_height_cm = 10.0f; 
    }

    // 代入化简后的公式：V (cm/s) = (integral * H * 100) / timespan
    upixels_data.opt_vel_x = ((float)upixels_data.flow_x_integral * current_height_cm * 100.0f) / (float)upixels_data.integration_timespan;
    upixels_data.opt_vel_y = ((float)upixels_data.flow_y_integral * current_height_cm * 100.0f) / (float)upixels_data.integration_timespan;
}
/**
 * @brief 在主循环中高频调用的数据拉取与解析函数
 */
void upixels_get_data_loop(void)
{
    uint8 rx_data;
    // 不断查询是否收到新数据，如果有则送入状态机解析
    while (uart_query_byte(upixels_uart_idx, &rx_data)) 
    {
        upixels_parse_byte(rx_data);
    }
}