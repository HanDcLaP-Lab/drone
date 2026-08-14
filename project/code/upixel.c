#include "upixel.h"

upixels_data_t upixels_data = {0};
volatile uint8_t upixels_frame_ready = 0; // 完整帧解析就绪标志
volatile uint16_t upixels_frame_count = 0; // 累计接收帧数
volatile uint16_t upixels_count_500ms = 0; // 过去 500ms 内的更新次数
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
    // 门限保护：状态不可用 (valid==0) 或 高度低于 80cm 时，主动归零并重置滤波器
    if (upixels_data.valid == 0 || current_height_cm < OPT_MIN_VALID_HEIGHT_CM) {
        upixels_data.opt_vel_x = 0.0f;
        upixels_data.opt_vel_y = 0.0f;
        upixels_data.filt_vel_x = 0.0f;
        upixels_data.filt_vel_y = 0.0f;
        return;
    }
    
    // 积分时间为 0 时跳过解算防除零
    if (upixels_data.integration_timespan == 0) {
        return;
    }

    float dt_us = (float)upixels_data.integration_timespan;

    // 1. 姿态角倾斜修正 (Tilt Compensation): 修正镜头法向对地实际斜距
#if OPT_TILT_COMP_ENABLE
    float roll_rad = imu_data.roll * (3.14159265f / 180.0f);
    float pitch_rad = imu_data.pitch * (3.14159265f / 180.0f);
    float cos_tilt = cosf(roll_rad) * cosf(pitch_rad);
    if (cos_tilt < 0.707f) {
        cos_tilt = 0.707f; // 倾角 > 45° 时截断保护
    }
    float eff_height_cm = current_height_cm / cos_tilt;
#else
    float eff_height_cm = current_height_cm;
#endif

    // 2. 轴向映射与陀螺仪角速度解耦补偿 (Gyro Compensation):
    //    消除机体旋转造成的伪光流位移，并将光流轴系精准映射到机体轴系 (NED: X+前, Y+右)
    //    物理几何映射关系:
    //      - 光流 Y+ 朝后 (机体 X-): 前后平移由 -flow_y 测量; 机头抬起(gpitch>0)使视野前移误测出正向假位移，需加上 gpitch 补偿
    //      - 光流 X+ 朝右 (机体 Y+): 左右平移由 flow_x 测量; 左侧抬起(groll>0)使视野左移误测出正向假位移，需减去 groll 补偿
    //    角度单位转换: (deg/s * pi / 180) * (dt_us * 1e-6) * 10000 = deg/s * dt_us * (pi / 18000)
    #define GYRO_TO_FLOW_FACTOR (3.14159265f / 18000.0f)

#if OPT_GYRO_COMP_ENABLE
    // 旋转在机体各轴产生的假角位移量
    float rot_body_x = -OPT_GYRO_SIGN_PITCH * imu_data.gpitch * dt_us * GYRO_TO_FLOW_FACTOR;
    float rot_body_y =  OPT_GYRO_SIGN_ROLL  * imu_data.groll  * dt_us * GYRO_TO_FLOW_FACTOR;

    // 净平移角位移 (已扣除旋转干扰，并转换为机体系: X+前, Y+右)
    // trans_body_x = -flow_y - (-gpitch * dt * scale) = -flow_y + gpitch * dt * scale
    // trans_body_y =  flow_x - (groll * dt * scale)   =  flow_x - groll * dt * scale
    float trans_body_x = OPT_SIGN_BODY_X * (-(float)upixels_data.flow_y_integral - rot_body_x);
    float trans_body_y = OPT_SIGN_BODY_Y * ((float)upixels_data.flow_x_integral - rot_body_y);
#else
    float trans_body_x = OPT_SIGN_BODY_X * (-(float)upixels_data.flow_y_integral);
    float trans_body_y = OPT_SIGN_BODY_Y * (float)upixels_data.flow_x_integral;
#endif

    // 3. 计算机体系物理平移速度 (cm/s, opt_vel_x=机体前向速度, opt_vel_y=机体右向速度):
    //    V (cm/s) = (trans_body / 10000) * eff_height_cm / (dt_us / 1000000)
    //             = (trans_body * eff_height_cm * 100) / dt_us
    upixels_data.opt_vel_x = (trans_body_x * eff_height_cm * 100.0f) / dt_us;
    upixels_data.opt_vel_y = (trans_body_y * eff_height_cm * 100.0f) / dt_us;

    // 4. 一阶低通滤波去毛刺 (alpha 越小滤波越强，0.3 约等效 5Hz 截止 @100Hz 采样)
    #define FLOW_LPF_ALPHA  0.3f
    upixels_data.filt_vel_x += FLOW_LPF_ALPHA * (upixels_data.opt_vel_x - upixels_data.filt_vel_x);
    upixels_data.filt_vel_y += FLOW_LPF_ALPHA * (upixels_data.opt_vel_y - upixels_data.filt_vel_y);
}
/**
 * @brief 主循环中调用的光流速度计算消费函数
 * @param current_height_cm 当前高度
 */
void upixels_poll_and_calc(float current_height_cm)
{
    // 当中断完成一整帧的流式解析后置位，主循环消费并解算速度
    if (upixels_frame_ready)
    {
        upixels_frame_ready = 0; // 清除新帧标志位
        upixels_calc_velocity(current_height_cm);
    }
}

/**
 * @brief 在主循环中高频调用的数据拉取与解析函数
 */
void upixels_get_data_loop(void)
{
    uint8 rx_data;
    // 不断查询是否收到新数据，如果有则送入状态机解析
    while (uart_query_byte(UPIXEL_UART, &rx_data)) 
    {
        upixels_parse_byte(rx_data);
    }
}