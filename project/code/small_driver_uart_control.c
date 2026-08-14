#include "small_driver_uart_control.h"

small_device_value_struct motor_value;      // 定义通讯参数结构体


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 字节协议接收解析 (转速帧)
// 参数说明     receive_data        接收到的单字节
// 返回参数     void
// 使用示例     small_driver_byte_receive(receive_data);
// 备注信息     解析 0xA5 帧头 + 11 字节 (功能字 0x02 转速帧)
//-------------------------------------------------------------------------------------------------------------------
static void small_driver_byte_receive(uint8 receive_data)
{
    if(receive_data == 0xA5 && motor_value.receive_data_buffer[0] != 0xA5)                  // 收到帧头且当前不在帧内 重新开始接收
    {
        motor_value.receive_data_count = 0;
    }

    motor_value.receive_data_buffer[motor_value.receive_data_count ++] = receive_data;      // 保存串口数据

    if(motor_value.receive_data_count >= 11)                                                // 判断是否收到指定数量的数据
    {
        if(motor_value.receive_data_buffer[0] == 0xA5)                                      // 判断帧头是否正确
        {
            motor_value.sum_check_data = 0;                                                 // 清除校验位数据

            for(int i = 0; i < 10; i ++)
            {
                motor_value.sum_check_data += motor_value.receive_data_buffer[i];           // 重新计算校验位
            }

            if(motor_value.sum_check_data == motor_value.receive_data_buffer[10])           // 校验数据准确性
            {
                for(int i = 0; i < 4; i ++)
                {
                    motor_value.receive_speed_data[i] = (((int)motor_value.receive_data_buffer[i * 2 + 2] << 8) | (int)motor_value.receive_data_buffer[i * 2 + 3]);
                }
                motor_value.speed_data_updated = 1;                                         // 通知: 新转速数据就绪
            }
        }

        motor_value.receive_data_count = 0;                                                 // 清除缓冲区计数值
        memset(motor_value.receive_data_buffer, 0, 11);                                     // 清除缓冲区数据
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 字符串协议行解析 (提取电压/电流/温度)
// 参数说明     str                 一行已结尾的字符串
// 返回参数     void
// 使用示例     small_driver_string_parse((char *)motor_value.receive_string_buffer);
// 备注信息     行格式 "voltage:12.60 V, bus current:1.234 A, board temprature:25.678 ℃" (驱动端拼写 temprature)
//-------------------------------------------------------------------------------------------------------------------
static void small_driver_string_parse(char *str)
{
    char *pos;

    pos = strstr(str, "voltage:");                                                          // 定位电压字段
    if(pos != NULL)
    {
        motor_value.battery_voltage = (float)atof(strchr(pos, ':') + 1);                   // 冒号后为电压浮点数
        motor_value.voltage_data_updated = 1;                                               // 通知: 新电压电流温度就绪
    }

    pos = strstr(str, "bus current:");                                                      // 定位电流字段
    if(pos != NULL)
    {
        motor_value.bus_current = (float)atof(strchr(pos, ':') + 1);                       // 冒号后为电流浮点数
    }

    pos = strstr(str, "board temprature:");                                                 // 定位温度字段 (注意驱动端拼写 temprature)
    if(pos != NULL)
    {
        motor_value.board_temperature = (float)atof(strchr(pos, ':') + 1);                  // 冒号后为温度浮点数
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 字符串协议接收解析 (按行累积)
// 参数说明     receive_data        接收到的单字节
// 返回参数     void
// 使用示例     small_driver_string_receive(receive_data);
// 备注信息     收到 '\n' 判定一行结束并解析
//-------------------------------------------------------------------------------------------------------------------
static void small_driver_string_receive(uint8 receive_data)
{
    if(receive_data == '\r')                                                                // 忽略回车符
    {
        return;
    }

    if(receive_data == '\n')                                                                // 换行符表示一行结束
    {
        motor_value.receive_string_buffer[motor_value.receive_string_count] = 0;            // 添加字符串结束符
        small_driver_string_parse((char *)motor_value.receive_string_buffer);               // 解析该行
        motor_value.receive_string_count = 0;                                               // 清除接收计数
        return;
    }

    if(motor_value.receive_string_count < SMALL_DRIVER_STRING_BUFFER_SIZE - 1)              // 防止缓冲区溢出
    {
        motor_value.receive_string_buffer[motor_value.receive_string_count ++] = receive_data;
    }
    else
    {
        motor_value.receive_string_count = 0;                                               // 溢出丢弃 重新开始
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 串口接收回调函数 (字节协议 转速 + 字符串协议 电压/电流/温度)
// 参数说明     void
// 返回参数     void
// 使用示例     uart_control_callback();
// 备注信息     该函数需要在对应的串口接收中断中调用
//-------------------------------------------------------------------------------------------------------------------
void uart_control_callback(void)
{
    uint8 receive_data;                                                                     // 定义临时变量

    if(uart_query_byte(SMALL_DRIVER_UART, &receive_data))                                   // 接收串口数据
    {
        if(receive_data == 0xA5)                                                            // 0xA5 为字节协议帧头 复位字符串行并进入字节帧解析
        {
            motor_value.receive_string_count = 0;                                           // 丢弃未完成的字符串行
            small_driver_byte_receive(receive_data);                                        // 字节协议解析 (转速)
        }
        else if(motor_value.receive_data_count > 0)                                         // 字节帧进行中 继续累积 (帧内数据可为任意值)
        {
            small_driver_byte_receive(receive_data);                                        // 字节协议解析 (转速)
        }
        else                                                                                // 无字节帧进行中 按字符串行累积
        {
            small_driver_string_receive(receive_data);                                      // 字符串协议解析 (电压/电流/温度)
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 设置电机占空比
// 参数说明     left_duty       左侧电机占空比  范围 -10000 ~ 10000  负数为反转
// 参数说明     right_duty      右侧电机占空比  范围 -10000 ~ 10000  负数为反转
// 返回参数     void
// 使用示例     small_driver_set_duty(1000, -1000, 1000, -1000);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_set_duty(int16 motor_duty_1, int16 motor_duty_2, int16 motor_duty_3, int16 motor_duty_4)
{
    motor_value.send_data_buffer[0] = 0xA5;                                         // 配置帧头

    motor_value.send_data_buffer[1] = 0X01;                                         // 配置功能字

    motor_value.send_data_buffer[2] = (uint8)((motor_duty_1 & 0xFF00) >> 8);        // 拆分 电机1占空比 的高八位

    motor_value.send_data_buffer[3] = (uint8)(motor_duty_1 & 0x00FF);               // 拆分 电机1占空比 的低八位

    motor_value.send_data_buffer[4] = (uint8)((motor_duty_2 & 0xFF00) >> 8);        // 拆分 电机2占空比 的高八位

    motor_value.send_data_buffer[5] = (uint8)(motor_duty_2 & 0x00FF);               // 拆分 电机2占空比 的低八位

    motor_value.send_data_buffer[6] = (uint8)((motor_duty_3 & 0xFF00) >> 8);        // 拆分 电机3占空比 的高八位

    motor_value.send_data_buffer[7] = (uint8)(motor_duty_3 & 0x00FF);               // 拆分 电机3占空比 的低八位

    motor_value.send_data_buffer[8] = (uint8)((motor_duty_4 & 0xFF00) >> 8);        // 拆分 电机4占空比 的高八位

    motor_value.send_data_buffer[9] = (uint8)(motor_duty_4 & 0x00FF);               // 拆分 电机4占空比 的低八位

    motor_value.send_data_buffer[10] = 0;                                           // 和校验清除

    for(int i = 0; i < 10; i ++)
    {
        motor_value.send_data_buffer[10] += motor_value.send_data_buffer[i];        // 计算校验位
    }

    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 11);                     // 发送设置占空比的 字节包 数据
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 获取速度信息
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_get_speed();
// 备注信息     仅需发送一次 驱动将周期发出速度信息(默认10ms) 字节协议
//-------------------------------------------------------------------------------------------------------------------
void small_driver_get_speed(void)
{
    motor_value.send_data_buffer[0] = 0xA5;                                         // 配置帧头

    motor_value.send_data_buffer[1] = 0X02;                                         // 配置功能字

    motor_value.send_data_buffer[2] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[3] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[4] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[5] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[6] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[7] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[8] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[9] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[10] = 0xA7;                                        // 配置校验位

    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 11);         // 发送获取转速数据的 字节包 数据
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 获取电压电流温度信息
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_get_voltage();
// 备注信息     仅需发送一次 驱动将周期发出电压电流温度信息(默认10ms) 字符串协议
//-------------------------------------------------------------------------------------------------------------------
void small_driver_get_voltage(void)
{
    uart_write_string(SMALL_DRIVER_UART, SMALL_DRIVER_CMD_GET_VOLTAGE);             // 发送获取电压电流温度指令
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 参数初始化
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_init(void)
{
    memset(motor_value.send_data_buffer, 0, 11);                             // 清除缓冲区数据

    memset(motor_value.receive_data_buffer, 0, 11);                          // 清除缓冲区数据

    memset(motor_value.receive_speed_data, 0, 8);                            // 清除接收数据

    memset(motor_value.receive_string_buffer, 0, SMALL_DRIVER_STRING_BUFFER_SIZE);   // 清除字符串行缓冲

    motor_value.receive_data_count          = 0;

    motor_value.sum_check_data              = 0;

    motor_value.speed_data_updated          = 0;

    motor_value.receive_string_count        = 0;

    motor_value.battery_voltage             = 0.0f;

    motor_value.bus_current                 = 0.0f;

    motor_value.board_temperature           = 0.0f;

    motor_value.voltage_data_updated        = 0;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 串口通讯初始化
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_uart_init();
// 备注信息     根据 SMALL_DRIVER_GET_MODE 宏选择获取转速或电压电流温度
//-------------------------------------------------------------------------------------------------------------------
void small_driver_uart_init(void)
{
    uart_init(SMALL_DRIVER_UART, SMALL_DRIVER_BAUDRATE, SMALL_DRIVER_RX, SMALL_DRIVER_TX);      // 串口初始化

    uart_rx_interrupt(SMALL_DRIVER_UART, 1);                                                    // 使能串口接收中断

    small_driver_init();                                                                        // 结构体参数初始化

    small_driver_set_duty(0, 0, 0, 0);                                                          // 设置0占空比

#if SMALL_DRIVER_GET_MODE == SMALL_DRIVER_GET_SPEED
    small_driver_get_speed();                                                                   // 获取实时速度数据 (字节协议)
#elif SMALL_DRIVER_GET_MODE == SMALL_DRIVER_GET_VOLTAGE
    small_driver_get_voltage();                                                                 // 获取电压电流温度 (字符串协议)
#endif
}

void driver_uart_test(void)
{
    motor_value.send_data_buffer[0] = 0xA5;                                         // 发送帧头

    motor_value.send_data_buffer[1] = 0X03;                                         // 设置功能码

    motor_value.send_data_buffer[2] = 0x00;                                         // 数据位全零

    motor_value.send_data_buffer[3] = 0x00;                                         // 数据位全零

    motor_value.send_data_buffer[4] = 0x00;                                         // 数据位全零

    motor_value.send_data_buffer[5] = 0x00;                                         // 数据位全零

    motor_value.send_data_buffer[6] = 0x00;                                         // 数据位全零

    motor_value.send_data_buffer[7] = 0x00;                                         // 数据位全零

    motor_value.send_data_buffer[8] = 0x00;                                         // 数据位全零

    motor_value.send_data_buffer[9] = 0x00;                                         // 数据位全零

    motor_value.send_data_buffer[10] = 0xA8;                                        // 设置校验位

    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 11);         // 发送设置占空比到 字节区 函数
}
