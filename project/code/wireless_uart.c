#include "fly_ctrl.h"
#include "zf_common_headfile.h"
#include "duplex_comm.h"   // 板间双向通讯计数器/原因码 (wireless_uart_output_duplex 使用)

uint8 data_buffer[32];
char buf[16];
uint8 data_len;
void wireless_uart_init_(){
    
    if(wireless_uart_init())                                                    // 判断是否通过初始化
    {
        while(1)                                                                // 初始化失败就在这进入死循环
        {
            system_delay_ms(100);                                               // 短延时快速闪灯表示异常
        }
    }
    wireless_uart_send_byte('\r');
    wireless_uart_send_byte('\n');
    wireless_uart_send_string("SEEKFREE wireless uart demo.\r\n");              // 初始化正常 输出测试信息
}

// void wireless_uart_get_(){
//     data_len = (uint8)wireless_uart_read_buffer(data_buffer, 32);             // 查看是否有消息 默认缓冲区是 WIRELESS_UART_BUFFER_SIZE 总共 64 字节
//         if(data_len != 0)                                                       // 收到了消息 读取函数会返回实际读取到的数据个数
//         {
//             Flight_Lock();
//             wireless_uart_send_buffer(data_buffer, data_len);                     // 将收到的消息发送回去
//             memset(data_buffer, 0, 32);
//             func_uint_to_str((char *)data_buffer, data_len);
//             wireless_uart_send_string("\r\ndata len:");                                 // 显示实际收到的数据信息
//             wireless_uart_send_buffer(data_buffer, strlen((const char *)data_buffer));    // 显示收到的数据个数
//             wireless_uart_send_string(".\r\n");
//         }
// }
void wireless_uart_send_int(int32_t send_a)
{
    snprintf(buf, sizeof(buf), "%d", send_a);
    wireless_uart_send_string(buf);
}
void wireless_uart_send_float(float send_a)
{
    snprintf(buf, sizeof(buf), "%.1f",send_a);
    wireless_uart_send_string(buf);
}

void wireless_uart_output_status(void){   //打印目标倾角和实际倾角
    wireless_uart_send_float(flight_target.target_roll);
    wireless_uart_send_string(",");
    wireless_uart_send_float(imu_data.roll);
    wireless_uart_send_string(",");
    wireless_uart_send_float(flight_target.target_pitch);
    wireless_uart_send_string(",");
    wireless_uart_send_float(imu_data.pitch);
    wireless_uart_send_string(",");
    wireless_uart_send_float(flight_target.target_g_roll);
    wireless_uart_send_string(",");
    wireless_uart_send_float(imu_data.groll);
    wireless_uart_send_string(",");
    wireless_uart_send_float(flight_target.target_g_pitch);
    wireless_uart_send_string(",");
    wireless_uart_send_float(imu_data.gpitch);
    wireless_uart_send_string("\n");
}

void wireless_uart_output_imu(void){   //打印imu数据
    wireless_uart_send_float(imu_data.roll);
    wireless_uart_send_string(",");
    wireless_uart_send_float(imu_data.pitch);
    wireless_uart_send_string(",");
    wireless_uart_send_float(imu_data.yaw);
    wireless_uart_send_string(",");
    wireless_uart_send_float(imu_data.z);
    wireless_uart_send_string("\n");
}

void wireless_uart_output_pid(void){    //打印pid数据
    wireless_uart_send_float(flight_target.target_roll);
    wireless_uart_send_string(",");
    wireless_uart_send_float(flight_target.target_pitch);
    wireless_uart_send_string(",");
    wireless_uart_send_float(flight_target.target_g_roll);
    wireless_uart_send_string(",");
    wireless_uart_send_float(flight_target.target_g_pitch);
    wireless_uart_send_string(",");
    wireless_uart_send_float(motor_out.roll);
    wireless_uart_send_string(",");
    wireless_uart_send_float(motor_out.pitch);
    wireless_uart_send_string(",");
    wireless_uart_send_float(imu_data.roll);
    wireless_uart_send_string(",");
    wireless_uart_send_float(imu_data.pitch);
    wireless_uart_send_string(",");
    wireless_uart_send_float(pid_roll.integral);
    wireless_uart_send_string(",");
    wireless_uart_send_float(pid_pitch.integral);
    wireless_uart_send_string("\n");
}

void wireless_uart_output_yaw(void){
    wireless_uart_send_float(imu_data.yaw);
    wireless_uart_send_string(",");
    wireless_uart_send_float(flight_target.target_yaw);
    wireless_uart_send_string(",");
    wireless_uart_send_float(imu_data.gyaw);
    wireless_uart_send_string(",");
    wireless_uart_send_float(flight_target.target_g_yaw);
    wireless_uart_send_string(",");
    wireless_uart_send_float(motor_out.yaw);
    wireless_uart_send_string("\n");
}

void wireless_uart_output_motor(void){
    wireless_uart_send_float(motor_out.lf);
    wireless_uart_send_string(",");
    wireless_uart_send_float(motor_out.rf);
    wireless_uart_send_string(",");
    wireless_uart_send_float(motor_out.lb);
    wireless_uart_send_string(",");
    wireless_uart_send_float(motor_out.rb);
    wireless_uart_send_string(",");
    wireless_uart_send_float(imu_data.vz);
    wireless_uart_send_string(",");
    wireless_uart_send_float(imu_data.z);
    //wireless_uart_send_string(",");
    //wireless_uart_send_int((int32_t)notch_active_count);
    wireless_uart_send_string("\n");
}

void wireless_uart_output_groud(void){
    wireless_uart_send_float(share_data_from_1[S1_RAW_CAR_X]);
    wireless_uart_send_string(",");
    wireless_uart_send_float(share_data_from_1[S1_K_CAR_X]);
    // wireless_uart_send_string(",");
    // wireless_uart_send_float(share_data_from_1[10]); // reserved
    //wireless_uart_send_string(",");
    //wireless_uart_send_float(share_data_from_1[S1_K_CAR_Y]);
    wireless_uart_send_string("\n");
}

void wireless_uart_output_height(void){
    wireless_uart_send_float(imu_data.z);
    wireless_uart_send_string(",");
    wireless_uart_send_float(imu_data.vz);
    wireless_uart_send_string(",");
    wireless_uart_send_float(imu_data.z);
    wireless_uart_send_string(",");
    wireless_uart_send_float(imu_data.z);
    wireless_uart_send_string(",");
    wireless_uart_send_float(z_rate);
    wireless_uart_send_string(",");
    wireless_uart_send_float(z_acc);
    wireless_uart_send_string("\n");
}

// 板间双向通讯收发统计调试输出 (格式同 output_coast: 纯数字 + 逗号, 结尾换行)
// 输出: <TX请求>,<成功>,<超时>,<解码失败>,<命令字不匹配>,<FIFO溢出>\n
//   TX请求       = duplex_request_count   (已发起并完成发送的请求帧数)
//   成功         = duplex_reply_ok_count  (成功收到小车应答帧)
//   超时         = duplex_timeout_count   (超时未收到应答, 主要丢包来源)
//   解码失败     = duplex_decode_fail_count (帧头/帧尾/校验损坏)
//   命令字不匹配 = duplex_cmd_mismatch_count (收到帧但 cmd 非 CMD_SLAVE)
//   FIFO溢出     = duplex_rx_fifo_drop_count (接收缓冲写满丢字节)
void wireless_uart_output_duplex(void){
    wireless_uart_send_int((int32_t)duplex_request_count);
    wireless_uart_send_string(",");
    wireless_uart_send_int((int32_t)duplex_reply_ok_count);
    wireless_uart_send_string(",");
    wireless_uart_send_int((int32_t)duplex_timeout_count);
    wireless_uart_send_string(",");
    wireless_uart_send_int((int32_t)duplex_decode_fail_count);
    wireless_uart_send_string(",");
    wireless_uart_send_int((int32_t)duplex_cmd_mismatch_count);
    wireless_uart_send_string(",");
    wireless_uart_send_int((int32_t)duplex_rx_fifo_drop_count);
    wireless_uart_send_string("\n");
}