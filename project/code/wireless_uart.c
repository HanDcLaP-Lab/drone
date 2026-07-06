#include "fly_ctrl.h"
#include "zf_common_headfile.h"

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
    // wireless_uart_send_float(imu_data.roll);
    // wireless_uart_send_string(",");
    // wireless_uart_send_float(imu_data.pitch);
    // wireless_uart_send_string(",");
    // wireless_uart_send_float(imu_data.yaw);
    // wireless_uart_send_string(",");
    wireless_uart_send_float(motor_out.lf);
    wireless_uart_send_string(",");
    wireless_uart_send_int(tof_base_throttle);
    wireless_uart_send_string(",");
    wireless_uart_send_float(flight_target.target_roll);
    wireless_uart_send_string(",");
    wireless_uart_send_float(flight_target.target_pitch);
    wireless_uart_send_string(",");
    wireless_uart_send_float(imu_data.z);
    wireless_uart_send_string(",");
    extern uint16_t tof_cnt;
    wireless_uart_send_float(tof_cnt);tof_cnt=0;
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
    wireless_uart_send_float(share_data_from_1[S1_K_CAR_X]);
    wireless_uart_send_string(",");
    wireless_uart_send_float(share_data_from_1[S1_K_CAR_Y]);
    // wireless_uart_send_string(",");
    // wireless_uart_send_float(share_data_from_1[10]); // reserved
    //wireless_uart_send_string(",");
    //wireless_uart_send_float(share_data_from_1[S1_K_CAR_Y]);
    wireless_uart_send_string("\n");
}

void wireless_uart_output_imu_sample_rate(void){
    uint16_t gyro_count = imu_gyro_new_sample_count;
    uint16_t acc_count = imu_acc_new_sample_count;
    imu_gyro_new_sample_count = 0;
    imu_acc_new_sample_count = 0;

    wireless_uart_send_int((int32_t)gyro_count);
    wireless_uart_send_string(",");
    wireless_uart_send_int((int32_t)acc_count);
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
