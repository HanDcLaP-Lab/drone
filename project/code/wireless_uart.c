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

// ================== 四电机500ms平均值调试 ==================
#define MOTOR_AVG_WINDOW_MS 500U

static uint32_t motor_avg_sum_lf = 0;
static uint32_t motor_avg_sum_rf = 0;
static uint32_t motor_avg_sum_lb = 0;
static uint32_t motor_avg_sum_rb = 0;
static uint16_t motor_avg_sample_count = 0;
static uint32_t motor_avg_window_start_ms = 0;
static uint8_t motor_avg_window_initialized = 0;

static volatile uint32_t motor_avg_snapshot_lf = 0;
static volatile uint32_t motor_avg_snapshot_rf = 0;
static volatile uint32_t motor_avg_snapshot_lb = 0;
static volatile uint32_t motor_avg_snapshot_rb = 0;
static volatile uint16_t motor_avg_snapshot_count = 0;
static volatile uint8_t motor_avg_print_pending = 0;

void wireless_uart_motor_average_sample(void){
    uint32_t now_ms = dataC.pit0_cnt;

    if (!motor_avg_window_initialized) {
        motor_avg_window_start_ms = now_ms;
        motor_avg_window_initialized = 1;
    } else if ((uint32_t)(now_ms - motor_avg_window_start_ms) >= MOTOR_AVG_WINDOW_MS) {
        if (motor_avg_sample_count > 0) {
            motor_avg_snapshot_lf = motor_avg_sum_lf;
            motor_avg_snapshot_rf = motor_avg_sum_rf;
            motor_avg_snapshot_lb = motor_avg_sum_lb;
            motor_avg_snapshot_rb = motor_avg_sum_rb;
            motor_avg_snapshot_count = motor_avg_sample_count;
            motor_avg_print_pending = 1;
        }
        motor_avg_sum_lf = 0;
        motor_avg_sum_rf = 0;
        motor_avg_sum_lb = 0;
        motor_avg_sum_rb = 0;
        motor_avg_sample_count = 0;
        motor_avg_window_start_ms = now_ms;
    }

    // DEBUG模式实际下发为0；其余状态采集限幅后的最终电机输出。
    if (current_drone_state != DRONE_STATE_DEBUG) {
        motor_avg_sum_lf += (uint32_t)motor_out.lf;
        motor_avg_sum_rf += (uint32_t)motor_out.rf;
        motor_avg_sum_lb += (uint32_t)motor_out.lb;
        motor_avg_sum_rb += (uint32_t)motor_out.rb;
    }
    motor_avg_sample_count++;
}

void wireless_uart_output_motor_average(void){
    uint32_t sum_lf;
    uint32_t sum_rf;
    uint32_t sum_lb;
    uint32_t sum_rb;
    uint16_t sample_count;
    uint32_t primask;
    float divisor;

    if (!motor_avg_print_pending) return;

    primask = interrupt_global_disable();
    sum_lf = motor_avg_snapshot_lf;
    sum_rf = motor_avg_snapshot_rf;
    sum_lb = motor_avg_snapshot_lb;
    sum_rb = motor_avg_snapshot_rb;
    sample_count = motor_avg_snapshot_count;
    motor_avg_print_pending = 0;
    interrupt_global_enable(primask);

    if (sample_count == 0) return;

    divisor = (float)sample_count;
    wireless_uart_send_float((float)sum_lf / divisor);
    wireless_uart_send_string(",");
    wireless_uart_send_float((float)sum_rf / divisor);
    wireless_uart_send_string(",");
    wireless_uart_send_float((float)sum_lb / divisor);
    wireless_uart_send_string(",");
    wireless_uart_send_float((float)sum_rb / divisor);
    wireless_uart_send_string("\r\n");
}

void wireless_uart_output_groud(void){
    wireless_uart_send_float(vision_snap[S1_K_CAR_X]);
    wireless_uart_send_string(",");
    wireless_uart_send_float(vision_snap[S1_K_CAR_Y]);
    // wireless_uart_send_string(",");
    // wireless_uart_send_float(vision_snap[10]); // reserved
    //wireless_uart_send_string(",");
    //wireless_uart_send_float(vision_snap[S1_K_CAR_Y]);
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

#define BEACON_BRIGHTNESS_PRINT_MS 500U

void wireless_uart_output_beacon_brightness(void){
    static float peak_sum = 0.0f;
    static float brightest9_sum = 0.0f;
    static float peak_x_sum = 0.0f;
    static float peak_y_sum = 0.0f;
    static float peak_threshold_sum = 0.0f;
    static float raw_area_sum = 0.0f;
    static float distance_sum = 0.0f;
    static float peak_rho_min = 0.0f;
    static float peak_rho_max = 0.0f;
    static uint16_t sample_count = 0;
    static uint16_t raw_pass_count = 0;
    static uint16_t target_valid_count = 0;
    static uint32_t window_start_ms = 0;

    float peak_x = vision_snap[S1_BRIGHTEST_X];
    float peak_y = vision_snap[S1_BRIGHTEST_Y];
    float dx = peak_x - (float)CAM_CX;
    float dy = peak_y - (float)CAM_CY;
    float peak_rho = sqrtf(dx * dx + dy * dy);

    if (sample_count == 0) {
        window_start_ms = dataC.pit0_cnt;
        peak_rho_min = peak_rho;
        peak_rho_max = peak_rho;
    } else {
        if (peak_rho < peak_rho_min) peak_rho_min = peak_rho;
        if (peak_rho > peak_rho_max) peak_rho_max = peak_rho;
    }

    peak_sum += vision_snap[S1_BRIGHTEST_GRAY];
    brightest9_sum += vision_snap[S1_BRIGHTEST9_MEAN];
    peak_x_sum += peak_x;
    peak_y_sum += peak_y;
    peak_threshold_sum += vision_snap[S1_BRIGHTEST_THRESH];
    raw_area_sum += vision_snap[S1_RAW_THRESH_AREA];
    distance_sum += vision_snap[S1_BRIGHTEST_DIST];
    if (vision_snap[S1_RAW_THRESH_AREA] > 0.0f) raw_pass_count++;
    if (((uint8_t)vision_snap[S1_LOCKED_COUNT] & 2U) != 0U) target_valid_count++;
    sample_count++;

    if ((uint32_t)(dataC.pit0_cnt - window_start_ms) < BEACON_BRIGHTNESS_PRINT_MS) return;

    float divisor = (float)sample_count;
    // peak,top9,x,y,rho_span,threshold,raw_area,raw_pass%,target_valid%,distance
    wireless_uart_send_float(peak_sum / divisor);
    wireless_uart_send_string(",");
    wireless_uart_send_float(brightest9_sum / divisor);
    wireless_uart_send_string(",");
    wireless_uart_send_float(peak_x_sum / divisor);
    wireless_uart_send_string(",");
    wireless_uart_send_float(peak_y_sum / divisor);
    wireless_uart_send_string(",");
    wireless_uart_send_float(peak_rho_max - peak_rho_min);
    wireless_uart_send_string(",");
    wireless_uart_send_float(peak_threshold_sum / divisor);
    wireless_uart_send_string(",");
    wireless_uart_send_float(raw_area_sum / divisor);
    wireless_uart_send_string(",");
    wireless_uart_send_float((float)raw_pass_count * 100.0f / divisor);
    wireless_uart_send_string(",");
    wireless_uart_send_float((float)target_valid_count * 100.0f / divisor);
    wireless_uart_send_string(",");
    wireless_uart_send_float(distance_sum / divisor);
    wireless_uart_send_string("\r\n");

    peak_sum = 0.0f;
    brightest9_sum = 0.0f;
    peak_x_sum = 0.0f;
    peak_y_sum = 0.0f;
    peak_threshold_sum = 0.0f;
    raw_area_sum = 0.0f;
    distance_sum = 0.0f;
    sample_count = 0;
    raw_pass_count = 0;
    target_valid_count = 0;
}
