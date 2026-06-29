/*********************************************************************************************************************
 * CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
 * Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 CYT4BB 开源库的一部分
 *
 * CYT4BB 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
 * 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅<https://www.gnu.org/licenses/>
 *
 * 额外注明：
 * 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
 * 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
 * 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
 * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
 *
 * 文件名称          cm7_0_isr
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          IAR 9.40.1
 * 适用平台          CYT4BB
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2024-1-9      pudding            first version
 * 2024-5-14     pudding            新增12个pit周期中断 增加部分注释说明
* 2025-2-4      pudding            优化串口中断逻辑，防止意外干扰导致的卡死问题，优化串口波特率计算逻辑
* 2025-2-4      pudding            新增两个串口接口
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "debug_data.h"
#include "duplex_comm.h"               // 双向板间通讯主机接口 (Duplex_Comm_On_Uart_Rx)
                                       // 注: DUPLEX_SWITCH 宏定义于 data_complex.h, 已由 zf_common_headfile.h 包含

uint16_t target = 0;
uint16_t has_stopped = 0;
// **************************** PIT中断函数 (1ms一次) ****************************
void pit0_ch0_isr() {
    pit_isr_flag_clear(PIT_CH0);
    dataC.pit0_cnt++;
    // [临时] 关闭 TOF 数据更新 (联调板间通讯期间暂停 TOF)
    // tof_update();
    IMU_Update_Loop();

    Flight_Control_Loop(); 
    if(fabsf(imu_data.pitch) > 21.0f || fabsf(imu_data.roll) > 21.0f){
        if(has_stopped == 0){
            wireless_uart_send_string("emergency stop\r\n");
            car_en = 0;
        }
        Flight_Lock();
        has_stopped = 1;
    }
    
    motor_pwm_set();
    debug_data_get();

}

void pit0_ch1_isr() 
{
    pit_isr_flag_clear(PIT_CH1);

    
    // 悬停控制任务
    
    //Flight_Hover_Control_Task(); 
    
}

void pit0_ch2_isr()  // 定时器通道 2 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH2);
    wireless_uart_output_duplex();

    // wireless_uart_send_float(imu_data.yaw);
    // wireless_uart_send_string(",");
    // wireless_uart_send_float(flight_target.target_yaw);
    // wireless_uart_send_string(",");
    // wireless_uart_send_float(imu_data.gyaw);
    // wireless_uart_send_string(",");
    // wireless_uart_send_float(flight_target.target_g_yaw);
    //wireless_uart_send_float(pid_g_roll.integral);
    // wireless_uart_send_string(",");
    // wireless_uart_send_string("\n"wireless_uart_output_，ptor();
    //wireless_uart_output_motor();
    //printf("%d",notch_active_count);
    //wireless_uart_output_imu();
    //wireless_uart_output_groud();
    //wireless_uart_output_yaw();
    // extern float share_data_from_1[];
    // extern float car_pos_sol1_0,car_pos_sol1_1;
    //printf("%.2f,%.2f,%.2f\n", imu_data.z , z_temp , tof_z);
    //wireless_uart_output_height();
}

void pit0_ch10_isr()  // 定时器通道 10 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH10);
    //debug_key1_test();
    //app_state_machine_update();
    //display_motor_output_display();
    //display_image_display();
}

void pit0_ch11_isr()  // 定时器通道 11 周期中断服务函数
{
    //display_motor_output_display();
    pit_isr_flag_clear(PIT_CH11);
}

void pit0_ch12_isr()  // 定时器通道 12 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH12);
}

void pit0_ch13_isr()  // 定时器通道 13 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH13);
}

void pit0_ch14_isr()  // 定时器通道 14 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH14);
}

void pit0_ch15_isr()  // 定时器通道 15 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH15);
}

void pit0_ch16_isr()  // 定时器通道 16 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH16);
}

void pit0_ch17_isr()  // 定时器通道 17 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH17);
}

void pit0_ch18_isr()  // 定时器通道 18 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH18);
}

void pit0_ch19_isr()  // 定时器通道 19 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH19);
}

void pit0_ch20_isr()  // 定时器通道 20 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH20);
}

void pit0_ch21_isr()  // 定时器通道 21 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH21);
    tsl1401_collect_pit_handler();
}
// **************************** PIT中断函数 ****************************


// **************************** 串口中断函数 ****************************
// 串口0默认作为调试串口
void uart0_isr (void)
{
    if(uart_isr_mask(UART_0))            // 串口0接收中断
    {
        
#if DEBUG_UART_USE_INTERRUPT             // 如果开启 debug 串口中断
        debug_interrupr_handler();       // 调用 debug 串口接收处理函数 数据会被 debug 环形缓冲区读取
#endif                                   // 如果修改了 DEBUG_UART_INDEX 那这段代码需要放到对应的串口中断去
      
    }
    else                                 // 串口0发送中断
    {           
        
        
        
    }
}

void uart1_isr (void)
{
    if(uart_isr_mask(UART_1))            // 串口1接收中断
    {
        
        wireless_module_uart_handler();  // 无线模块统一回调函数
      
    }
    else                                // 串口1发送中断
    {
      
        
        
    }
}

void uart2_isr (void)
{
    if(uart_isr_mask(UART_2))            // 串口2接收中断
    {
        
        gnss_uart_callback();            // GPS模块回调函数      
        
    }
    else                                // 串口2发送中断
    {
        
        
       
    }
}

// 在 cm7_0_isr.c 中
void uart3_isr (void)
{
    if(uart_isr_mask(UART_3))            // 串口3接收中断
    {
    }
    else                                
    {
    }
}

void uart4_isr (void)
{
    if(uart_isr_mask(UART_4))            // 串口4接收中断
    {
        // ============================================================================
        // 板间双向通讯接入点 (DUPLEX_SWITCH 宏隔离)
        //   DUPLEX_SWITCH = 1 (双向模式): UART4 RX 被 duplex_comm 用于接收小车 CMD_SLAVE
        //                                 应答帧, 收到的字节写入 duplex 接收 FIFO,
        //                                 由主循环 Duplex_Comm_Poll() 解析。
        //   DUPLEX_SWITCH = 0 (单向模式): 维持原行为, 调用接收机回调 uart_receiver_handler()。
        // 注意(需硬件确认): 双向模式下 UART4 由 duplex_comm 独占用于接收小车应答帧;
        //   若 UART4 同时被遥控接收机占用, 则两者会冲突, 此点需由用户确认硬件接线。
        //   原 uart_receiver_handler() 仅在单向模式 (DUPLEX_SWITCH=0) 下生效。
        // ============================================================================
#if DUPLEX_SWITCH
        Duplex_Comm_On_Uart_Rx();        // 双向: 接收小车应答帧入 duplex FIFO
#else
        uart_receiver_handler();         // 单向(原行为): 串口接收机回调函数
#endif
    }
    else                                // 串口4发送中断
    {
      
        
        
    }
}

void uart5_isr (void)
{
    if(uart_isr_mask(UART_5))            // 串口5接收中断
    {
        
        
       
    }
    else                                // 串口5发送中断
    {
      
        
        
    }
}

void uart6_isr (void)
{
    if(uart_isr_mask(UART_6))            // 串口6接收中断
    {
        uart_control_callback();//无刷驱动回调函数
        
       
    }
    else                                // 串口6发送中断
    {
      
        
        
    }
}
// **************************** 串口中断函数 ****************************

// **************************** 外部中断函数 ****************************
void gpio_0_exti_isr()                  // 外部 GPIO_0 中断服务函数     
{
    
  
  
}

void gpio_1_exti_isr()                  // 外部 GPIO_1 中断服务函数     
{
    if(exti_flag_get(P01_0))		// 示例P1_0端口外部中断判断
    {

      
      
            
    }
    if(exti_flag_get(P01_1))
    {

            
            
    }
}

void gpio_2_exti_isr()                  // 外部 GPIO_2 中断服务函数
{
    if(exti_flag_get(P02_0))
    {


    }


}

void gpio_3_exti_isr()                  // 外部 GPIO_3 中断服务函数     
{

    if(exti_flag_get(VL53L8CX_INT_PIN))
    {
#if TOF_SENSOR_VL53L8CX
        vl53l8cx_data_ready = 1;   // VL53L8CX INT 数据就绪, tof_update 在下次 1ms tick 读取
#endif
    }

}

void gpio_4_exti_isr()                  // 外部 GPIO_4 中断服务函数     
{



}

void gpio_5_exti_isr()                  // 外部 GPIO_5 中断服务函数     
{



}


void gpio_6_exti_isr()                  // 外部 GPIO_6 中断服务函数     
{



}

void gpio_7_exti_isr()                  // 外部 GPIO_7 中断服务函数     
{



}

void gpio_8_exti_isr()                  // 外部 GPIO_8 中断服务函数     
{



}

void gpio_9_exti_isr()                  // 外部 GPIO_9 中断服务函数     
{



}

void gpio_10_exti_isr()                  // 外部 GPIO_10 中断服务函数     
{



}

void gpio_11_exti_isr()                  // 外部 GPIO_11 中断服务函数     
{



}

void gpio_12_exti_isr()                  // 外部 GPIO_12 中断服务函数     
{



}

void gpio_13_exti_isr()                  // 外部 GPIO_13 中断服务函数     
{



}

void gpio_14_exti_isr()                  // 外部 GPIO_14 中断服务函数     
{



}

void gpio_15_exti_isr()                  // 外部 GPIO_15 中断服务函数     
{



}

void gpio_16_exti_isr()                  // 外部 GPIO_16 中断服务函数     
{



}

void gpio_17_exti_isr()                  // 外部 GPIO_17 中断服务函数     
{



}

void gpio_18_exti_isr()                  // 外部 GPIO_18 中断服务函数     
{



}

void gpio_19_exti_isr()                  // 外部 GPIO_19 中断服务函数     
{



}

void gpio_20_exti_isr()                  // 外部 GPIO_20 中断服务函数     
{



}

void gpio_21_exti_isr()                  // 外部 GPIO_21 中断服务函数     
{



}

void gpio_22_exti_isr()                  // 外部 GPIO_22 中断服务函数     
{



}

void gpio_23_exti_isr()                  // 外部 GPIO_23 中断服务函数     
{



}
// **************************** 外部中断函数 ****************************