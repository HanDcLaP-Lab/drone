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
 * 文件名称          main_cm7_0
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          IAR 9.40.1
 * 适用平台          CYT4BB
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2024-1-4       pudding            first version
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "debug_data.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************

// =================== 引脚与硬件宏定义 ===================
#define LED1         (P19_0)
#define UART_KEY     (P19_2)
#define DEBUG_PROBE  (P02_3)  // 示波器探头: 高=主循环计算执行中 (P02_0 已分配给光流 UART5_RX)

// =================== 全局与静态通信变量 ===================
float float_buffer[UART_DATA_LENGTH] = {0};

extern volatile uint8_t emergency_stop_print_pending;
extern volatile uint8_t periodic_print_pending;

// =================== 主程序入口 ===================

int main(void) {
    clock_init(SYSTEM_CLOCK_250M);  // 系统时钟 250MHz 初始化
    debug_init();                   // 调试串口初始化

    system_delay_ms(1500);

    // GPIO 与调试引脚初始化
    gpio_init(UART_KEY, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(DEBUG_PROBE, GPO, GPIO_LOW, GPO_PUSH_PULL);

    app_init();
    share_data_from_0[S0_DRONE_STATE] = (float)current_drone_state;
    SCB_CleanDCache_by_Addr((void*)&share_data_from_0, sizeof(share_data_from_0));

    { // 初始化
        main_kalman_init();
        // 2. 初始化底层传感器与执行器
        imu_init();
        tof_init();
        upixels_init();  // 光流模块 LC-302-3C (UART5 @19200)
        wireless_uart_init_();
        seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);
#if DUPLEX_SWITCH
        // 双向模式: 初始化 UART4/方向引脚(P19_2)/接收FIFO, 并接管 P19_2 进入接收态(低)
        Duplex_Comm_Init();
#else
        Board_Comm_Init();
#endif
        small_driver_uart_init();
        Flight_Control_Init();
        Calibration_Init(); // 起飞后悬停校准模块
        dataC.camera_offset_x = CAM_OFFSET_X;
        dataC.camera_offset_y = CAM_OFFSET_Y;

        system_delay_ms(1000);     // 等待传感器稳定

        // 3. 启动周期中断
        pit_ms_init(PIT_CH2, 500);  // 打印
        pit_ms_init(PIT_CH0, 1);       // TOF + 1ms 计时
        pit_us_init(PIT_CH1, 1250);    // IMU + 飞控 (1.25ms, 800Hz)
    }

    while (true) {
        gpio_high(DEBUG_PROBE);

        // =========================================================================
        // 【阶段 1】系统时间戳更新与运行期状态机
        // =========================================================================
#if DUPLEX_SWITCH
        Duplex_Comm_Set_Now_Ms(dataC.pit0_cnt);
#endif
        app_state_machine_update();         // 运行期拨码模式检测
        Calibration_Update();                // 起飞后悬停自校准状态机 (非阻塞)
        Flight_Nav_Mode_Update(imu_data.z); // 水平导航模式单点仲裁 (视觉 vs 光流)

        // =========================================================================
        // 【阶段 2】上位机调参处理与调试服务
        // =========================================================================
        debug_data_notify_handler();
        debug_data_send_handler();

        seekfree_assistant_data_analysis();
        for (int i = 0; i < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; i++) {
            if (seekfree_assistant_parameter_update_flag[i]) {
                seekfree_assistant_parameter_update_flag[i] = 0;
                Fly_Param_Update(i + 1, seekfree_assistant_parameter[i]);
            }
        }

        // =========================================================================
        // 【阶段 3】水平导航与外环控制调度 (视觉位置环 / 光流速度环)
        // =========================================================================
        
        // --- 3.1 跨核视觉数据拉取与一致性快照复核 ---
        if (Data_Complex_Sync_Vision_Snapshot()) {
            // 高高度执行视觉悬停位置与航向控制
            if (Flight_Get_Nav_Mode() == NAV_MODE_VISION_HOVER) {
                Flight_Hover_Control_Task();
            }

            // 打包下传数据并触发板间发送
            Float_Buffer_write(float_buffer, vision_snap);
#if DUPLEX_SWITCH
            Duplex_Comm_Trigger(float_buffer);
#else
            Board_Comm_Send_Data(float_buffer);
#endif
        } else if (Data_Complex_Is_Vision_Lost()) {
            // 视觉超时失联保护
            Flight_Hover_Lost_Protection();
        }

        // --- 3.2 光流解析、速度解算与低高度定点任务 ---
        uint8_t flow_frame_new = upixels_poll_and_calc(imu_data.z);
        if (Flight_Get_Nav_Mode() == NAV_MODE_OPTICAL_FLOW) {
            Flight_OpticalFlow_Control_Task(flow_frame_new);
        }

        // =========================================================================
        // 【阶段 4】跨核数据同步、板间通讯与异步串口日志
        // =========================================================================
        
        // 4.1 将 Core0 飞控与传感器数据刷回共享 RAM 供 Core1 读取
        M7_0_data_send(share_data_from_0);
        SCB_CleanDCache_by_Addr((void*)&share_data_from_0, sizeof(share_data_from_0));

#if DUPLEX_SWITCH
        // 4.2 双向通讯轮询处理接收 FIFO
        Duplex_Comm_Poll();
#endif

        // 4.3 异步打印服务
        if (emergency_stop_print_pending) {
            emergency_stop_print_pending = 0;
            wireless_uart_send_string("emergency stop\r\n");
        }
        if (periodic_print_pending) {
            periodic_print_pending = 0;
        }

        // 前馈角接收打印
        wireless_uart_output_feedforward_rx();

        gpio_low(DEBUG_PROBE);
        system_delay_us(400);
    }
}

