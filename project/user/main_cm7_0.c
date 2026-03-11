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
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************

void M7_0_data_send(volatile float* data_out);
#pragma location = 0x28001000  
__root __no_init volatile float share_data_from_1[M7_1_DATA_LENGTH]; // Core 1 写 -> Core 0 读 (视觉数据)

#pragma location = 0x28001040  // 偏移64字节，确保与上面数组不在同一个Cache Line (32字节)
__root __no_init volatile float share_data_from_0[M7_1_DATA_LENGTH]; // Core 0 写 -> Core 1 读 (IMU数据)

float f_buffer[UART_DATA_LENGTH] = {0};

#define PIT_NUM0 (PIT_CH0)
#define PIT_NUM1 (PIT_CH1)
#define PIT_NUM2 (PIT_CH2)

#define LED1 (P19_0)
#define UART_KEY (P19_2)

int main(void) {
    clock_init(SYSTEM_CLOCK_250M);  // 时钟配置及系统初始化<务必保留>
    debug_init();                   // 调试串口信息初始化

    // 此处编写用户代码 例如外设初始化代码等
    system_delay_ms(1500);

    app_init();
    share_data_from_0[4] = (float)current_drone_state;
    SCB_CleanDCache_by_Addr((void*)&share_data_from_0, sizeof(share_data_from_0));

    gpio_init(UART_KEY, GPO, GPIO_HIGH, GPO_PUSH_PULL); //uart
    wireless_uart_init_();
    Board_Comm_Init();

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);

    //display_init();
    // 初始化逻辑已封装至 app_init 及 app_flight_start 中
    if (current_drone_state != DRONE_STATE_NORMAL_FLIGHT) {
        printf("DEBUG MODE: Flight Peripherals Bypassed.\r\n");
    }

    

    // 此处编写用户代码 例如外设初始化代码等

    while (true) {
        app_state_machine_update(); // 状态机轮询，检测模式切换
        seekfree_assistant_data_analysis();

        // 2. 检查是否有参数更新 (遍历所有通道)
        for (int i = 0; i < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; i++) {
            // 如果第 i 个通道有数据更新标志
            if (seekfree_assistant_parameter_update_flag[i]) {
                // 清除标志位
                seekfree_assistant_parameter_update_flag[i] = 0;
                
                // 将参数应用到 PID (通道号 = 索引 + 1)
                // seekfree_assistant_parameter[i] 是接收到的浮点数值
                //Fly_Param_Update(i + 1, seekfree_assistant_parameter[i]); 
                Fly_Param_Update_Visual(i + 1, seekfree_assistant_parameter[i]);
                
                // 可选：通过无线串口回传确认，告诉上位机收到并更新了
                // wireless_uart_send_string("Param Updated\r\n");
            }
        }

        // 1. 读取视觉数据前，先无效化 Cache (从 RAM 拉取 Core 1 写入的最新数据)
        SCB_InvalidateDCache_by_Addr((void*)&share_data_from_1, sizeof(share_data_from_1));
        
        if (current_drone_state == DRONE_STATE_NORMAL_FLIGHT) 
        {
            if (share_data_from_1[15] != 0.0f)
            {
                share_data_from_1[15] = 0.0f;
                Flight_Hover_Control_Task(); 
                SCB_CleanDCache_by_Addr((void*)&share_data_from_1, sizeof(share_data_from_1));
                
                F_Buffer_write(f_buffer, share_data_from_1);
                Board_Comm_Send_Data(f_buffer);
            }
        }
        
        // 2. 刷入 RAM 供 Core 1 读取
        M7_0_data_send(share_data_from_0);
        SCB_CleanDCache_by_Addr((void*)&share_data_from_0, sizeof(share_data_from_0));
        
        system_delay_ms(1); // 稍微延时
    }
}

// **************************** 代码区域 ****************************
void M7_0_data_send(volatile float* data_out) { // Core 0 调用，写入 data_out (share_data_from_0)
    data_out[0] = imu_data.roll; 
    data_out[1] = imu_data.pitch;
    data_out[2] = imu_data.yaw;
    data_out[3] = imu_data.z;
    data_out[4] = (float)current_drone_state;
}