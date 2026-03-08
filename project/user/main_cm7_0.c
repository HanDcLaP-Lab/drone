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
//---------------------------------多核心通讯---------------------------------------------//
//#define DATA_LENGTH (8)  // 数组数据长度(移动至image.h文件中统一定义)

#pragma location = 0x28001000  
__root __no_init volatile float share_data_from_1[M7_1_DATA_LENGTH]; // Core 1 写 -> Core 0 读 (视觉数据)

#pragma location = 0x28001040  // 偏移64字节，确保与上面数组不在同一个Cache Line (32字节)
__root __no_init volatile float share_data_from_0[M7_1_DATA_LENGTH]; // Core 0 写 -> Core 1 读 (IMU数据)

float f_buffer[8] = {0.5, 1.5, 2.5, 3.5, 4.5, 1.5, 2.5, 2.5};

#define PIT_NUM0 (PIT_CH0)
#define PIT_NUM1 (PIT_CH1)
#define PIT_NUM2 (PIT_CH2)
#define PIT_NUM3 (PIT_CH10)
#define LED1 (P19_0)

int main(void) {
    clock_init(SYSTEM_CLOCK_250M);  // 时钟配置及系统初始化<务必保留>
    debug_init();                   // 调试串口信息初始化

    // 此处编写用户代码 例如外设初始化代码等
    system_delay_ms(1500);

    wireless_uart_init_();
    Board_Comm_Init();
    key_switch_init();
    pit_ms_init(PIT_NUM3, 10);
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);
    
    Kalman_Init(&K_w_ax,1e-3f,0.01,0);
    Kalman_Init(&K_w_ay,1e-3f,0.01,0);
    Kalman_Init(&K_groll,1e-3f,0.01f,0);
    Kalman_Init(&K_gpitch,1e-3f,0.01f,0);
    Kalman_Init(&K_gyaw,1e-3f,0.01f,0);
    
    // 加速度计滤波初始化 (Q=0.001, R=0.1 强滤波以抑制震动)
    Kalman_Init(&K_ax, 0.001f, 0.1f, 0);
    Kalman_Init(&K_ay, 0.001f, 0.1f, 0);
    Kalman_Init(&K_az, 0.001f, 0.1f, 9.8f); // Z轴初始设为重力

    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    display_init();
    imu_init();
    tof_init();
    motor_pwm_init();  /// pwm输出初始化
    Flight_Control_Init();

    pit_ms_init(PIT_NUM1, 20); // 图像处理中断 20ms
    pit_ms_init(PIT_NUM2, 400); // 输出中断 400ms
    system_delay_ms(1000);
    pit_ms_init(PIT_NUM0, 1); // 飞控主循环中断 1ms

    // 此处编写用户代码 例如外设初始化代码等

    while (true) {
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
        
        // 2. 写入 IMU 数据，并 Clean Cache (刷入 RAM 供 Core 1 读取)
        M7_1_data_send_m7_0(share_data_from_0);
        SCB_CleanDCache_by_Addr((void*)&share_data_from_0, sizeof(share_data_from_0));
        Board_Comm_Send_Data(f_buffer);
        system_delay_ms(5); // 稍微延时

    }
}

// **************************** 代码区域 ****************************
