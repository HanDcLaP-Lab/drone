

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
* 文件名称          main_cm7_1
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

// #define TEST_UART        UART_4 
// #define TEST_BAUDRATE    115200 
// #define TEST_TX_PIN      UART4_TX_P14_1 
// #define TEST_RX_PIN      UART4_RX_P14_0  
//----------------------------多核通讯-----------------------------//


//float uart_data[UART_DATA_LENGTH] = {0}; 

#define PIT_NUM3 (PIT_CH10)
#define PIT_NUM4 (PIT_CH11)

int32_t image_cnt = 0;

#pragma location = 0x28001000                                                   
volatile float share_data_from_1[M7_1_DATA_LENGTH] = {0};      // Core 1 写 -> Core 0 读

#pragma location = 0x28001040
volatile float share_data_from_0[M7_1_DATA_LENGTH] = {0};      // Core 0 写 -> Core 1 读



int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_info_init();                  // 调试串口信息初始化

    //uart_init(TEST_UART, TEST_BAUDRATE, TEST_TX_PIN, TEST_RX_PIN);
    camera_init();
    system_delay_ms(2000);
    display_init();
    key_switch_init();
    pit_ms_init(PIT_NUM3 , 10);
    while(true)
    {          
        // 等待摄像头采集完成 (同步物理帧率，50Hz)
        if (mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag = 0;
            
            // 1. 读取 IMU 数据前，先无效化 Cache (从 RAM 拉取 Core 0 写入的最新数据)
            SCB_InvalidateDCache_by_Addr(&share_data_from_0, sizeof(share_data_from_0));
            int drone_mode = (int)share_data_from_0[4];
            cam_down.threshold = (uint8_t)debug_params[0];
            image_processing_loop();
            
            // 使用最新的IMU数据(来自Core0)和最新的图像中心(来自image_processing_loop)进行解算
            // share_data_from_0: [0]=Roll, [1]=Pitch, [3]=Height
            if (drone_mode == 1) // DRONE_STATE_NORMAL_FLIGHT = 1
            {
                calculate_ground_positions(share_data_from_0[3], share_data_from_0[1], share_data_from_0[0]);
            }

            // 2. 写入视觉数据，并 Clean Cache (刷入 RAM 供 Core 0 读取)
            M7_1_data_send(share_data_from_1);
            share_data_from_1[15] = 1.0f;
            SCB_CleanDCache_by_Addr(&share_data_from_1, sizeof(share_data_from_1));
            if (drone_mode == 0) // DRONE_STATE_DEBUG = 0
            {
                // 将底层的 0/1 放大为 0/255 以便屏幕显示
                for(int i = 0; i < MT9V03X_H * MT9V03X_W; i++) {
                    image_copy[0][i] = cam_down.binarized_image[i] ? 255 : 0;
                }
                
                // 将要显示的数组刷入内存供 DMA 搬运
                SCB_CleanDCache_by_Addr((void*)image_copy, sizeof(image_copy));
                
                // 显示二值化图像 (假设全屏大小为 188x120)
                ips200_displayimage03x((const uint8 *)image_copy , MT9V03X_W, MT9V03X_H);
                
                // 在屏幕下方显示状态与阈值
                ips200_show_string(0, 16*9, "Mode: DEBUG");

                if (current_param_idx == 0) {
                    ips200_show_string(0, 16*10, "-> Thresh:"); // 带有指示箭头代表当前高亮选中
                } else {
                    ips200_show_string(0, 16*10, "   Thresh:"); // 未选中时用空格对齐
                }
                ips200_show_int(80, 16*10, cam_down.threshold, 3);
            }
            //UART
            //uart_write_buffer(TEST_UART, (const uint8_t *)uart_data, sizeof(uart_data));
        }
    }
}

// **************************** 代码区域 ****************************
