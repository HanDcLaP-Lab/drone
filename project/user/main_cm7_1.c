

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
void M7_1_data_send(volatile float* data_out);

#pragma location = 0x28001000                                                   
volatile float share_data_from_1[M7_1_DATA_LENGTH] = {0};      // Core 1 写 -> Core 0 读

#pragma location = 0x28001040
volatile float share_data_from_0[M7_1_DATA_LENGTH] = {0};      // Core 0 写 -> Core 1 读



int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_info_init();                  // 调试串口信息初始化

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
            
            // 1. 拉取 Core 0 写入的最新数据
            SCB_InvalidateDCache_by_Addr(&share_data_from_0, sizeof(share_data_from_0));
            int drone_mode = (int)share_data_from_0[4];
            cam_down.threshold = (uint8_t)debug_params[0];

            image_processing_loop();               // 执行核心视觉算法


            // 2. 刷入 RAM 供 Core 0 读取
            M7_1_data_send(share_data_from_1);
            share_data_from_1[15] = 1.0f; // 图像处理完成标志位，Core 0 可根据此位判断何时读取数据
            SCB_CleanDCache_by_Addr(&share_data_from_1, sizeof(share_data_from_1));

            // 3. 屏幕打印
            if (drone_mode == 0) // DRONE_STATE_DEBUG = 0
            {
                display_image_debug_display();
            }

            // 图像处理效率观测
            static uint16_t frame_cnt = 0;
            frame_cnt++;
            if (frame_cnt >= 100) {
                frame_cnt = 0;
                //printf("100");
            }
            //UART
            //uart_write_buffer(TEST_UART, (const uint8_t *)uart_data, sizeof(uart_data));
        }
    }
}

// **************************** 代码区域 ****************************
void M7_1_data_send(volatile float* data_out) { 
    // 读取独立的专属变量
    data_out[0] = cam_down.car_center_y;   // 原来的 centers[0][0]
    data_out[1] = cam_down.car_center_x;   // 原来的 centers[0][1]
    data_out[2] = (float)cam_down.car_area; 
    data_out[3] = car_ground_pos.x;
    data_out[4] = car_ground_pos.y;
    data_out[5] = target_ground_pos.x;
    data_out[6] = target_ground_pos.y;
    
    // 飞控只需知道视野内有几个有效灯(小车+信标)
    uint8_t effective_lights = 0;
    if (cam_down.car_valid) effective_lights++;
    if (cam_down.target_valid) effective_lights++;
    data_out[14] = (float)effective_lights;
    
    if (!cam_down.car_valid) {
        data_out[0] = 0;
        data_out[1] = 0;
        data_out[2] = 0;
    }
}