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


#define PIT_NUM0              (PIT_CH0 )
#define PIT_NUM1              (PIT_CH1 )
#define PIT_NUM2              (PIT_CH2 )
#define LED1                    (P19_0) 

   
int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                       // 调试串口信息初始化
    
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);   
    wireless_uart_init_();
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
    while(1)///定时器0初始化
    {
        if(imu660ra_init())
        {
           printf("\r\n imu660ra init error.");                                 // imu660ra 初始化失败
        }
        else
        {
           break;
        }
        gpio_toggle_level(LED1);                                                // 翻转 LED 引脚输出电平 控制 LED 亮灭 初始化出错这个灯会闪的很慢
    }
    display_init();
    camera_init();
    Kalman_Init(&K_w_ax,1e-4f,0.01,0);
    Kalman_Init(&K_w_ay,1e-4f,0.01,0);

    while(1)
    {
        if(dl1b_init())
            gpio_toggle_level(LED1);                                            // 翻转 LED 引脚输出电平 控制 LED 亮灭 初始化出错这个灯会闪的很慢
        else
            {
                break;
            }
        system_delay_ms(1000);                                                  // 闪灯表示异常
    }
    
    motor_pwm_init(); ///pwm输出初始化
    Flight_Control_Init(); 
    
    pit_ms_init(PIT_NUM0, 1);  
    system_delay_ms(5000); 
    pit_ms_init(PIT_NUM1, 20);
    pit_ms_init(PIT_NUM2, 500);
    // 此处编写用户代码 例如外设初始化代码等
    
    
    // 此处编写用户代码 例如外设初始化代码等

    while(true)
    {
        // 此处编写需要循环执行的代码
         // wireless_uart_get_();
        // 此处编写需要循环执行的代码
    }
}

// **************************** 代码区域 ****************************
