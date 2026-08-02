

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
static float frame_seq = 0.0f;  // 共享区帧序号 (S1_FRAME_SEQ 槽位, 仅 CM7_1 写)


int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_info_init();                  // 调试串口信息初始化

    camera_init();
    Kalman_Init(&K_car_x, IMAGE_POS_KALMAN_Q, IMAGE_POS_KALMAN_R, 0.0f);
    Kalman_Init(&K_car_y, IMAGE_POS_KALMAN_Q, IMAGE_POS_KALMAN_R, 0.0f);
    Kalman_Init(&K_target_x, IMAGE_POS_KALMAN_Q, IMAGE_POS_KALMAN_R, 0.0f);
    Kalman_Init(&K_target_y, IMAGE_POS_KALMAN_Q, IMAGE_POS_KALMAN_R, 0.0f);
    system_delay_ms(2000);
    display_init();
    key_switch_init();
    pit_ms_init(PIT_NUM3 , 10);
    Image_IMU_Snapshot_t frame_start_snap = {0};
    uint8_t frame_start_snap_valid = 0;
    while(true)
    {          
        // 等待摄像头采集完成 (同步物理帧率，50Hz)
        if (mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag = 0;

            // 原子快照: 短暂关中断, 防止相机ISR的memcpy写入mt9v03x_image
            // 与本帧处理竞态造成图像撕裂 (约50~110us @250MHz)
            __disable_irq();
            memcpy(cam_down.raw_image, mt9v03x_image, MT9V03X_IMAGE_SIZE);
            __enable_irq();

            // 1. 拉取 Core 0 写入的最新数据
            SCB_InvalidateDCache_by_Addr(&share_data_from_0, sizeof(share_data_from_0));

            Image_IMU_Snapshot_t frame_boundary_snap;
            frame_boundary_snap.roll   = share_data_from_0[S0_IMU_ROLL];
            frame_boundary_snap.pitch  = share_data_from_0[S0_IMU_PITCH];
            frame_boundary_snap.yaw    = share_data_from_0[S0_IMU_YAW];
            frame_boundary_snap.height = share_data_from_0[S0_IMU_HEIGHT];

            // 连续采集时，本帧完成边界也是下一帧的起始边界。
            // 用上一个边界的姿态处理刚完成的图像，避免使用帧尾姿态投影整帧像素。
            img_imu_snap = frame_start_snap_valid ? frame_start_snap : frame_boundary_snap;
            frame_start_snap = frame_boundary_snap;
            frame_start_snap_valid = 1;

            int drone_mode = (int)share_data_from_0[S0_DRONE_STATE];
            // 将 ISR 修改的 debug_params 同步到 cam_down 并重算 LUT，
            // 放在主循环而非 ISR 中以避免 thresh_by_rho2[] 的读写竞态。
            if ((uint8_t)debug_params[0] != cam_down.threshold_max) {
                threshold_max_update((uint8_t)debug_params[0]);
                cam_down.threshold_max = debug_params[0];
            }

            image_processing_loop();               // 执行核心视觉算法

            // 2. 刷入 RAM 供 Core 0 读取
            // 先写全部数据, 最后写递增帧序号并整区写回 (写序保证 Core0 复核序号时数据已完整落 RAM)
            M7_1_data_send(share_data_from_1);
            frame_seq += 1.0f;
            if (frame_seq >= 0x800000UL) frame_seq = 1.0f; // float 精确整数上限 2^24 内回绕
            share_data_from_1[S1_FRAME_SEQ] = frame_seq;
            SCB_CleanDCache_by_Addr(&share_data_from_1, sizeof(share_data_from_1));

            // 3. 屏幕打印
            if (drone_mode == 0) // DRONE_STATE_DEBUG = 0
            {
                display_image_debug_display(cam_down.car_center_x, cam_down.car_center_y, cam_down.car_valid, cam_down.target_center_x, cam_down.target_center_y, cam_down.target_valid);
            }

            // 图像处理效率观测
            static uint16_t frame_cnt = 0;
            frame_cnt++;
            if (frame_cnt >= 100) {
                frame_cnt = 0;
            }
        }
    }
}

// **************************** 代码区域 ****************************
