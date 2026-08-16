#ifndef _TOF_H
#define _TOF_H

// ================== 传感器切换与引脚配置 ==================
// 置 1 后编译时包含 vl53l8cx_xtalk_calib_data.h 并加载校准数据
#define VL53L8CX_USE_XTALK_CALIB   1
// 1 = VL53L8CX (SPI), 0 = DL1B (软I2C, 逐飞库)
#define TOF_SENSOR_VL53L8CX  1

// 16区修剪均值: 小端/大端舍弃百分比 (0-100)
// 非零则至少舍弃1个, 舍弃数向下取整; 均0则不修剪
#define TOF_TRIM_LO_PCT  50
#define TOF_TRIM_HI_PCT  0

// SPI 引脚 (SPI_2, P15 端口 — 复用 IMU 位置测试)
#define VL53L8CX_SPI_IDX      SPI_3
#define VL53L8CX_SPI_CLK      SPI3_CLK_P03_2
#define VL53L8CX_SPI_MOSI     SPI3_MOSI_P03_1
#define VL53L8CX_SPI_MISO     SPI3_MISO_P03_0
#define VL53L8CX_CS_PIN       P03_3
//#define VL53L8CX_XSHUT_PIN    P07_2     // LPn 硬件拉高
#define VL53L8CX_INT_PIN      P03_4       // INT/GPIO1, 数据就绪中断
#define VL53L8CX_SPI_BAUDRATE (8 * 1000 * 1000)  // 5 MHz (match drone_test)

// ================== VL53L8CX 传感器声明 ==================
#if TOF_SENSOR_VL53L8CX
#include "vl53l8cx/vl53l8cx_api.h"
extern VL53L8CX_Configuration  vl53l8cx_dev;
extern VL53L8CX_ResultsData    vl53l8cx_results;
extern volatile uint8_t         vl53l8cx_data_ready;
#endif

// ================== 通用 ToF 接口 ==================
extern float    tof_actual_dt;       // 实测 TOF 帧间隔 (秒), 调试用
extern int16_t  tof_base_throttle;   // 高度 PID 修正量 (不含悬停基准, 倾角补偿在 fly_ctrl 中叠加)
extern float    z_rate;              // 调试: 高度位置环输出 (目标爬升率 cm/s)
extern float    z_acc;               // 调试: 高度速度环输出 (油门增量)
extern volatile uint32_t tof_update_seq; // 有效高度数据更新序号

void tof_init(void);
void tof_update(void);               // 每 1ms 调用，读取传感器并更新高度/高度PID

#endif
