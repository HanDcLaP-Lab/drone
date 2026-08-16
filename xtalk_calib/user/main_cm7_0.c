/*********************************************************************************************************************
 * VL53L8CX Xtalk 校准工程 (基于 SeekFree 空工程, 仅 CM7_0 编写)
 * 共享 drone/libraries, CM7_1 保持空工程不变。
 * 注意: 不包含 zf_common_headfile.h, 避免引入飞控工程头文件。
 ********************************************************************************************************************/

#include <stdio.h>
#include <string.h>
#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_driver_uart.h"
#include "zf_driver_spi.h"
#include "zf_driver_gpio.h"
#include "zf_driver_delay.h"
#include "vl53l8cx/platform.h"
#include "vl53l8cx/vl53l8cx_api.h"
#include "vl53l8cx/vl53l8cx_plugin_xtalk.h"

// ================= 与飞控工程一致的 VL53L8CX 硬件配置 =================
#define XTALK_SPI_IDX      SPI_3
#define XTALK_SPI_CLK      SPI3_CLK_P03_2
#define XTALK_SPI_MOSI     SPI3_MOSI_P03_1
#define XTALK_SPI_MISO     SPI3_MISO_P03_0
#define XTALK_CS_PIN       P03_3
#define XTALK_SPI_BAUDRATE (8 * 1000 * 1000)

// 校准参数
#define XTALK_REFLECTANCE_PERCENT  3U
#define XTALK_NB_SAMPLES           16U
#define XTALK_DISTANCE_MM          600U

static VL53L8CX_Configuration g_vl53l8cx_dev;

static void VL53L8CX_Hw_Init(void) {
    spi_init(XTALK_SPI_IDX, SPI_MODE3, XTALK_SPI_BAUDRATE,
             XTALK_SPI_CLK, XTALK_SPI_MOSI, XTALK_SPI_MISO, SPI_CS_NULL);
    gpio_init(XTALK_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    g_vl53l8cx_dev.platform.spi_n  = XTALK_SPI_IDX;
    g_vl53l8cx_dev.platform.cs_pin = XTALK_CS_PIN;
}

static uint8_t VL53L8CX_Wait_Alive(void) {
    uint8_t is_alive = 0;
    while (1) {
        vl53l8cx_is_alive(&g_vl53l8cx_dev, &is_alive);
        if (is_alive) {
            printf("VL53L8CX detected\r\n");
            return 0;
        }
        printf("VL53L8CX not found, retry...\r\n");
        system_delay_ms(1000);
    }
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    debug_init();
    system_delay_ms(1500);

    printf("\r\n==== VL53L8CX Xtalk Calibration ====\r\n");

    VL53L8CX_Hw_Init();
    VL53L8CX_Wait_Alive();

    uint8_t status = vl53l8cx_init(&g_vl53l8cx_dev);
    if (status != 0) {
        printf("vl53l8cx_init error: %u\r\n", status);
        while (1);
    }
    printf("VL53L8CX init done\r\n");

    vl53l8cx_set_resolution(&g_vl53l8cx_dev, VL53L8CX_RESOLUTION_4X4);
    vl53l8cx_set_ranging_frequency_hz(&g_vl53l8cx_dev, 50);
    vl53l8cx_set_ranging_mode(&g_vl53l8cx_dev, VL53L8CX_RANGING_MODE_CONTINUOUS);
    vl53l8cx_set_target_order(&g_vl53l8cx_dev, VL53L8CX_TARGET_ORDER_CLOSEST);

    printf("Place a flat target at %u mm, covering full FOV.\r\n", XTALK_DISTANCE_MM);
    printf("Send any UART char to start xtalk calibration...\r\n");

    uint8_t rx = 0;
    //while (uart_query_byte(UART_0, &rx) == 0) {
    //}

    printf("Calibrating xtalk ...\r\n");
    status = vl53l8cx_calibrate_xtalk(&g_vl53l8cx_dev,
                                      XTALK_REFLECTANCE_PERCENT,
                                      XTALK_NB_SAMPLES,
                                      XTALK_DISTANCE_MM);
    if (status != 0) {
        printf("Xtalk calibration failed, status=0x%02X\r\n", status);
        if (status & VL53L8CX_STATUS_XTALK_FAILED) {
            printf("(XTALK_FAILED: coverglass too good or calibration target issue)\r\n");
        }
        while (1);
    }

    uint8_t xtalk[VL53L8CX_XTALK_BUFFER_SIZE];
    status = vl53l8cx_get_caldata_xtalk(&g_vl53l8cx_dev, xtalk);
    if (status != 0) {
        printf("Get xtalk data failed, status=0x%02X\r\n", status);
        while (1);
    }

    printf("\r\n// ============ 将以下内容保存为 vl53l8cx_xtalk_calib_data.h ============\r\n");
    printf("#ifndef _VL53L8CX_XTALK_CALIB_DATA_H_\r\n");
    printf("#define _VL53L8CX_XTALK_CALIB_DATA_H_\r\n");
    printf("#include <stdint.h>\r\n");
    printf("const uint8_t vl53l8cx_xtalk_calib_data[%u] = {\r\n", (unsigned int)VL53L8CX_XTALK_BUFFER_SIZE);
    for (uint16_t i = 0; i < VL53L8CX_XTALK_BUFFER_SIZE; i++) {
        if (i % 12 == 0) printf("\r\n    ");
        printf("0x%02X,", xtalk[i]);
    }
    printf("\r\n};\r\n");
    printf("#endif\r\n");
    printf("// ====================================================================\r\n");

    printf("Calibration done. Please copy the header above.\r\n");
    while (1);
}