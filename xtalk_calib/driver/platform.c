/**
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */


#include "platform.h"

// VL53L8CX SPI: 模式3, 8-bit MSB先, bit15=1写/0读, 最大块4096
#define COMMS_CHUNK_SIZE    4096
#define SPI_WRITE_MASK(x)   ((uint16_t)((x) | 0x8000))
#define SPI_READ_MASK(x)    ((uint16_t)((x) & ~0x8000))

uint8_t VL53L8CX_RdByte(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_value)
{
	//uint8_t status = 255;
	
	/* Need to be implemented by customer. This function returns 0 if OK */

	return VL53L8CX_RdMulti(p_platform, RegisterAdress, p_value, 1);
}

uint8_t VL53L8CX_WrByte(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t value)
{
	//uint8_t status = 255;

	/* Need to be implemented by customer. This function returns 0 if OK */

	return VL53L8CX_WrMulti(p_platform, RegisterAdress, &value, 1);
}

uint8_t VL53L8CX_WrMulti(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	//int8_t status = 255;
	
		/* Need to be implemented by customer. This function returns 0 if OK */

	uint32_t pos;
    uint8_t  status = 0;

    for (pos = 0; pos < size; pos += COMMS_CHUNK_SIZE) {
        uint32_t chunk = size - pos;
        if (chunk > COMMS_CHUNK_SIZE) chunk = COMMS_CHUNK_SIZE;

        uint16_t addr = SPI_WRITE_MASK(RegisterAdress + pos);
        p_platform->wr_buffer[0] = (addr >> 8) & 0xFF;
        p_platform->wr_buffer[1] =  addr       & 0xFF;
        memcpy(&p_platform->wr_buffer[2], &p_values[pos], chunk);

        gpio_low(p_platform->cs_pin);
        spi_write_8bit_array(p_platform->spi_n, p_platform->wr_buffer, chunk + 2);
        gpio_high(p_platform->cs_pin);
    }
    return status;
}

uint8_t VL53L8CX_RdMulti(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	//uint8_t status = 255;
	
	/* Need to be implemented by customer. This function returns 0 if OK */
	
	uint32_t pos;
    uint8_t  status = 0;

    for (pos = 0; pos < size; pos += COMMS_CHUNK_SIZE) {
        uint32_t chunk = size - pos;
        if (chunk > COMMS_CHUNK_SIZE) chunk = COMMS_CHUNK_SIZE;

        uint16_t addr = SPI_READ_MASK(RegisterAdress + pos);
        uint8_t addr_buf[2];
        addr_buf[0] = (addr >> 8) & 0xFF;
        addr_buf[1] =  addr       & 0xFF;

        gpio_low(p_platform->cs_pin);
        spi_write_8bit_array(p_platform->spi_n, addr_buf, 2);
        spi_read_8bit_array (p_platform->spi_n, &p_values[pos], chunk);
        gpio_high(p_platform->cs_pin);
    }
    return status;
}

uint8_t VL53L8CX_Reset_Sensor(
		VL53L8CX_Platform *p_platform)
{
	uint8_t status = 0;
	
	/* (Optional) Need to be implemented by customer. This function returns 0 if OK */
	
	/* Set pin LPN to LOW */
	/* Set pin AVDD to LOW */
	/* Set pin VDDIO  to LOW */
	/* Set pin CORE_1V8 to LOW */
	gpio_low(p_platform->xshut_pin);
	VL53L8CX_WaitMs(p_platform, 100);

	/* Set pin LPN to HIGH */
	/* Set pin AVDD to HIGH */
	/* Set pin VDDIO to HIGH */
	/* Set pin CORE_1V8 to HIGH */
	gpio_high(p_platform->xshut_pin);
	VL53L8CX_WaitMs(p_platform, 100);

	return status;
}

void VL53L8CX_SwapBuffer(
		uint8_t 		*buffer,
		uint16_t 	 	 size)
{
	uint32_t i, tmp;
	
	/* Example of possible implementation using <string.h> */
	for(i = 0; i < size; i = i + 4) 
	{
		tmp = (
		  buffer[i]<<24)
		|(buffer[i+1]<<16)
		|(buffer[i+2]<<8)
		|(buffer[i+3]);
		
		memcpy(&(buffer[i]), &tmp, 4);
	}
}	

uint8_t VL53L8CX_WaitMs(
		VL53L8CX_Platform *p_platform,
		uint32_t TimeMs)
{
	//uint8_t status = 255;

	/* Need to be implemented by customer. This function returns 0 if OK */
	system_delay_ms(TimeMs);
	return 0;
}
