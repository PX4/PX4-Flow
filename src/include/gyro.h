/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef SPI_L3GD20_H_
#define SPI_L3GD20_H_

#include <stdint.h>
#include "settings.h"

/* M25P SPI Flash supported commands */
#define l3gd20_CMD_WRITE			0x02	/* Write to Memory instruction */
#define l3gd20_CMD_WRSR				0x01	/* Write Status Register instruction */
#define l3gd20_CMD_WREN				0x06	/* Write enable instruction */
#define l3gd20_CMD_READ				0x03	/* Read from Memory instruction */
#define l3gd20_CMD_RDSR				0x05	/* Read Status Register instruction  */
#define l3gd20_CMD_RDID				0x9F	/* Read identification */
#define l3gd20_CMD_SE				0xD8	/* Sector Erase instruction */
#define l3gd20_CMD_BE				0xC7	/* Bulk Erase instruction */

#define l3gd20_WIP_FLAG				0x01	/* Write In Progress (WIP) flag */
#define l3gd20_DUMMY_BYTE			0xAA

#define DIR_READ					(1<<7)
#define DIR_WRITE					(0<<7)
#define ADDR_INCREMENT				(1<<6)

#define ADDR_WHO_AM_I				0x0f
#define WHO_I_AM					0xd3

#define ADDR_CTRL_REG1				0x20	/* sample rate constants are in the public header */
#define REG1_POWER_NORMAL			(1<<3)
#define REG1_Z_ENABLE				(1<<2)
#define REG1_Y_ENABLE				(1<<1)
#define REG1_X_ENABLE				(1<<0)

#define ADDR_CTRL_REG2				0x21
#define ADDR_CTRL_REG3				0x22
#define ADDR_CTRL_REG4				0x23
#define REG4_BDU					(1<<7)
#define REG4_BIG_ENDIAN				(1<<6)
#define REG4_SPI_3WIRE				(1<<0)

#define ADDR_CTRL_REG5				0x24
#define REG5_BOOT					(1<<7)
#define REG5_FIFO_EN				(1<<6)
#define REG5_HIGHPASS_ENABLE		(1<<4)

#define ADDR_REFERENCE				0x25
#define ADDR_TEMPERATURE			0x26

#define L3GD20_TEMP_OFFSET_CELSIUS  40

#define ADDR_STATUS_REG				0x27
#define STATUS_ZYXOR				(1<<7)
#define SATAUS_ZOR					(1<<6)
#define STATUS_YOR					(1<<5)
#define STATUS_XOR					(1<<4)
#define STATUS_ZYXDA				(1<<3)
#define STATUS_ZDA					(1<<2)
#define STATUS_YDA					(1<<1)
#define STATUS_XDA					(1<<0)

#define ADDR_OUT_X					0x28	/* 16 bits */
#define ADDR_OUT_Y					0x2A	/* 16 bits */
#define ADDR_OUT_Z					0x2C	/* 16 bits */

#define ADDR_FIFO_CTRL				0x2e
#define FIFO_MODE_BYPASS			(0<<5)
#define FIFO_MODE_FIFO				(1<<5)
#define FIFO_MODE_STREAM			(2<<5)
#define FIFO_MODE_STREAM_TO_FIFO	(3<<5)
#define FIFO_MODE_BYPASS_TO_STREAM	(4<<5)
#define FIFO_THRESHOLD_MASK			0x1f

#define ADDR_FIFO_SRC				0x2f
#define FIFO_THREHSHOLD_OVER		(1<<7)
#define FIFO_OVERRUN				(1<<6)
#define FIFO_EMPTY					(1<<5)

/* SPIx Interface */
#define SPIx						SPI2
#define SPIx_CLK					RCC_APB1Periph_SPI2
#define SPIx_CLK_INIT				RCC_APB1PeriphClockCmd

#define SPIx_SCK_PIN				GPIO_Pin_13
#define SPIx_SCK_GPIO_PORT			GPIOB
#define SPIx_SCK_GPIO_CLK			RCC_AHB1Periph_GPIOB
#define SPIx_SCK_SOURCE				GPIO_PinSource13
#define SPIx_SCK_AF					GPIO_AF_SPI2

#define SPIx_MISO_PIN				GPIO_Pin_14
#define SPIx_MISO_GPIO_PORT			GPIOB
#define SPIx_MISO_GPIO_CLK			RCC_AHB1Periph_GPIOB
#define SPIx_MISO_SOURCE			GPIO_PinSource14
#define SPIx_MISO_AF				GPIO_AF_SPI2

#define SPIx_MOSI_PIN				GPIO_Pin_15
#define SPIx_MOSI_GPIO_PORT			GPIOB
#define SPIx_MOSI_GPIO_CLK			RCC_AHB1Periph_GPIOB
#define SPIx_MOSI_SOURCE			GPIO_PinSource15
#define SPIx_MOSI_AF				GPIO_AF_SPI2

#define l3gd20_CS_PIN				GPIO_Pin_12
#define l3gd20_CS_GPIO_PORT			GPIOB
#define l3gd20_CS_GPIO_CLK			RCC_AHB1Periph_GPIOB

/* Select l3gd20: Chip Select pin low */
#define l3gd20_CS_LOW()       GPIO_ResetBits(l3gd20_CS_GPIO_PORT, l3gd20_CS_PIN)
/* Deselect l3gd20: Chip Select pin high */
#define l3gd20_CS_HIGH()      GPIO_SetBits(l3gd20_CS_GPIO_PORT, l3gd20_CS_PIN)

/**
  * @brief  Configures Gyroscope.
  */
void gyro_config(void);

/**
 * @brief Read out newest gyro value
 */
void gyro_read(float* x_rate, float* y_rate, float* z_rate, int16_t* gyro_temp);

/* Low layer functions */
void spi_config(void);
void l3gd20_config(void);
uint8_t l3gd20_ReadByte(void);
uint8_t l3gd20_SendByte(uint8_t byte);
uint16_t l3gd20_SendHalfWord(uint16_t HalfWord);
void l3gd20_WriteEnable(void);
void l3gd20_WaitForWriteEnd(void);
uint8_t getGyroRange(void);
int getGyroScalingFactor(void);

#endif /* SPI_L3GD20_H_ */
