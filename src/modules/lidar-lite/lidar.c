/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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

#include <stdbool.h>
#include "distance.h"
#include "usbd_cdc_vcp.h"
#include "main.h"
#include "distance_mode_filter.h"
#include "timer.h"

#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"

//Stated range of sensor
#define MINIMUM_DISTANCE 0.0f
#define MAXIMUM_DISTANCE 40.0f

#define LIDAR_ADDRESS 0x62
#define REG_CONTROL	0x0
#define REG_STATUS  0x1
//for bias calibration
#define REG_OFFSET  0x13
#define REG_HWVER   0x41
#define REG_STATUS2 0x47
#define REG_SWVER		0x4f
#define REG_DATA		0x8f
//REG_DATA is 2-byte read...

//CONTROL
#define BITS_ACQUIRE_WITH_CORRECTION 0x04
//STATUS
#define BITS_HEALTH 0x01
#define BITS_INVALID 0x08
//STATUS2
#define BITS_BUSY 0x01

bool i2c_started = false;

static void i2c_init_master(void)
{
	//Reset I2C
	I2C_DeInit(I2C1);
	I2C_SoftwareResetCmd(I2C1, ENABLE);
	I2C_SoftwareResetCmd(I2C1, DISABLE);

	//Turn on clocks
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	//Route I2C out to B8 (SCL) and B9 (SDA)
	GPIO_InitTypeDef gpio_init;
	gpio_init.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	gpio_init.GPIO_Mode = GPIO_Mode_AF;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
	gpio_init.GPIO_OType = GPIO_OType_OD;
	GPIO_Init(GPIOB, &gpio_init);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1); //SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1); //SDA

	//NVIC
	NVIC_InitTypeDef nvic_init;
	nvic_init.NVIC_IRQChannel = I2C1_EV_IRQn;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 0;
	nvic_init.NVIC_IRQChannelSubPriority = 0;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);
	nvic_init.NVIC_IRQChannel = I2C1_ER_IRQn;
	NVIC_Init(&nvic_init);

	//I2C block setup
	I2C_InitTypeDef i2c_init;
	i2c_init.I2C_ClockSpeed = 100000;
	i2c_init.I2C_Mode = I2C_Mode_I2C;
	i2c_init.I2C_DutyCycle = I2C_DutyCycle_2;
	i2c_init.I2C_OwnAddress1 = 0x0;
	i2c_init.I2C_Ack = I2C_Ack_Disable;
	i2c_init.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C1, &i2c_init);
	I2C_Cmd(I2C1, ENABLE);

	i2c_started = true;
}

/* Data from the LIDAR */

float sample, sample_filter;
uint32_t measure_time = 0;
bool sample_valid = false;

__EXPORT void distance_init(void)
{
	i2c_init_master();

	/* Run interrupt-based drivers after config */
	I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);
	I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);
	I2C_ITConfig(I2C1, I2C_IT_BUF, ENABLE);

}

/* In order to minimize busywait time, everything is done with interrupt callbacks after config.
 * lidar_trigger should be called periodically, and lidar_readback should be called periodically at least
 * 20ms after lidar_trigger is called.
 * An attempt is made to provide a clean interface (used in e.g. lidar_trigger), so that all the
 * state machine nastiness is encapsulated in I2C1_EV_IRQHandler...
 * Async I2C stuff eventually ends up in lidar_process...
 */

#define IDLE 0
#define WRITE 1
#define READ 2
//idle -> WRITE -> idle
//idle -> READ -> idle
uint8_t ld_state = IDLE;
uint8_t ld_reg = 0;
uint8_t ld_buffer[2] = {0};
uint8_t ld_length = 0;
uint8_t ld_xferred = 0;
uint8_t ld_address = 0;
uint32_t ld_nextev = 0;

__EXPORT void distance_trigger(void)
{
	if(!i2c_started)
		return;
		
	if (ld_state != IDLE) {
		// Previous reading did not complete
		distance_init();
		sample_valid = false;
	}
	
	ld_state = WRITE;
	ld_reg = REG_CONTROL;
	ld_buffer[0] = BITS_ACQUIRE_WITH_CORRECTION;
	ld_length = 1;
	ld_xferred = 0;
	ld_address = LIDAR_ADDRESS;
	ld_nextev = I2C_EVENT_MASTER_MODE_SELECT;
	I2C_GenerateSTART(I2C1, ENABLE);
}

__EXPORT void distance_readback(void)
{
	if(!i2c_started)
		return;
	ld_state = READ;
	ld_reg = REG_DATA;
	ld_length = 2;
	ld_xferred = 0;
	ld_address = LIDAR_ADDRESS;
	ld_nextev = I2C_EVENT_MASTER_MODE_SELECT;
	I2C_GenerateSTART(I2C1, ENABLE);
}

static void lidar_process(void);

void I2C1_EV_IRQHandler(void);
__EXPORT void I2C1_EV_IRQHandler(void)
{
	if(ld_nextev != 0 && I2C_CheckEvent(I2C1, ld_nextev)) {
		switch(ld_nextev) {
			//invariant: state != IDLE
			case I2C_EVENT_MASTER_MODE_SELECT:
				if(ld_state == WRITE || (ld_state == READ && ld_xferred == 0)) {
					I2C_Send7bitAddress(I2C1, ld_address << 1, I2C_Direction_Transmitter);
					ld_nextev = I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED;
				} else if(ld_state == READ) {
					I2C_Send7bitAddress(I2C1, ld_address << 1, I2C_Direction_Receiver);
					ld_nextev = I2C_EVENT_MASTER_BYTE_RECEIVED;
					if(ld_xferred == ld_length) { // last byte
						I2C_AcknowledgeConfig(I2C1, DISABLE);
						I2C_GenerateSTOP(I2C1, ENABLE);
					} else {
						I2C_AcknowledgeConfig(I2C1, ENABLE);
					}
				} else {
					I2C_GenerateSTOP(I2C1, ENABLE), ld_state = IDLE, ld_nextev = 0;
				}
				break;
			//invariant: state != IDLE, ld_xferred == 0
			case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
				if(ld_state != IDLE && ld_xferred == 0) {
					I2C_SendData(I2C1, ld_reg), ld_xferred++;
					ld_nextev = I2C_EVENT_MASTER_BYTE_TRANSMITTED;
				} else {
					I2C_GenerateSTOP(I2C1, ENABLE), ld_state = IDLE, ld_nextev = 0;
				}
				break;

			//invariant: state == WRITE, 0 < ld_xferred <= length+1
			//or: state == READ, ld_xferred == 1
			case I2C_EVENT_MASTER_BYTE_TRANSMITTED:
				if(ld_state == WRITE && ld_xferred == (ld_length + 1)) {
					I2C_GenerateSTOP(I2C1, ENABLE);
					ld_nextev = 0, ld_state = IDLE;
				} else if(ld_state == WRITE && ld_xferred > 0 && ld_xferred < (ld_length + 1)) {
					I2C_SendData(I2C1, ld_buffer[ld_xferred-1]);
				 	ld_xferred++;
				} else if(ld_state == READ && ld_xferred == 1) {
					I2C_GenerateSTOP(I2C1, ENABLE);
					I2C_GenerateSTART(I2C1, ENABLE);
					ld_nextev = I2C_EVENT_MASTER_MODE_SELECT;
				} else {
					I2C_GenerateSTOP(I2C1, ENABLE), ld_state = IDLE, ld_nextev = 0;
				}
				break;

			//invariant: state == READ, 0 < ld_xferred < length + 1
			case I2C_EVENT_MASTER_BYTE_RECEIVED:
				if(ld_state == READ && ld_xferred == ld_length) { //last byte
					ld_buffer[ld_xferred-1] = I2C_ReceiveData(I2C1);
					lidar_process();
					ld_state = IDLE, ld_nextev = 0;
				} else if(ld_state == READ && ld_xferred < ld_length && ld_xferred > 0) {
					ld_buffer[ld_xferred-1] = I2C_ReceiveData(I2C1);
					ld_xferred++;
				} else {
					I2C_GenerateSTOP(I2C1, ENABLE), ld_state = IDLE, ld_nextev = 0;
				}
				if(ld_nextev == I2C_EVENT_MASTER_BYTE_RECEIVED) {
					if(ld_xferred == ld_length) { // last byte
						I2C_AcknowledgeConfig(I2C1, DISABLE);
						I2C_GenerateSTOP(I2C1, ENABLE);
					} else {
						I2C_AcknowledgeConfig(I2C1, ENABLE);
					}
				}
				break;
			default:
				break;
		}
	} else if (I2C1->SR2 == 0 && I2C1->SR1 == 0x40) {
		I2C_ReceiveData(I2C1);
	}
}

void I2C1_ER_IRQHandler(void);
__EXPORT void I2C1_ER_IRQHandler(void) {
	if((I2C_ReadRegister(I2C1, I2C_Register_SR1) & 0xFF00) != 0x00) {
		I2C_GenerateSTOP(I2C1, ENABLE);
		I2C1->SR1 &= 0x00FF;
		ld_state = IDLE;
	}
	sample_valid = false;
}

static void lidar_process(void)
{
	if(ld_reg == REG_DATA) {
		int16_t sample16 = (((int16_t)ld_buffer[1]) << 8) | ld_buffer[0];
		if(sample16 <= 0) { // 0 or less means invalid data
		sample_valid = false;
			return;
		}
		sample16 = ((sample16 >> 8) & 0xFF) | ((sample16 << 8) & 0xFF00);
		sample = (sample16 / 100.0f); /* convert cm->m */
		if(sample < MINIMUM_DISTANCE || sample > MAXIMUM_DISTANCE) {
			sample_valid = false;
			return;
		}
		sample_filter = insert_distance_value_and_get_mode_value(sample);
		measure_time = get_boot_time_us();
		sample_valid = true;
	}
}

__EXPORT bool distance_read(float* distance_filtered, float* distance_raw)
{
	if(sample_valid) {
		*distance_raw = sample;
		*distance_filtered = sample_filter;
		return true;
	}
	return false;
}

__EXPORT uint32_t get_distance_measure_time(void)
{
	return measure_time;
}
