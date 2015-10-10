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

#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"

//Stated range of sensor
#define MINIMUM_DISTANCE 0.0f
#define MAXIMUM_DISTANCE 60.0f

#define LIDAR_ADDRESS 0x55

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
}


__EXPORT void distance_trigger(void)
{
	
}

__EXPORT void distance_readback(void)
{
	if(!i2c_started)
		return;
	I2C_GenerateSTART(I2C1, ENABLE);
}

static void lidar_process(uint16_t value);
void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);

__EXPORT void I2C1_EV_IRQHandler(void)
{
	if (I2C_GetITStatus(I2C1, I2C_IT_SB)) {
		I2C_Send7bitAddress(I2C1, LIDAR_ADDRESS << 1, I2C_Direction_Receiver);
	} else if (I2C_GetITStatus(I2C1, I2C_IT_ADDR)) {
		I2C_AcknowledgeConfig(I2C1, ENABLE);
		I2C_GenerateSTOP(I2C1, DISABLE);
		(void) I2C1->SR2;
	} else if (I2C_GetITStatus(I2C1, I2C_IT_BTF)) {
		I2C_AcknowledgeConfig(I2C1, DISABLE);
		I2C_GenerateSTOP(I2C1, ENABLE);
		
		uint16_t value = I2C_ReceiveData(I2C1);
		value |= I2C_ReceiveData(I2C1) << 8;
		lidar_process(value);
	}
}

__EXPORT void I2C1_ER_IRQHandler(void) {
	if((I2C_ReadRegister(I2C1, I2C_Register_SR1) & 0xFF00) != 0x00) {
		I2C_GenerateSTOP(I2C1, ENABLE);
		I2C1->SR1 &= 0x00FF;
	}
	sample_valid = false;
}

static void lidar_process(uint16_t sample16)
{
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
