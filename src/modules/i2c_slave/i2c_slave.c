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
#include <uavcan_if.h>
#include "fmu_comm.h"

#include "px4_config.h"
#include "px4_macros.h"
#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

#include "led.h"
#include "i2c_frame.h"
#include "gyro.h"
#include "sonar.h"
#include "main.h"
#include "result_accumulator.h"

#define I2C1_OWNADDRESS_1_BASE 0x42 //7bit base address

/* prototypes */
void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);

static char offset = 0;
uint8_t dataRX = 0;
uint8_t txDataFrame1[2][I2C_FRAME_SIZE];
uint8_t txDataFrame2[2][I2C_INTEGRAL_FRAME_SIZE];
uint8_t publishedIndexFrame1 = 0;
uint8_t publishedIndexFrame2 = 0;
uint8_t notpublishedIndexFrame1 = 1;
uint8_t notpublishedIndexFrame2 = 1;
uint8_t readout_done_frame1 = 1;
uint8_t readout_done_frame2 = 1;
uint8_t stop_accumulation = 0;

static char readI2CAddressOffset(void) {
	//read 3bit address offset of 7 bit address
	offset = 0x00;
	offset = GPIO_ReadInputData(GPIOC ) >> 13; //bit 0
	offset = offset | ((GPIO_ReadInputData(GPIOC ) >> 14) << 1); //bit 1
	offset = offset | ((GPIO_ReadInputData(GPIOC ) >> 15) << 2); //bit 2
	offset = (~offset) & 0x07;
	return offset;
}

static char i2c_get_ownaddress1(void) {
	return (I2C1_OWNADDRESS_1_BASE + readI2CAddressOffset()) << 1; //add offset to base and shift 1 bit to generate valid 7 bit address
}

__EXPORT void fmu_comm_init(void) {
	I2C_DeInit(I2C1 );       //Deinit and reset the I2C to avoid it locking up
	I2C_SoftwareResetCmd(I2C1, ENABLE);
	I2C_SoftwareResetCmd(I2C1, DISABLE);

	GPIO_InitTypeDef gpio_init;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	gpio_init.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	gpio_init.GPIO_Mode = GPIO_Mode_AF;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	//Pull up resistor
	gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
	//Open Drain
	gpio_init.GPIO_OType = GPIO_OType_OD;
	GPIO_Init(GPIOB, &gpio_init);

	GPIO_InitTypeDef gpio_init2;

	gpio_init2.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	gpio_init2.GPIO_Mode = GPIO_Mode_IN;
	gpio_init2.GPIO_Speed = GPIO_Speed_50MHz;
	gpio_init2.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &gpio_init2);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1 ); // SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1 ); // SDA

	NVIC_InitTypeDef NVIC_InitStructure, NVIC_InitStructure2;

	NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure2.NVIC_IRQChannel = I2C1_ER_IRQn;
	NVIC_InitStructure2.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure2.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure2.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure2);

	I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);
	I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);
	I2C_ITConfig(I2C1, I2C_IT_BUF, ENABLE);

	I2C_InitTypeDef i2c_init;
	i2c_init.I2C_ClockSpeed = 400000;
	i2c_init.I2C_Mode = I2C_Mode_I2C;
	i2c_init.I2C_DutyCycle = I2C_DutyCycle_2;
	i2c_init.I2C_OwnAddress1 = i2c_get_ownaddress1();
	i2c_init.I2C_Ack = I2C_Ack_Enable;
	i2c_init.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C1, &i2c_init);

	I2C_StretchClockCmd(I2C1, ENABLE);
	I2C_Cmd(I2C1, ENABLE);
}

__EXPORT void fmu_comm_run(void) {}

void I2C1_EV_IRQHandler(void) {

	//uint8_t dataRX;
	static uint8_t txDataIndex1 = 0x00;
	static uint8_t txDataIndex2 = 0x00;
	static uint8_t rxDataIndex = 0x00;
	switch (I2C_GetLastEvent(I2C1 )) {

	case I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED : {
		I2C1 ->SR1;
		I2C1 ->SR2;
		rxDataIndex = 0;
		break;
	}
	case I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED : {
		I2C1 ->SR1;
		I2C1 ->SR2;
		break;
	}
	case I2C_EVENT_SLAVE_BYTE_RECEIVED : {
		//receive address offset
		dataRX = I2C_ReceiveData(I2C1 );
		rxDataIndex++;
		//set Index
		txDataIndex1 = dataRX;
		if (dataRX > I2C_FRAME_SIZE) {
			txDataIndex2 = dataRX - I2C_FRAME_SIZE;
		}
		else {
			txDataIndex2 = 0;
		}
			//indicate sending
		readout_done_frame1 = 0;
		readout_done_frame2 = 0;
		break;
	}
	case I2C_EVENT_SLAVE_BYTE_TRANSMITTING :
	case I2C_EVENT_SLAVE_BYTE_TRANSMITTED : {

		if (txDataIndex1 < (I2C_FRAME_SIZE)) {
			I2C_SendData(I2C1,
					txDataFrame1[publishedIndexFrame1][txDataIndex1]);
			txDataIndex1++;
		} else {
			I2C_SendData(I2C1,
					txDataFrame2[publishedIndexFrame2][txDataIndex2]);
			if (txDataIndex2 < I2C_INTEGRAL_FRAME_SIZE) {
				txDataIndex2++;
			}

		}

		//check whether last byte is read frame1
		if (txDataIndex1 >= (I2C_FRAME_SIZE-1)) {
			readout_done_frame1 = 1;
		}

		//check whether last byte is read fram2 and reset accumulation
		if (txDataIndex2 >= (I2C_INTEGRAL_FRAME_SIZE-1)) {
			readout_done_frame2 = 1;
			stop_accumulation = 1;
		}

		break;
	}

	case I2C_EVENT_SLAVE_ACK_FAILURE : {
		I2C1 ->SR1 &= 0x00FF;
		break;
	}

	case I2C_EVENT_SLAVE_STOP_DETECTED : {
		I2C1 ->SR1;
		I2C1 ->CR1 |= 0x1;
		break;
	}
	}
}

void I2C1_ER_IRQHandler(void) {

	/* Read SR1 register to get I2C error */
	if ((I2C_ReadRegister(I2C1, I2C_Register_SR1 ) & 0xFF00) != 0x00) {
		/* Clears error flags */
		I2C1 ->SR1 &= 0x00FF;
	}
}


__EXPORT void fmu_comm_update(float dt, float x_rate, float y_rate, float z_rate, int16_t gyro_temp,
					  uint8_t qual, float pixel_flow_x, float pixel_flow_y, float rad_per_pixel,
					  bool distance_valid, float ground_distance, uint32_t distance_age) {
	static result_accumulator_ctx accumulator;
	static bool initialized = false;

	if (!initialized) {
		result_accumulator_init(&accumulator);
		initialized = true;
	}

	/* reset if readout has been performed	*/
	if (stop_accumulation == 1) {
		result_accumulator_reset(&accumulator);
		stop_accumulation = 0;
	}

	/* feed the accumulator and recalculate */
	result_accumulator_feed(&accumulator, dt, x_rate, y_rate, z_rate, gyro_temp,
							qual, pixel_flow_x, pixel_flow_y, rad_per_pixel, 
							distance_valid, ground_distance, distance_age);

	i2c_frame f;
	i2c_integral_frame f_integral;

	result_accumulator_fill_i2c_data(&accumulator, &f, &f_integral);

	/* perform buffer swapping */
	notpublishedIndexFrame1 = 1 - publishedIndexFrame1; // choose not the current published 1 buffer
	notpublishedIndexFrame2 = 1 - publishedIndexFrame2; // choose not the current published 2 buffer


  // fill I2C transmitbuffer1 with frame1 values
	memcpy(&(txDataFrame1[notpublishedIndexFrame1]),
		&f, I2C_FRAME_SIZE);

	// fill I2C transmitbuffer2 with frame2 values
	memcpy(&(txDataFrame2[notpublishedIndexFrame2]),
		&f_integral, I2C_INTEGRAL_FRAME_SIZE);

	//swap buffers frame1 if I2C bus is idle
	if (readout_done_frame1) {
		publishedIndexFrame1 = 1 - publishedIndexFrame1;
	}

	//swap buffers frame2 if I2C bus is idle
	if (readout_done_frame2) {
		publishedIndexFrame2 = 1 - publishedIndexFrame2;
	}
}
