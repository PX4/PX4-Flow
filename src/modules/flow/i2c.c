///****************************************************************************
// *
// *   Copyright (C) 2013 Fortiss An-Institut TU Munchen All rights reserved.
// *   Author: Thomas Boehm <thomas.boehm@fortiss.org>
// *
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions
// * are met:
// *
// * 1. Redistributions of source code must retain the above copyright
// *    notice, this list of conditions and the following disclaimer.
// * 2. Redistributions in binary form must reproduce the above copyright
// *    notice, this list of conditions and the following disclaimer in
// *    the documentation and/or other materials provided with the
// *    distribution.
// *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
// * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
// * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// * POSSIBILITY OF SUCH DAMAGE.
// *
// ****************************************************************************/
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

/**
 * @file i2c.c
 * Definition of i2c frames.
 * @author Thomas Boehm <thomas.boehm@fortiss.org>
 * @author James Goppert <james.goppert@gmail.com>
 */

#include "px4_config.h"
#include "px4_macros.h"
#include "i2c.h"
#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

#include "led.h"
#include "i2c_frame.h"
#include "gyro.h"
#include "sonar.h"
#include "main.h"

#include "mavlink_bridge_header.h"
#include <mavlink.h>

/* prototypes */
void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);
char readI2CAddressOffset(void);

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

void i2c_init() {

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
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure2.NVIC_IRQChannel = I2C1_ER_IRQn;
	NVIC_InitStructure2.NVIC_IRQChannelPreemptionPriority = 0;
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

void update_TX_buffer(float pixel_flow_x, float pixel_flow_y,
		float flow_comp_m_x, float flow_comp_m_y, uint8_t qual,
		float ground_distance, float gyro_x_rate, float gyro_y_rate,
		float gyro_z_rate, int16_t gyro_temp, legacy_12c_data_t *pd) {
	static uint16_t frame_count = 0;

	i2c_frame f;
	i2c_integral_frame f_integral;

	f.frame_count = frame_count;
	f.pixel_flow_x_sum = pixel_flow_x * 10.0f;
	f.pixel_flow_y_sum = pixel_flow_y * 10.0f;
	f.flow_comp_m_x = flow_comp_m_x * 1000;
	f.flow_comp_m_y = flow_comp_m_y * 1000;
	f.qual = qual;
	f.ground_distance = ground_distance * 1000;

	f.gyro_x_rate = gyro_x_rate * getGyroScalingFactor();
	f.gyro_y_rate = gyro_y_rate * getGyroScalingFactor();
	f.gyro_z_rate = gyro_z_rate * getGyroScalingFactor();
	f.gyro_range = getGyroRange();

	uint32_t time_since_last_sonar_update;

	time_since_last_sonar_update = (get_boot_time_us()
			- get_sonar_measure_time());

	if (time_since_last_sonar_update < 255 * 1000) {
		f.sonar_timestamp = time_since_last_sonar_update / 1000; //convert to ms
	} else {
		f.sonar_timestamp = 255;
	}

	static float accumulated_flow_x = 0;
	static float accumulated_flow_y = 0;
	static uint16_t accumulated_framecount = 0;
	static uint16_t accumulated_quality = 0;
	static float accumulated_gyro_x = 0;
	static float accumulated_gyro_y = 0;
	static float accumulated_gyro_z = 0;
	static uint32_t integration_timespan = 0;
	static uint32_t lasttime = 0;

	/* calculate focal_length in pixel */
	const float focal_length_px = ((global_data.param[PARAM_FOCAL_LENGTH_MM])
			/ (4.0f * 6.0f) * 1000.0f); //original focal lenght: 12mm pixelsize: 6um, binning 4 enabled

	// reset if readout has been performed
	if (stop_accumulation == 1) {

		//debug output
//		mavlink_msg_optical_flow_send(MAVLINK_COMM_2, get_boot_time_us(),
//				global_data.param[PARAM_SENSOR_ID], accumulated_flow_x * 10.0f,
//				accumulated_gyro_x * 10.0f, integration_timespan,
//				accumulated_framecount, (uint8_t) (accumulated_quality / accumulated_framecount), ground_distance);

		integration_timespan = 0;
		accumulated_flow_x = 0;			 //mrad
		accumulated_flow_y = 0;			 //mrad
		accumulated_framecount = 0;
		accumulated_quality = 0;
		accumulated_gyro_x = 0;			 //mrad
		accumulated_gyro_y = 0;			 //mrad
		accumulated_gyro_z = 0;			 //mrad
		stop_accumulation = 0;
	}

	//accumulate flow and gyro values between sucessive I2C readings
	//update only if qual !=0
	if (qual > 0) {
		uint32_t deltatime = (get_boot_time_us() - lasttime);
		integration_timespan += deltatime;
		accumulated_flow_x += pixel_flow_y * 1000.0f / focal_length_px * 1.0f;//mrad axis swapped to align x flow around y axis
		accumulated_flow_y += pixel_flow_x * 1000.0f / focal_length_px * -1.0f;	//mrad
		accumulated_framecount++;
		accumulated_quality += qual;
		accumulated_gyro_x += gyro_x_rate * deltatime * 0.001f;	//mrad  gyro_x_rate * 1000.0f*deltatime/1000000.0f;
		accumulated_gyro_y += gyro_y_rate * deltatime * 0.001f;	//mrad
		accumulated_gyro_z += gyro_z_rate * deltatime * 0.001f;	//mrad
	}

	//update lasttime
	lasttime = get_boot_time_us();

	f_integral.frame_count_since_last_readout = accumulated_framecount;
	f_integral.gyro_x_rate_integral = accumulated_gyro_x * 10.0f;	//mrad*10
	f_integral.gyro_y_rate_integral = accumulated_gyro_y * 10.0f;	//mrad*10
	f_integral.gyro_z_rate_integral = accumulated_gyro_z * 10.0f; //mrad*10
	f_integral.pixel_flow_x_integral = accumulated_flow_x * 10.0f; //mrad*10
	f_integral.pixel_flow_y_integral = accumulated_flow_y * 10.0f; //mrad*10
	f_integral.integration_timespan = integration_timespan;     //microseconds
	f_integral.ground_distance = ground_distance * 1000;		    //mmeters
	f_integral.sonar_timestamp = time_since_last_sonar_update;  //microseconds
	f_integral.qual =
			(uint8_t) (accumulated_quality / accumulated_framecount); //0-255 linear quality measurement 0=bad, 255=best
	f_integral.gyro_temperature = gyro_temp;//Temperature * 100 in centi-degrees Celsius

	notpublishedIndexFrame1 = 1 - publishedIndexFrame1; // choose not the current published 1 buffer
	notpublishedIndexFrame2 = 1 - publishedIndexFrame2; // choose not the current published 2 buffer

	// HACK!! To get the data
	// In a V2 hw build with can and uavcan these macros 
	// are used to passs the data to uavcan.

        uavcan_export(&pd->frame, &f, I2C_FRAME_SIZE);
        uavcan_export(&pd->integral_frame, &f_integral, I2C_INTEGRAL_FRAME_SIZE);

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

	frame_count++;

}

char readI2CAddressOffset(void) {
	//read 3bit address offset of 7 bit address
	offset = 0x00;
	offset = GPIO_ReadInputData(GPIOC ) >> 13; //bit 0
	offset = offset | ((GPIO_ReadInputData(GPIOC ) >> 14) << 1); //bit 1
	offset = offset | ((GPIO_ReadInputData(GPIOC ) >> 15) << 2); //bit 2
	offset = (~offset) & 0x07;
	return offset;
}

char i2c_get_ownaddress1(void) {
	return (I2C1_OWNADDRESS_1_BASE + readI2CAddressOffset()) << 1; //add offset to base and shift 1 bit to generate valid 7 bit address
}
