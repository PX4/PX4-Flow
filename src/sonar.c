/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Laurens Mackay <mackayl@student.ethz.ch>
 *   		 Dominik Honegger <dominik.honegger@inf.ethz.ch>
 *   		 Petri Tanskanen <tpetri@inf.ethz.ch>
 *   		 Samuel Zihlmann <samuezih@ee.ethz.ch>
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

#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_dma.h"
#include "misc.h"
#include "utils.h"
#include "usart.h"
#include "settings.h"
#include "sonar.h"

extern int atoi (__const char *__nptr);
extern uint32_t get_boot_time_ms(void);

static char data_buffer[5]; // array for collecting decoded data

static volatile uint32_t last_measure_time = 0;
static volatile uint32_t measure_time = 0;
static volatile float dt = 0.0f;
static volatile int valid_data;
static volatile int data_counter = 0;
static volatile int data_valid = 0;
static volatile int new_value = 0;

static volatile uint32_t sonar_measure_time_interrupt = 0;
static volatile uint32_t sonar_measure_time = 0;

/* kalman filter states */
float x_pred = 0.0f; // m
float v_pred = 0.0f;
float x_post = 0.0f; // m
float v_post = 0.0f; // m/s

float sonar_raw = 0.0f;  // m

/**
  * @brief  Triggers the sonar to measure the next value
  *
  * see datasheet for more info
  */
void sonar_trigger(){
	GPIO_SetBits(GPIOE, GPIO_Pin_8);
}

/**
  * @brief  Sonar interrupt handler
  */
void UART4_IRQHandler(void)
{
	if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
	{
		/* Read one byte from the receive data register */
		uint8_t data = (USART_ReceiveData(UART4));

		if (data == 'R')
		{
			/* this is the first char (start of transmission) */
			data_counter = 0;
			data_valid = 1;

			/* set sonar pin 4 to low -> we want triggered mode */
			GPIO_ResetBits(GPIOE, GPIO_Pin_8);
		}
		else if (0x30 <= data && data <= 0x39)
		{
			if (data_valid)
			{
				data_buffer[data_counter] = data;
				data_counter++;
			}
		}
		else if (data == 0x0D)
		{
			if (data_valid && data_counter == 4)
			{
				data_buffer[4] = 0;
				int temp = atoi(data_buffer);

				/*
				 * 4744 or 4743 is invalid data (or upper/lower than maximal/minimal range)
				 */
				if (temp > 0 && temp < 4743)
				{
					/* it is in normal sensor range, take it */
					last_measure_time = measure_time;
					measure_time = get_boot_time_ms();
                    sonar_measure_time_interrupt = measure_time;
					dt = ((float)(measure_time - last_measure_time)) / 1000.0f;

					valid_data = temp;
					new_value = 1;
				}
			}

			data_valid = 0;
		}
		else
		{
			data_valid = 0;
		}
	}
}

/**
  * @brief  Basic Kalman filter
  */
void sonar_filter()
{
	/* no data for long time */
	if (dt > 0.25) // more than 2 values lost
	{
		v_pred = 0;
	}

	x_pred = x_post + dt * v_pred;
	v_pred = v_post;

	float x_new = ((float) valid_data) / 1000.0f;
	sonar_raw = x_new;
	x_post = x_pred + global_data.param[PARAM_SONAR_KALMAN_L1] * (x_new - x_pred);
	v_post = v_pred + global_data.param[PARAM_SONAR_KALMAN_L2] * (x_new - x_pred);

}


/**
  * @brief  Read out newest sonar data
  *
  * @param  sonar_value_filtered Filtered return value
  * @param  sonar_value_raw Raw return value
  */
void sonar_read(float* sonar_value_filtered, float* sonar_value_raw)
{
	/* getting new data with only around 10Hz */
	if(new_value) {
		sonar_filter();
		new_value = 0;
        sonar_measure_time = get_boot_time_ms();
	}

	*sonar_value_filtered = x_post;
	*sonar_value_raw = sonar_raw;

}

/**
 * @brief  Configures the sonar sensor Peripheral.
 */
void sonar_config(void)
{
	valid_data = 0;

	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIO clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	/* Configure l3gd20 CS pin in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* Configures the nested vectored interrupt controller. */
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the USARTx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the USART clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	/* Enable GPIO clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	/* Connect UART pins to AF7 */
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);

	GPIO_InitTypeDef GPIO_InitStructure_Serial2;
	GPIO_InitStructure_Serial2.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure_Serial2.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure_Serial2.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure_Serial2.GPIO_PuPd = GPIO_PuPd_UP;

	/* USART RX pin configuration */
	GPIO_InitStructure_Serial2.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOC, &GPIO_InitStructure_Serial2);

	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;

	/* Configure the UART4 */
	USART_Init(UART4, &USART_InitStructure);

	/* Enable UART4 interrupt */
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);

	USART_Cmd(UART4, ENABLE);

}

uint32_t get_sonar_measure_time()
{
    return sonar_measure_time;
}

uint32_t get_sonar_measure_time_interrupt()
{
    return sonar_measure_time_interrupt;
}

