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

#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include "settings.h"

#define TXBUFFERSIZE   	(64*64) // 4 KByte
#define RXBUFFERSIZE   	(64*64)

/* prototypes */
uint8_t usart2_tx_ringbuffer_push(const uint8_t* ch, uint8_t len);
uint8_t usart3_tx_ringbuffer_push(const uint8_t* ch, uint8_t len);
int usart2_char_available(void);
int usart3_char_available(void);
uint8_t usart2_rx_ringbuffer_pop(void);
uint8_t usart3_rx_ringbuffer_pop(void);
uint8_t usart2_rx_ringbuffer_push_from_usart(void);
uint8_t usart3_rx_ringbuffer_push_from_usart(void);
uint8_t usart2_tx_ringbuffer_pop_to_usart(void);
uint8_t usart3_tx_ringbuffer_pop_to_usart(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);
void usart_init(void);

/* fill output buffers with some asciis to start with */
uint8_t usart2_tx_buffer[TXBUFFERSIZE] = "\n\r    ____ _  ____ __  ________    ____ _       __\n\r   / __ \\ |/ / // / / ____/ /   / __ \\ |     / /\n\r  / /_/ /   / // /_/ /_  / /   / / / / | /| / / \n\r / ____/   /__  __/ __/ / /___/ /_/ /| |/ |/ /  \n\r/_/   /_/|_| /_/ /_/   /_____/\\____/ |__/|__/   \n\r                                                \n\r";
uint8_t usart2_rx_buffer[RXBUFFERSIZE] = "";
uint8_t usart3_tx_buffer[TXBUFFERSIZE] = "\n\r    ____ _  ____ __  ________    ____ _       __\n\r   / __ \\ |/ / // / / ____/ /   / __ \\ |     / /\n\r  / /_/ /   / // /_/ /_  / /   / / / / | /| / / \n\r / ____/   /__  __/ __/ / /___/ /_/ /| |/ |/ /  \n\r/_/   /_/|_| /_/ /_/   /_____/\\____/ |__/|__/   \n\r                                                \n\r";
uint8_t usart3_rx_buffer[RXBUFFERSIZE] = "";

int usart2_tx_counter_read = 0;
int usart2_tx_counter_write = 300;
int usart2_rx_counter_read = 0;
int usart2_rx_counter_write = 0;
int usart3_tx_counter_read = 0;
int usart3_tx_counter_write = 300;
int usart3_rx_counter_read = 0;
int usart3_rx_counter_write = 0;

/**
  * @brief  Push one byte to ringbuffer of USART2
  */
uint8_t usart2_tx_ringbuffer_push(const uint8_t* ch, uint8_t len)
{
	USART_ITConfig(USART2, USART_IT_TXE, DISABLE);

	/* if there is free space in buffer */
	if ((((usart2_tx_counter_read - usart2_tx_counter_write) - 1) + TXBUFFERSIZE) % TXBUFFERSIZE > len)
	{
		uint8_t i;
		for (i = 0; i < len; i++)
		{
			usart2_tx_buffer[usart2_tx_counter_write] = ch[i];
			usart2_tx_counter_write = (usart2_tx_counter_write + 1) % TXBUFFERSIZE;
		}

		USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
		return 1;
	}

	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
	return 0;
}

/**
  * @brief  Push one byte to ringbuffer of USART3
  */
uint8_t usart3_tx_ringbuffer_push(const uint8_t* ch, uint8_t len)
{
	USART_ITConfig(USART3, USART_IT_TXE, DISABLE);

	/* if there is free space in buffer */
	if ((((usart3_tx_counter_read - usart3_tx_counter_write) - 1) + TXBUFFERSIZE) % TXBUFFERSIZE > len)
	{
		uint8_t i;
		for (i = 0; i < len; i++)
		{
			usart3_tx_buffer[usart3_tx_counter_write] = ch[i];
			usart3_tx_counter_write = (usart3_tx_counter_write + 1) % TXBUFFERSIZE;
		}

		USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
		return 1;
	}

	USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
	return 0;
}

/**
  * @brief  Check character availability USART2
  */
int usart2_char_available(void)
{
	return (usart2_rx_counter_read != usart2_rx_counter_write);
}

/**
  * @brief  Check character availability USART3
  */
int usart3_char_available(void)
{
	return (usart3_rx_counter_read != usart3_rx_counter_write);
}

/**
  * @brief  Pop one byte from ringbuffer of USART2
  */
uint8_t usart2_rx_ringbuffer_pop(void)
{
	USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);

	uint8_t value = usart2_rx_buffer[usart2_rx_counter_read];
	usart2_rx_counter_read = (usart2_rx_counter_read + 1) % TXBUFFERSIZE;

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	return value;
}

/**
  * @brief  Pop one byte from ringbuffer of USART3
  */
uint8_t usart3_rx_ringbuffer_pop(void)
{
	USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);

	uint8_t value = usart3_rx_buffer[usart3_rx_counter_read];
	usart3_rx_counter_read = (usart3_rx_counter_read + 1) % TXBUFFERSIZE;

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	return value;
}

/**
  * @brief  Copy from USART2 to ringbuffer
  */
uint8_t usart2_rx_ringbuffer_push_from_usart(void)
{
	usart2_rx_buffer[usart2_rx_counter_write] = USART_ReceiveData(USART2);
	int temp = (usart2_rx_counter_write + 1) % TXBUFFERSIZE;

	if(temp == usart2_rx_counter_read)
	{
		return 0;
	}

	usart2_rx_counter_write = temp;
	return 1;
}

/**
  * @brief  Copy from USART3 to ringbuffer
  */
uint8_t usart3_rx_ringbuffer_push_from_usart(void)
{
	//USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
	usart3_rx_buffer[usart3_rx_counter_write] = USART_ReceiveData(USART3);
	int temp = (usart3_rx_counter_write + 1) % TXBUFFERSIZE;

	if(temp == usart3_rx_counter_read)
	{
		return 0;
	}

	usart3_rx_counter_write = temp;
	return 1;
}

/**
  * @brief  Copy from ringbuffer to USART2
  */
uint8_t usart2_tx_ringbuffer_pop_to_usart(void)
{
	if (usart2_tx_counter_read != usart2_tx_counter_write)
	{
		USART_SendData(USART2, usart2_tx_buffer[usart2_tx_counter_read]);
		usart2_tx_counter_read= (usart2_tx_counter_read+1) % TXBUFFERSIZE;
		return 1;
	}
	return 0;
}

/**
  * @brief  Copy from ringbuffer to USART3
  */
uint8_t usart3_tx_ringbuffer_pop_to_usart(void)
{
	if (usart3_tx_counter_read != usart3_tx_counter_write)
	{
		USART_SendData(USART3, usart3_tx_buffer[usart3_tx_counter_read]);
		usart3_tx_counter_read= (usart3_tx_counter_read+1) % TXBUFFERSIZE;
		return 1;
	}
	return 0;
}

/**
  * @brief  USART2 interrupt handler
  */
void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		if(usart2_rx_ringbuffer_push_from_usart() == 0)
		{
			/* Disable the Receive interrupt if buffer is full */
			USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
		}
		return;
	}

	if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
	{
		if(usart2_tx_ringbuffer_pop_to_usart() == 0)
		{
			/* Disable the Transmit interrupt if buffer is empty */
			USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
		}

		return;
	}
}

/**
  * @brief  USART3 interrupt handler
  */
void USART3_IRQHandler(void)
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		if(usart3_rx_ringbuffer_push_from_usart() == 0)
		{
			/* Disable the Receive interrupt if buffer is full */
			USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
		}
		return;
	}

	if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
	{
		if(usart3_tx_ringbuffer_pop_to_usart() == 0)
		{
			/* Disable the Transmit interrupt if buffer is empty */
			USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
		}

		return;
	}
}

/**
  * @brief  Configures USART2 and USART3 for communication
  */
void usart_init(void)
{
	/* Configures the nested vectored interrupt controller */
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the USARTx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable the USART clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Connect USART pins to AF7 */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	/* USART2 TX pin configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* USART2 RX pin configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* USART3 TX pin configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* USART3 RX pin configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* USARTx configured as follow:
		- BaudRate = 115200 baud
		- Word Length = 8 Bits
		- One Stop Bit
		- No parity
		- Hardware flow control disabled (RTS and CTS signals)
		- Receive and transmit enabled
	*/
	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_InitStructure.USART_BaudRate = global_data.param[PARAM_USART2_BAUD];
	USART_Init(USART2, &USART_InitStructure);
	USART_InitStructure.USART_BaudRate = global_data.param[PARAM_USART3_BAUD];
	USART_Init(USART3, &USART_InitStructure);

	USART_Cmd(USART2, ENABLE);
	USART_Cmd(USART3, ENABLE);

	/* Enable the Transmit interrupt: this interrupt is generated when
	* the transmit data register is empty
	*/
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
	USART_ITConfig(USART3, USART_IT_TXE, ENABLE);

	/* Enable the Receive interrupt: this interrupt is generated when
	* the receive data register is not empty
	*/
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}

