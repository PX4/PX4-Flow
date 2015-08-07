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

#include "mavlink_bridge_header.h"
#include <mavlink.h>
#include "utils.h"
#include "dcmi.h"
#include "timer.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_dcmi.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_tim.h"
#include "misc.h"
#include "stm32f4xx.h"
//#include "led.h"

struct _dcmi_transport_ctx;
typedef struct _dcmi_transport_ctx dcmi_transport_ctx;

/* DMA buffers */
uint8_t dcmi_dma_buffer_1[CONFIG_DCMI_DMA_BUFFER_SIZE];
uint8_t dcmi_dma_buffer_2[CONFIG_DCMI_DMA_BUFFER_SIZE];

bool dcmi_init(void *usr, 
			   camera_transport_transfer_done_cb transfer_done_cb, 
			   camera_transport_frame_done_cb frame_done_cb,
			   void *cb_usr);

void dcmi_reset(void *usr);

/**
 * @brief HW initialization of DCMI clock
 */
static void dcmi_clock_init();
/**
 * @brief HW initialization DCMI
 */
static void dcmi_hw_init(void);
/**
 * @brief  Configures DCMI and DMA to capture image from the camera.
 */
static void dcmi_dma_init();
/**
 * @brief Enable DCMI and DMA stream
 */
static void dcmi_dma_enable();
/**
 * @brief Initialize/Enable DCMI Interrupt
 */
static void dcmi_dma_it_init();

struct _dcmi_transport_ctx {
	/* assets */
	camera_transport_transfer_done_cb transfer_done_cb;
	camera_transport_frame_done_cb frame_done_cb;
	void *cb_usr;
	
	/* IRQ monitoring: */
	volatile uint32_t last_dma_irq_t;
	uint32_t last_dma_irq_pause_t;
};

static dcmi_transport_ctx dcmi_ctx;

const camera_transport_interface dcmi_transport_interface = {
	.usr                  = &dcmi_ctx,
	.transfer_size        = CONFIG_DCMI_DMA_BUFFER_SIZE,
	.init                 = dcmi_init,
	.reset                = dcmi_reset
};

const camera_transport_interface *dcmi_get_transport_interface() {
	return &dcmi_transport_interface;
}

bool dcmi_init(void *usr, 
			   camera_transport_transfer_done_cb transfer_done_cb, 
			   camera_transport_frame_done_cb frame_done_cb,
			   void *cb_usr) {
	dcmi_transport_ctx *ctx = (dcmi_transport_ctx *)usr;
	memset(ctx, 0, sizeof(dcmi_transport_ctx));
	/* init assets: */
	ctx->transfer_done_cb = transfer_done_cb;
	ctx->frame_done_cb    = frame_done_cb;
	ctx->cb_usr           = cb_usr;
	ctx->last_dma_irq_t   = get_boot_time_us();
	ctx->last_dma_irq_pause_t = 1000000;
	/* initialize hardware: */
	dcmi_clock_init();
	dcmi_hw_init();
	dcmi_dma_init();
	dcmi_dma_enable();
	dcmi_dma_it_init();
	return true;
}

void dcmi_reset(void *usr) {
	/* stop the DMA: */
	DMA_Cmd(DMA2_Stream1, DISABLE);
	/* clear pending interrupt: */
	if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1) != RESET) {
		DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);
	}
	/* re-enable DMA */
	DMA_Cmd(DMA2_Stream1, ENABLE);
}

/**
 * @brief Interrupt handler of DCMI
 */
void DCMI_IRQHandler(void) {
	if (DCMI_GetITStatus(DCMI_IT_FRAME) != RESET) {
		DCMI_ClearITPendingBit(DCMI_IT_FRAME);
		/* get context: */
		dcmi_transport_ctx *ctx = &dcmi_ctx;
		/* calculate time delta to the end of DMA2_Stream1_IRQHandler: 
		 * this should be almost zero because normally the DCMI interrupt will happen after the DMA interrupt */
		uint32_t dt = get_time_delta_us(ctx->last_dma_irq_t);
		/* take the last pause between DMA request as a basis to calculate the maximum allowed delta time: */
		uint32_t allowed_dt = ctx->last_dma_irq_pause_t / 4;
		if (allowed_dt < 10) allowed_dt = 10;
		/* typedef void (*camera_transport_frame_done_cb)(void *usr); */
		ctx->frame_done_cb(ctx->cb_usr, dt > allowed_dt);
	}
}

/**
 * @brief Interrupt handler of DCMI DMA stream
 */
void DMA2_Stream1_IRQHandler(void) {
	/* transfer completed */
	if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1) != RESET) {
		DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);
		/* get the buffer that has been completed: */
		//LEDOn(LED_ACT);
		void *buffer;
		if (DMA_GetCurrentMemoryTarget(DMA2_Stream1) == 0) {
			buffer = dcmi_dma_buffer_2;
		} else {
			buffer = dcmi_dma_buffer_1;
		}
		/* get context: */
		dcmi_transport_ctx *ctx = &dcmi_ctx;
		/* calculate the pause-time: */
		ctx->last_dma_irq_pause_t = get_time_delta_us(ctx->last_dma_irq_t);
		/* typedef void (*camera_transport_transfer_done_cb)(void *usr, const void *buffer, size_t size); */
		ctx->transfer_done_cb(ctx->cb_usr, buffer, CONFIG_DCMI_DMA_BUFFER_SIZE);
		/* store the time when the function finished. */
		ctx->last_dma_irq_t       = get_boot_time_us();
		//LEDOff(LED_ACT);
	}
}

static void dcmi_dma_it_init() {
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the DCMI global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = DCMI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DCMI_ITConfig(DCMI_IT_FRAME, ENABLE);
	
	/* Enable the DMA global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE); // transfer complete interrupt
}

static void dcmi_dma_enable() {
	/* Enable DMA2 stream 1 and DCMI interface then start image capture */
	DMA_Cmd(DMA2_Stream1, ENABLE);
	DCMI_Cmd(ENABLE);
	DCMI_CaptureCmd(ENABLE);
}

static void dcmi_clock_init() {
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* GPIOC clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	/* GPIOC Configuration:  TIM3 CH3 (PC8)  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Connect TIM3 pins to AF2 */;
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 3;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel3 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 2;// TIM_TimeBaseStructure.TIM_Period/2;

	TIM_OC3Init(TIM3, &TIM_OCInitStructure);

	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);

	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);
}

static void dcmi_hw_init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	/*** Configures the DCMI GPIOs to interface with the OV2640 camera module ***/
	/* Enable DCMI GPIOs clocks */
	RCC_AHB1PeriphClockCmd(
			RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC
					| RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE, ENABLE);

	/* Connect DCMI pins to AF13 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_DCMI); //DCMI_HSYNC
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_DCMI); //DCMI_PIXCL

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_DCMI); //DCMI_D5
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_DCMI); //DCMI_VSYNC

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_DCMI); //DCMI_D0
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_DCMI); //DCMI_D1

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_DCMI); //DCMI_D8
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_DCMI); //DCMI_D9

	GPIO_PinAFConfig(GPIOE, GPIO_PinSource0, GPIO_AF_DCMI); //DCMI_D2
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource1, GPIO_AF_DCMI); //DCMI_D3
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource4, GPIO_AF_DCMI); //DCMI_D4
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_DCMI); //DCMI_D6
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_DCMI); //DCMI_D7

	/* DCMI GPIO configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_10 | GPIO_Pin_12;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4
			| GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}

static void dcmi_dma_init() {
	DCMI_InitTypeDef DCMI_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	/*** Configures the DCMI to interface with the camera module ***/
	/* Enable DCMI clock */
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI, ENABLE);

	/* DCMI configuration */
	DCMI_InitStructure.DCMI_CaptureMode = DCMI_CaptureMode_Continuous;
	DCMI_InitStructure.DCMI_SynchroMode = DCMI_SynchroMode_Hardware;
	DCMI_InitStructure.DCMI_PCKPolarity = DCMI_PCKPolarity_Rising;
	DCMI_InitStructure.DCMI_VSPolarity = DCMI_VSPolarity_Low;
	DCMI_InitStructure.DCMI_HSPolarity = DCMI_HSPolarity_Low;
	DCMI_InitStructure.DCMI_CaptureRate = DCMI_CaptureRate_All_Frame;
	DCMI_InitStructure.DCMI_ExtendedDataMode = DCMI_ExtendedDataMode_8b;

	/* Configures the DMA2 to transfer Data from DCMI */
	/* Enable DMA2 clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	/* DMA2 Stream1 Configuration */
	DMA_DeInit(DMA2_Stream1);

	DMA_InitStructure.DMA_Channel = DMA_Channel_1;
	DMA_InitStructure.DMA_PeripheralBaseAddr = DCMI_DR_ADDRESS;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)dcmi_dma_buffer_1;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = CONFIG_DCMI_DMA_BUFFER_SIZE / 4; // buffer size in date unit (word)
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_DoubleBufferModeConfig(DMA2_Stream1, (uint32_t)dcmi_dma_buffer_2, DMA_Memory_0);
	DMA_DoubleBufferModeCmd(DMA2_Stream1, ENABLE);

	/* DCMI configuration */
	DCMI_Init(&DCMI_InitStructure);

	/* DMA2 IRQ channel Configuration */
	DMA_Init(DMA2_Stream1, &DMA_InitStructure);
}
