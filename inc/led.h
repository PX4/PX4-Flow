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

#ifndef LED_H_
#define LED_H_

#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"

typedef enum
{
	LED_ACT = 0,	// Blue
	LED_COM = 1, 	// Amber
	LED_ERR = 2,	// Red
} Led_TypeDef;


#define LEDn						3

#define LED_ACTIVITY_PIN			GPIO_Pin_3
#define LED_ACTIVITY_GPIO_PORT		GPIOE
#define LED_ACTIVITY_GPIO_CLK		RCC_AHB1Periph_GPIOE

#define LED_BOOTLOADER_PIN			GPIO_Pin_2
#define LED_BOOTLOADER_GPIO_PORT	GPIOE
#define LED_BOOTLOADER_GPIO_CLK		RCC_AHB1Periph_GPIOE

#define LED_TEST_PIN				GPIO_Pin_7
#define LED_TEST_GPIO_PORT			GPIOE
#define LED_TEST_GPIO_CLK			RCC_AHB1Periph_GPIOE


void LEDInit(Led_TypeDef Led);
void LEDOn(Led_TypeDef Led);
void LEDOff(Led_TypeDef Led);
void LEDToggle(Led_TypeDef Led);


#endif /* LED_H_ */
