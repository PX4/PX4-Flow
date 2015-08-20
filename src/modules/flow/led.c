/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Samuel Zihlmann <samuezih@ee.ethz.ch>
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

#include "led.h"

/* LED GPIOs */
GPIO_TypeDef* LED_GPIO_PORTS[LEDn] = {LED_ACTIVITY_GPIO_PORT, LED_BOOTLOADER_GPIO_PORT, LED_TEST_GPIO_PORT};
const uint16_t LED_GPIO_PINS[LEDn] = {LED_ACTIVITY_PIN, LED_BOOTLOADER_PIN, LED_TEST_PIN};
const uint32_t LED_GPIO_CLKS[LEDn] = {LED_ACTIVITY_GPIO_CLK, LED_BOOTLOADER_GPIO_CLK, LED_TEST_GPIO_CLK};

/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured.
  *   This parameter can be one of following parameters:
  *     @arg LED_ACT
  *     @arg LED_COM
  *     @arg LED_ERR
  * @retval None
  */
void LEDInit(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* Enable the GPIO_LED Clock */
  RCC_AHB1PeriphClockCmd(LED_GPIO_CLKS[Led], ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = LED_GPIO_PINS[Led];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LED_GPIO_PORTS[Led], &GPIO_InitStructure);
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on.
  *   This parameter can be one of following parameters:
  *     @arg LED_ACT
  *     @arg LED_COM
  *     @arg LED_ERR
  * @retval None
  */
void LEDOn(Led_TypeDef Led)
{
	GPIO_ResetBits(LED_GPIO_PORTS[Led],LED_GPIO_PINS[Led]);
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off.
  *   This parameter can be one of following parameters:
  *     @arg LED_ACT
  *     @arg LED_COM
  *     @arg LED_ERR
  * @retval None
  */
void LEDOff(Led_TypeDef Led)
{
	GPIO_SetBits(LED_GPIO_PORTS[Led],LED_GPIO_PINS[Led]);
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled.
  *   This parameter can be one of following parameters:
  *     @arg LED_ACT
  *     @arg LED_COM
  *     @arg LED_ERR
  * @retval None
  */
void LEDToggle(Led_TypeDef Led)
{
	GPIO_ToggleBits(LED_GPIO_PORTS[Led],LED_GPIO_PINS[Led]);
}
