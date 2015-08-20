/****************************************************************************
 *
 *   Copyright (C) 2015 PX4 Development Team. All rights reserved.
 *   Author: David Sidrane<david_s5@nscdg.com>
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
#include <px4_config.h>

#include <stdint.h>
#include <stdbool.h>

#include <bsp/board.h>
#include <chip.h>
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"


#define TMR_FREQUENCY   STM32_TIMCLK1
/*
 *  PE[09] PE9/TIM1_CH1/FMC_D6                               40       TIM1_CH1 R Miswired to B
 *  PE[11] PE11/TIM1_CH2/8PI4_NSS/FMC_D8/LCD_G3              42       TIM1_CH2 G
 *  PE[13] PE13/TIM1_CH3/SPI4_MISO/FMC_D10/LCD_DE            44       TIM1_CH2 B Miswired to R
 */

/****************************************************************************
 * Name: board_led_rgb
 *
 * Description:
 *
 * Input Parameters:
 *
 *  red   - intensity of the red led
 *  green - intensity of the green led
 *  blue  - intensity of the blue led
 *  hz    - number of flashes per second
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/


void board_led_rgb(uint16_t red, uint16_t green , uint16_t blue,
    uint16_t hz)
{

	long fosc = TMR_FREQUENCY;
	long prescale = 2048;
	long p1s = fosc / prescale;
	long p0p5s  = p1s / 2;
	static bool once = 0;

	if (!once) {
		once = 1;


            RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
            GPIO_InitTypeDef GPIO_InitStructure;

            GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13);
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
            GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
            GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
            GPIO_Init(GPIOE, &GPIO_InitStructure);

            // Connect TIM1 pins to AF
            GPIO_PinAFConfig(GPIOE, GPIO_PinSource9,  GPIO_AF_TIM1);
            GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
            GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);

            TIM1->EGR |= TIM_PSCReloadMode_Immediate;
            TIM1->PSC = prescale;
            TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

            TIM1->CCMR1 = (TIM_OCMode_PWM1|TIM_CCMR1_OC1PE) | (TIM_OCMode_PWM1<<8|TIM_CCMR1_OC2PE);
            TIM1->CCMR2 = (TIM_OCMode_PWM1|TIM_CCMR1_OC2PE);

            TIM1->CCER |= (TIM_CCER_CC1P|TIM_CCER_CC1E|TIM_CCER_CC2P|TIM_CCER_CC2E|TIM_CCER_CC3P|TIM_CCER_CC3E);

            TIM_CCPreloadControl(TIM1, ENABLE);
            TIM_Cmd(TIM1, ENABLE);
            TIM_CtrlPWMOutputs(TIM1, ENABLE);
	}

	long p2  = hz == 0 ? p1s : p1s / hz;
	TIM1->ARR = p2;
        p2  = hz == 0 ? p1s + 1 : p0p5s / hz;
        TIM1->CCR3 = (red * p2) / 255;
        TIM1->CCR2 = (green * p2) / 255;
        TIM1->CCR1 = (blue * p2) / 255;

        TIM_Cmd(TIM1, hz == 0 ? DISABLE : ENABLE);

}

