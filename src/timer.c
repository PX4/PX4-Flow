
/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Petri Tanskanen <tpetri@inf.ethz.ch>
 *   		 Lorenz Meier <lm@inf.ethz.ch>
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

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"

#include "timer.h"
#include "led.h"

#define __INLINE inline
#define __ASM asm
#include "core_cmInstr.h"


#define MS_TIMER_COUNT		100 /* steps in 10 microseconds ticks */

/* boot time in milliseconds ticks */
volatile uint32_t boot_time_ms = 0;
volatile uint32_t boot_time_us_base = 0;

static uint32_t ticks_per_ms;

#define TIMER_FLAG_ENABLED 0x01
#define TIMER_FLAG_TRIGGERED 0x02

typedef struct _timer_info {
	uint32_t period;
	uint32_t countdown;
	void (*timer_fn)(void);
	volatile uint8_t flags;
} timer_info;

static timer_info timer[NTIMERS];

/**
  * @brief  Triggered by systen timer interrupt every millisecond.
  * @param  None
  * @retval None
  */
void timer_update(void)
{
	boot_time_ms++;
	/* we do it like this to have correct roll-over behaviour */
	boot_time_us_base += 1000u;

	/* each timer decrements every millisecond if > 0 */
	for (unsigned i = 0; i < NTIMERS; i++) {
		/* dont need exclusive access for reading the enabled flag because it will not be changed very often */
		if (timer[i].flags & TIMER_FLAG_ENABLED) {
			if (timer[i].countdown > 0) {
				timer[i].countdown--;
			} else {
				timer[i].countdown = timer[i].period - 1;
				/* set the flag: */
				timer[i].flags |= TIMER_FLAG_TRIGGERED;
				/* force a fail of the flags access instructions: */
				__CLREX();
			}
		}
	}
}

void timer_check(void) {
	for (unsigned i = 0; i < NTIMERS; i++) {
		bool triggered = false;
		uint8_t tmp;
		do {
			tmp = __LDREXB(&timer[i].flags);
			triggered = (tmp & TIMER_FLAG_TRIGGERED);
			if (triggered) {
				tmp &= ~TIMER_FLAG_TRIGGERED;
			}
		} while (!__STREXB(tmp, &timer[i].flags));
		if (triggered) {
			timer[i].timer_fn();
		}
	}
}

void timer_register(void (*timer_fn)(void), uint32_t period_ms) {
	/* find free timer: */
	int idx;
	for (idx = 0; idx < NTIMERS; idx++) {
		if ((timer[idx].flags & TIMER_FLAG_ENABLED) == 0) {
			break;
		}
	}
	if (idx >= NTIMERS) {
		/* capture error */
		LEDOn(LED_ERR);
		while (1);
	}
	/* setup the info struct: */
	timer[idx].period    = period_ms;
	timer[idx].countdown = period_ms - 1;
	timer[idx].timer_fn  = timer_fn;
	/* enable it: */
	timer[idx].flags = TIMER_FLAG_ENABLED;
}

void timer_init(void)
{
	/* init clock */
	ticks_per_ms = SystemCoreClock / 1000;
	/* init all timers */
	for (int idx = 0; idx < NTIMERS; idx++) {
		/* disable it: */
		timer[idx].flags = 0;
	}
	/* enable systick timer: */
	if (SysTick_Config(ticks_per_ms)) /* set timer to trigger interrupt every 1 millisecond */
	{
		/* capture clock error */
		LEDOn(LED_ERR);
		while (1);
	}
}

uint32_t get_boot_time_ms(void)
{
	// clear the COUNTFLAG:
	volatile bool dummy = (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0;
	volatile uint32_t val;
	do {
		// read the value:
		val = boot_time_ms;
		// make sure it did not roll over in the mean-time:
	} while(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk);
	return val;
}

uint32_t get_boot_time_us(void)
{
	// clear the COUNTFLAG:
	volatile bool dummy = (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0;
	volatile uint32_t val_us_base;
	volatile uint32_t val_tick;
	do {
		// read the value:
		val_us_base = boot_time_us_base;
		val_tick = SysTick->VAL;
		// make sure it did not roll over in the mean-time:
	} while(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk);
	// return the calculated value:
	return ((val_tick & SysTick_VAL_CURRENT_Msk) / ticks_per_ms) + val_us_base;
}

uint32_t calculate_time_delta_us(uint32_t end, uint32_t start) {
	return (int32_t)end - (int32_t)start;
}

uint32_t get_time_delta_us(uint32_t start)
{
	uint32_t now = get_boot_time_us();
	return calculate_time_delta_us(now, start);
}

void delay(uint16_t ms) {
	int32_t endt = get_boot_time_us() + (uint32_t)ms * 1000u;
	while ((endt - (int32_t)get_boot_time_us()) > 0);
}
