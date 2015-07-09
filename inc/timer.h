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

#ifndef TIMER_H_
#define TIMER_H_

#include <stdint.h>

#define NTIMERS         	16

/**	Initializes the timer module.
 */
void timer_init(void);

/**	Registers a new timer with a corresponding function.
 *	@note: The function will be called from within the timer_check function.
 *  @param timer_fn  The timer function to call when the timer rolls over.
 *					 This function is NOT called from the interrupt handler.
 * 	@param period_ms The period of the timer in milliseconds.
 */
void timer_register(void (*timer_fn)(void), uint32_t period_ms);

/**	Checks any pending timers and calls the respective timer functions.
 */
void timer_check(void);

void delay(uint16_t ms);

/** Returns the number of milliseconds since booting.
 */
uint32_t get_boot_time_ms(void);

/** Returns the number of microseconds since booting.
 */
uint32_t get_boot_time_us(void);

/** Computes the time delta in microseconds while taking the roll-over into account.
 */
uint32_t calculate_time_delta_us(uint32_t end, uint32_t start);

/** Computes the time delta in microseconds while taking the roll-over into account.
 */
uint32_t get_time_delta_us(uint32_t start);

/**
  * @brief  Triggered by systen timer interrupt every millisecond.
  * @param  None
  * @retval None
  */
void timer_update(void);

#endif /* TIMER_H_ */
