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

#ifndef USART_H_
#define USART_H_

#include <stdint.h>

/**
  * @brief  Configures USART2 and USART3 for communication
  */
void usart_init(void);

/**
  * @brief  Pop one byte from ringbuffer of USART2
  */
uint8_t usart2_rx_ringbuffer_pop(void);

/**
  * @brief  Push one byte to ringbuffer of USART2
  */
uint8_t usart2_tx_ringbuffer_push(const uint8_t* ch, uint8_t len);

/**
  * @brief  Pop one byte from ringbuffer of USART3
  */
uint8_t usart3_rx_ringbuffer_pop(void);

/**
  * @brief  Push one byte to ringbuffer of USART3
  */
uint8_t usart3_tx_ringbuffer_push(const uint8_t* ch, uint8_t len);

/**
  * @brief  Check character availability USART2
  */
int usart2_char_available(void);

/**
  * @brief  Check character availability USART3
  */
int usart3_char_available(void);

void USART2_IRQHandler(void);
void USART3_IRQHandler(void);

#endif /* USART_H_ */
