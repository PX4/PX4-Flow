/****************************************************************************
 *
 *   Copyright (C) 2015 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_config.h>
#include <chip.h>
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

#if defined(CONFIG_ARCH_BOARD_PX4FLOW_V1)
#undef CONFIG_HAS_PROBES
#endif

#if defined(CONFIG_ARCH_BOARD_PX4FLOW_V2)
#define CONFIG_HAS_PROBES
#define PROBE_PORT  C
#define PROBE_PINS  (GPIO_Pin_2|GPIO_Pin_4|GPIO_Pin_5)
#endif

#define PROBE_GLUE2_(A, B)    A##B

#if defined(CONFIG_HAS_PROBES) && defined(CONFIG_USE_PROBES)

#define PROBE_INIT_(port, pins) {  \
          RCC_AHB1PeriphClockCmd(PROBE_GLUE2_(RCC_AHB1Periph_GPIO, port), ENABLE); \
          GPIO_InitTypeDef GPIO_InitStructure; \
            GPIO_InitStructure.GPIO_Pin = pins; \
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; \
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
            GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; \
            GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; \
            GPIO_Init(PROBE_GLUE2_(GPIO, port), &GPIO_InitStructure); \
            }

#define PROBE_INIT() PROBE_INIT_(PROBE_PORT, PROBE_PINS)
#define PROBE(port, pin, state) do { if(state) { PROBE_GLUE2_(GPIO, port)->BSRRH = pin;} else { PROBE_GLUE2_(GPIO, port)->BSRRL = pin;}} while(0);
#define PROBE_1(state)  PROBE(PROBE_PORT, GPIO_Pin_2, state)
#define PROBE_2(state)  PROBE(PROBE_PORT, GPIO_Pin_4, state)
#define PROBE_3(state)  PROBE(PROBE_PORT, GPIO_Pin_5, state)

#else
#define PROBE_INIT()
#define PROBE(port, pin, state)
#define PROBE_1(state)
#define PROBE_2(state)
#define PROBE_3(state)
#endif
