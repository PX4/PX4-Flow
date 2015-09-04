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

#ifndef DCMI_H_
#define DCMI_H_

#include <stdint.h>
#include "mt9v034.h"

#define DCMI_DR_ADDRESS       0x50050028

/**
 * @brief Copy image to fast RAM address
 */
void dma_copy_image_buffers(uint8_t ** current_image, uint8_t ** previous_image, uint16_t buffer_size, uint8_t image_step);

/**
 * @brief Send calibration image with MAVLINK over USB
 */
void send_calibration_image(uint8_t ** image_buffer_fast_1, uint8_t ** image_buffer_fast_2);

/**
 * @brief Initialize DCMI DMA and enable image capturing
 */
void enable_image_capture(void);

/* Init Functions */
void dcmi_clock_init(void);
void dcmi_hw_init(void);
void dcmi_dma_init(uint16_t buffer_size);
void dcmi_it_init(void);
void dma_it_init(void);

/* Interrupt Handlers */
void DCMI_IRQHandler(void);
void DMA2_Stream1_IRQHandler(void);

void dcmi_dma_enable(void);
void dcmi_dma_disable(void);
void dma_reconfigure(void);
void dcmi_restart_calibration_routine(void);
void dma_swap_buffers(void);

uint32_t get_time_between_images(void);
uint32_t get_frame_counter(void);
void reset_frame_counter(void);

#endif /* DCMI_H_ */
