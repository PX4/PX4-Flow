///****************************************************************************
// *
// *   Copyright (C) 2013 Fortiss An-Institut TU Munchen All rights reserved.
// *   Author: Thomas Boehm <thomas.boehm@fortiss.org>
// *
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions
// * are met:
// *
// * 1. Redistributions of source code must retain the above copyright
// *    notice, this list of conditions and the following disclaimer.
// * 2. Redistributions in binary form must reproduce the above copyright
// *    notice, this list of conditions and the following disclaimer in
// *    the documentation and/or other materials provided with the
// *    distribution.
// *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
// * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
// * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// * POSSIBILITY OF SUCH DAMAGE.
// *
// ****************************************************************************/

/**
 * @file i2c.h
 * I2C communication functions.
 * @author Thomas Boehm <thomas.boehm@fortiss.org>
 */


#ifndef I2C_H_
#define I2C_H_
#include <inttypes.h>
#include <uavcan_if.h>

#define I2C1_OWNADDRESS_1_BASE 0x42 //7bit base address
/**
 * @brief  Configures I2C1 for communication as a slave (default behaviour for STM32F)
 */

void i2c_init(void);
void update_TX_buffer(float pixel_flow_x, float pixel_flow_y, float flow_comp_m_x, float flow_comp_m_y, uint8_t qual,
        float ground_distance, float x_rate, float y_rate, float z_rate, int16_t gyro_temp, legacy_12c_data_t *pd);
char i2c_get_ownaddress1(void);
#endif /* I2C_H_ */

