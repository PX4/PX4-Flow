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

#ifndef MT9V34_H_
#define MT9V34_H_

#include <stdint.h>
#include "settings.h"
#include "camera.h"

/**
 * Configures the maximum exposure time in number of image rows.
 * Exposure should not affect frame time.
 * Set to number of rows in the image.
 */
#define CONFIG_MAX_EXPOSURE_ROWS (64)
/**
 * Configures the maximum analog gain.
 * Range: 1x - 4x => 16 - 64.
 * Higher gain means more noise.
 */
#define CONFIG_MAX_ANALOG_GAIN (64)

/* Constants */
#define TIMEOUT_MAX      				10000

#define BINNING_ROW_A					4
#define BINNING_COLUMN_A				4
#define BINNING_ROW_B					2
#define BINNING_COLUMN_B				2
#define MINIMUM_HORIZONTAL_BLANKING		91 // see datasheet
#define MAX_IMAGE_HEIGHT				480
#define MAX_IMAGE_WIDTH					752
#define MINIMUM_COLUMN_START			1
#define MINIMUM_ROW_START				4

/* Camera I2C registers */
#define mt9v034_DEVICE_WRITE_ADDRESS    0xB8
#define mt9v034_DEVICE_READ_ADDRESS     0xB9

/* Context A */
#define MTV_COLUMN_START_REG_A  		0x01
#define MTV_ROW_START_REG_A     		0x02
#define MTV_WINDOW_HEIGHT_REG_A 		0x03
#define MTV_WINDOW_WIDTH_REG_A  		0x04
#define MTV_HOR_BLANKING_REG_A  		0x05
#define MTV_VER_BLANKING_REG_A  		0x06
#define MTV_COARSE_SW_1_REG_A			0x08
#define MTV_COARSE_SW_2_REG_A			0x09
#define MTV_COARSE_SW_CTRL_REG_A		0x0A
#define MTV_COARSE_SW_TOTAL_REG_A 		0x0B
#define MTV_FINE_SW_1_REG_A				0xD3
#define MTV_FINE_SW_2_REG_A				0xD4
#define MTV_FINE_SW_TOTAL_REG_A			0xD5
#define MTV_READ_MODE_REG_A        		0x0D
#define MTV_V1_CTRL_REG_A				0x31
#define MTV_V2_CTRL_REG_A				0x32
#define MTV_V3_CTRL_REG_A				0x33
#define MTV_V4_CTRL_REG_A				0x34
#define MTV_ANALOG_GAIN_CTRL_REG_A		0x35
#define MTV_BLC_VALUE_REG_A				0x48

/* Context B */
#define MTV_COLUMN_START_REG_B  		0xC9
#define MTV_ROW_START_REG_B     		0xCA
#define MTV_WINDOW_HEIGHT_REG_B 		0xCB
#define MTV_WINDOW_WIDTH_REG_B  		0xCC
#define MTV_HOR_BLANKING_REG_B  		0xCD
#define MTV_VER_BLANKING_REG_B  		0xCE
#define MTV_COARSE_SW_1_REG_B			0xCF
#define MTV_COARSE_SW_2_REG_B			0xD0
#define MTV_COARSE_SW_CTRL_REG_B		0xD1
#define MTV_COARSE_SW_TOTAL_REG_B 		0xD2
#define MTV_FINE_SW_1_REG_B				0xD6
#define MTV_FINE_SW_2_REG_B				0xD7
#define MTV_FINE_SW_TOTAL_REG_B			0xD8
#define MTV_READ_MODE_REG_B        		0x0E
#define MTV_V1_CTRL_REG_B				0x39
#define MTV_V2_CTRL_REG_B				0x3A
#define MTV_V3_CTRL_REG_B				0x3B
#define MTV_V4_CTRL_REG_B				0x3C
#define MTV_ANALOG_GAIN_CTRL_REG_B		0x36

/* General Registers */
#define MTV_CHIP_VERSION_REG    		0x00
#define MTV_CHIP_CONTROL_REG    		0x07
#define MTV_SOFT_RESET_REG      		0x0C

#define MTV_SENSOR_TYPE_CTRL_REG		0x0F
#define MTV_LED_OUT_CTRL_REG			0x1B
#define MTV_ADC_RES_CTRL_REG			0x1C
#define MTV_VREF_ADC_CTRL_REG			0x2C
#define MTV_FRAME_DARK_AVG_REG			0x42
#define MTV_FRAME_DARK_AVG_THR_REG		0x46
#define MTV_BLC_CTRL_REG				0x47
#define MTV_BLC_VALUE_STEP_SIZE_REG		0x4C
#define MTV_ROW_NOISE_CORR_CTRL_REG		0x70
#define MTV_ROW_NOISE_CONST_REG			0x71
#define MTV_PIXEL_FRAME_LINE_CTRL_REG	0x72
#define MTV_TEST_PATTERN_REG       		0x7F
#define MTV_TILED_DIGITAL_GAIN_REG		0x80
#define MTV_AGC_AEC_DESIRED_BIN_REG		0xA5
#define MTV_AEC_UPDATE_FREQ_REG			0xA6
#define MTV_AEC_LPF_REG					0xA8
#define MTV_AGC_UPDATE_FREQ_REG			0xA9
#define MTV_AGC_LPF_REG					0xAA
#define MTV_ANALOG_MAX_GAIN_REG        	0xAB
#define MTV_MIN_COARSE_SW_REG			0xAC
#define MTV_MAX_COARSE_SW_REG    		0xAD
#define MTV_AEC_AGC_BIN_DIFF_THR_REG	0xAE
#define MTV_AEC_AGC_ENABLE_REG			0xAF
#define MTV_AGC_AEC_PIXEL_COUNT_REG		0xB0

#define MTV_AGC_GAIN_OUT_REG			0xBA
#define MTV_AEC_EXPOSURE_REG			0xBB
#define MTV_AGC_AEC_CUR_BIN_REG			0xBC


/*
 * Resolution:
 * ROW_SIZE * BINNING_ROW <= MAX_IMAGE_WIDTH
 * COLUMN_SIZE * BINNING_COLUMN <= MAX_IMAGE_HEIGHT
 */

#define FULL_IMAGE_SIZE (188*120)
#define FULL_IMAGE_ROW_SIZE (188)
#define FULL_IMAGE_COLUMN_SIZE (120)

/* Functions */

/**
  * @brief  Configures the mt9v034 camera with two context (binning 4 and binning 2).
  */
void mt9v034_context_configuration(void);

/**
  * @brief  Changes sensor context based on settings
  */
void mt9v034_set_context(void);

uint16_t mt9v034_ReadReg16(uint8_t address);
uint8_t mt9v034_WriteReg16(uint16_t address, uint16_t Data);
uint8_t mt9v034_ReadReg(uint16_t Addr);
uint8_t mt9v034_WriteReg(uint16_t Addr, uint8_t Data);

#endif /* MT9V34_H_ */
