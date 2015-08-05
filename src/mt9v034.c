/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Laurens Mackay <mackayl@student.ethz.ch>
 *   		 Dominik Honegger <dominik.honegger@inf.ethz.ch>
 *   		 Petri Tanskanen <tpetri@inf.ethz.ch>
 *   		 Samuel Zihlmann <samuezih@ee.ethz.ch>
 *           Simon Laube <simon@leitwert.ch>
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

#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_gpio.h"
#include "mt9v034.h"

#include <string.h>

struct _mt9v034_sensor_ctx;
typedef struct _mt9v034_sensor_ctx mt9v034_sensor_ctx;

bool mt9v034_init(void *usr, const camera_img_param *img_param);
bool mt9v034_prepare_update_param(void *usr, const camera_img_param *img_param);
void mt9v034_restore_previous_param(void *usr);
void mt9v034_notify_readout_start(void *usr);
void mt9v034_get_current_param(void *usr, camera_img_param *img_param, bool *img_data_valid);

static bool mt9v034_init_hw(mt9v034_sensor_ctx *ctx);
static void mt9v034_configure_general(mt9v034_sensor_ctx *ctx);
static bool mt9v034_configure_context(mt9v034_sensor_ctx *ctx, int context_idx, const camera_img_param *img_param, bool full_refresh);

/**
  * @brief  Reads from a specific Camera register
  * @param  Addr: mt9v034 register address.
  * @retval data read from the specific register or 0xFFFF if timeout condition
  *         occurred.
  */
static uint16_t mt9v034_ReadReg(uint16_t Addr);
/**
  * @brief  Writes a byte at a specific Camera register
  * @param  Addr: mt9v034 register address.
  * @param  Data: Data to be written to the specific register
  * @retval true when operation is successful.
  */
static bool mt9v034_WriteReg(uint16_t Addr, uint16_t Data);

struct _mt9v034_sensor_ctx {
	/* context switching */
	
	volatile int cur_context;		///< The current camera sensor context that has been activated.
	volatile bool do_switch_context;///< When true the mt9v034_notify_readout_start function will switch the camera sensor context.
	volatile int desired_context;	///< The desired context when do_switch_context is true.
	volatile int previous_context;	///< The previous context index.
	
	camera_img_param context_p[2];	///< The parameters of the camera sensor register sets. (for context A and B)
	
	/* mirrored registers of the sensor */
	
	uint16_t chip_control_reg;		///< The current chip control register value.
	
	/* current frame parameter simulation model */
	
	camera_img_param cur_param;		///< The parameters of the frame that is beeing read out.
	bool cur_param_data_valid;		/**< Flag telling wether the image data is going to be valid.
									 *   The sensor outputs garbage on the first frame after changing resolution.
									 *   TODO: verify that this is always the case and why. */
	camera_img_param exp_param;		///< Because exposure parameters take one more frame time to propagate to the output this holds the exposure settings.
};

static mt9v034_sensor_ctx mt9v034_ctx;

const camera_sensor_interface mt9v034_sensor_interface = {
	.usr                  = &mt9v034_ctx,
	.init                 = mt9v034_init,
	.prepare_update_param = mt9v034_prepare_update_param,
	.restore_previous_param = mt9v034_restore_previous_param,
	.notify_readout_start = mt9v034_notify_readout_start,
	.get_current_param    = mt9v034_get_current_param
};

const camera_sensor_interface *mt9v034_get_sensor_interface() {
	return &mt9v034_sensor_interface;
}

bool mt9v034_init(void *usr, const camera_img_param *img_param) {
	mt9v034_sensor_ctx *ctx = (mt9v034_sensor_ctx *)usr;
	/* init context: */
	memset(ctx, 0, sizeof(mt9v034_sensor_ctx));
	ctx->cur_context = 0;
	ctx->desired_context = 0;
	ctx->previous_context = 0;
	ctx->do_switch_context = false;
	/* init hardware: */
	if (!mt9v034_init_hw(ctx)) return false;
	/* configure sensor: */
	mt9v034_configure_context(ctx, 0, img_param, true);
	mt9v034_configure_context(ctx, 1, img_param, true);
	mt9v034_configure_general(ctx);
	/* initialize the cur and exp param: */
	ctx->cur_param = *img_param;
	ctx->exp_param = *img_param;
	ctx->cur_param_data_valid = false;
	/* do a forced context update next. */
	ctx->do_switch_context = true;
	return true;
}

bool mt9v034_prepare_update_param(void *usr, const camera_img_param *img_param) {
	mt9v034_sensor_ctx *ctx = (mt9v034_sensor_ctx *)usr;
	/* deassert pending context switch command: */
	ctx->do_switch_context = false;
	/* determine which context to use to do the parameter update: */
	int cur_ctx_idx = ctx->cur_context;
	int new_ctx_idx = !cur_ctx_idx;
	if (!mt9v034_configure_context(ctx, new_ctx_idx, img_param, false)) {
		// invalid parameters.
		// check if there was a pending context switch when we started:
		if (cur_ctx_idx != ctx->desired_context) {
			ctx->do_switch_context = true;
		}
		return false;
	}
	/* setup context switching */
	ctx->previous_context  = ctx->cur_context;
	ctx->desired_context   = new_ctx_idx;
	ctx->do_switch_context = true;
	return true;
}

void mt9v034_restore_previous_param(void *usr) {
	mt9v034_sensor_ctx *ctx = (mt9v034_sensor_ctx *)usr;
	/* deassert pending context switch command: */
	ctx->do_switch_context = false;
	/* switch back to previous context */
	ctx->desired_context   = ctx->previous_context;
	ctx->do_switch_context = true;
}

void mt9v034_notify_readout_start(void *usr) {
	mt9v034_sensor_ctx *ctx = (mt9v034_sensor_ctx *)usr;
	camera_img_param *cur_ctx_param = &ctx->context_p[ctx->cur_context];
	/* update the sensor parameter simulation: */
	/* transfer exposure settings from exp_param into cur_param */
	//    There are no exposure settings yet.
	/* transfer exposure settings of currently active context into exp_param */
	//    There are no exposure settings yet.
	/* transfer other settings of currently active context into cur_param */
	if (cur_ctx_param->size.x != ctx->cur_param.size.x ||
	    cur_ctx_param->size.y != ctx->cur_param.size.y) {
		// first frame after changing resolution is invalid.
		ctx->cur_param_data_valid = false;
	} else {
		ctx->cur_param_data_valid = true;
	}
	ctx->cur_param.size    = cur_ctx_param->size;
	ctx->cur_param.binning = cur_ctx_param->binning;
	/* handle context switching: */
	if (ctx->do_switch_context) {
		/* update chip control register bit 15: */
		int desired_ctx_idx = ctx->desired_context;
		camera_img_param *ctx_param = &ctx->context_p[desired_ctx_idx];
		switch(desired_ctx_idx) {
			case 0: ctx->chip_control_reg &= 0x7FFF; break;
			case 1: ctx->chip_control_reg |= 0x8000; break;
		}
		mt9v034_WriteReg(MTV_CHIP_CONTROL_REG, ctx->chip_control_reg);
		/* update pixel count for AGC and AEC: */
		mt9v034_WriteReg(MTV_AGC_AEC_PIXEL_COUNT_REG, (uint32_t)ctx_param->size.x * (uint32_t)ctx_param->size.y);
		/* done. */
		ctx->do_switch_context = false;
		ctx->cur_context       = desired_ctx_idx;
	}
}

void mt9v034_get_current_param(void *usr, camera_img_param *img_param, bool *img_data_valid) {
	mt9v034_sensor_ctx *ctx = (mt9v034_sensor_ctx *)usr;
	*img_param = ctx->cur_param;
	*img_data_valid = ctx->cur_param_data_valid;
}

static void mt9v034_configure_general(mt9v034_sensor_ctx *ctx) {
	/*
	 * Chip control register.
	 * [2:0] Scan Mode:                 0 = Progressive scan.
	 * [4:3] Sensor operating mode:     1 = Master mode
	 *   [5] Stereoscopy Mode:          0 = Disabled
	 *   [6] Stereoscopic Master/Slave: 0 = Master
	 *   [7] Parallel output:           1 = Enabled
	 *   [8] Simultaneous mode:         1 = Enabled. Pixel and column readout take place in conjunction with exposure.
	 *  [15] Context A / B select       0 = Context A registers are used.
	 */
	ctx->chip_control_reg = 0x0188;
	mt9v034_WriteReg(MTV_CHIP_CONTROL_REG, ctx->chip_control_reg);

	/* settings that are the same for both contexts: */
	
	/*
	 * ADC Companding Mode
	 * [1:0] ADC Mode Context A:        2 = 10-bit linear
	 * [9:8] ADC Mode Context B:        2 = 10-bit linear
	 */
	mt9v034_WriteReg(MTV_ADC_RES_CTRL_REG, 0x0202);
	
	/*
	 * Row Noise Correction Control
	 *   [0] Noise Correction Ctx A:             0 / 1 = Disable / Enable
	 *   [1] Noise Use Black Level Avg Ctx A:    0 = Disable
	 *   [8] Noise Correction Ctx B:             0 / 1 = Disable / Enable
	 *   [9] Noise Use Black Level Avg Ctx B:    0 = Disable
	 */
	uint16_t row_noise_correction;
	if(global_data.param[PARAM_IMAGE_ROW_NOISE_CORR] && !global_data.param[PARAM_IMAGE_TEST_PATTERN]) {
		row_noise_correction = 0x0101;
	} else {
		row_noise_correction = 0x0000;
	}
	mt9v034_WriteReg(MTV_ROW_NOISE_CORR_CTRL_REG, row_noise_correction);
	
	/*
	 * Minimum Coarse Shutter Width
	 * Set to minimum. (1)
	 */
	mt9v034_WriteReg(MTV_MIN_COARSE_SW_REG, 0x0001);
	/*
	 * Maximum Coarse Shutter Width
	 */
	mt9v034_WriteReg(MTV_MAX_COARSE_SW_REG, CONFIG_MAX_EXPOSURE_ROWS);
	/*
	 * Maximum Analog Gain
	 */
	mt9v034_WriteReg(MTV_ANALOG_MAX_GAIN_REG, CONFIG_MAX_ANALOG_GAIN);
	
	/*
	 * AEC/AGC Desired Bin
	 * User-defined “desired bin” that gives a measure of how bright the image is intended to be.
	 * Range: 1 - 64.
	 */
	uint16_t desired_brightness;
	if(global_data.param[PARAM_IMAGE_LOW_LIGHT]) {
		desired_brightness = 58;
	} else {
		desired_brightness = 16;
	}
	mt9v034_WriteReg(MTV_AGC_AEC_DESIRED_BIN_REG, desired_brightness);
	/*
	 * AEC Update Frequency
	 * Number of frames to skip between changes in AEC
	 * Range: 0-15
	 */
	mt9v034_WriteReg(MTV_AEC_UPDATE_FREQ_REG, 2);
	/*
	 * AEC Low Pass Filter
	 * Range: 0-2
	 */
	mt9v034_WriteReg(MTV_AEC_LPF_REG, 1);
	/*
	 * AGC Output Update Frequency
	 * Number of frames to skip between changes in AGC
	 * Range: 0-15
	 */
	mt9v034_WriteReg(MTV_AGC_UPDATE_FREQ_REG, 2);
	/*
	 * AGC Low Pass Filter
	 * Range: 0-2
	 */
	mt9v034_WriteReg(MTV_AGC_LPF_REG, 2);
	/*
	 * AGC/AEC Enable
	 *   [0] AEC Enable Ctx A:                   1 = Enable
	 *   [1] AGC Enable Ctx A:                   1 = Enable
	 *   [8] AEC Enable Ctx B:                   1 = Enable
	 *   [9] AGC Enable Ctx B:                   1 = Enable
	 */
	mt9v034_WriteReg(MTV_AEC_AGC_ENABLE_REG, 0x0303);
	/*
	 * Sensor Type Control
	 *   [0] HDR Enable Ctx A:                   0 = Disable
	 *   [1] Color/Mono Sensor Control:          0 = Monochrome
	 *   [8] HDR Enable Ctx B:                   0 = Disable
	 */
	mt9v034_WriteReg(MTV_SENSOR_TYPE_CTRL_REG, 0x0000);
	
	/*
	 * Digital Test Pattern
	 *  [9:0] TWI Test Data:             0
	 *   [10] Use TWI Test Data:         0 = Disable.
	 *[12:11] Gray Shade Test Pattern:   0 / 2 = None / Horizontal Shades.
	 *   [13] Test Enable:               0 / 1 = Disable / Enable
	 *   [14] Flip TWI Test Data:        0 = Disable.
	 */
	uint16_t test_data;
	if(global_data.param[PARAM_IMAGE_TEST_PATTERN]) {
		test_data = 0x3000;
	} else {
		test_data = 0x0000;
	}
	mt9v034_WriteReg(MTV_TEST_PATTERN_REG, test_data);//enable test pattern

	/* Reset */
	mt9v034_WriteReg(MTV_SOFT_RESET_REG, 0x01);
	
	/*
	 * NOTES:
	 * Old code unexpectedly used:
	 *  - 12 to 10bit companding mode on 64x64 pixel image.
	 *  - Enabled AGC & AEC only on 64x64pixel image.
	 *  - Enabled HDR mode on big image.
	 *  - Used 64 x 64 = 4096 Pixels for AGC & AEC
	 *  - row / col bin 4 on 64x64
	 *  - row / col bin 2 on big image
	 */
}

static bool mt9v034_configure_context(mt9v034_sensor_ctx *ctx, int context_idx, const camera_img_param *img_param, bool full_refresh) {
	bool update_size    = full_refresh;
	bool update_binning = full_refresh;
	if (context_idx < 0 || context_idx >= 2) return false;
	camera_img_param *ctx_param = &ctx->context_p[context_idx];
	/* check which sets of parameters we need to update: */
	if (!full_refresh) {
		if (ctx_param->size.x != img_param->size.x ||
			ctx_param->size.y != img_param->size.y) {
			update_size = true;
		}
		if (ctx_param->binning != img_param->binning) {
			update_binning = true;
		}
	}
	
	/* 
	 * Coarse Shutter Width 1
	 * The row number in which the first knee occurs.
	 * Default: 0x01BB
	 */
	uint16_t coarse_sw1 = 0x01BB;
	/* 
	 * Coarse Shutter Width 2
	 * The row number in which the second knee occurs.
	 * Default: 0x01D9
	 */
	uint16_t coarse_sw2 = 0x01D9;
	/* 
	 * Shutter Width Control
	 * When Exposure Knee Point Auto Adjust is enabled,
	 * then one-half to the power of this value indicates the
	 * ratio of duration time t2, when saturation control
	 * gate is adjusted to level V2, to total coarse
	 * integration.
	 *  [3:0] T2 Ratio:					4 -> Tint_tot * (1/2) ^ n
	 *  [7:4] T3 Ratio:					6 -> Tint_tot * (1/2) ^ n
	 *    [8] Exposure Knee Auto Adj:	1 = Enabled.
	 *    [9] Single Knee Enable:		0 = Disabled.
	 * 
	 *  * Tint_tot = Coarse Shutter Width Total
	 * 
	 * NOTE: update vertical blanking if changing parameters here!
	 */
	uint16_t coarse_sw_ctrl = 0x0164;
	/*
	 * Coarse Shutter Width Total
	 * Total integration time in number of rows. This value is used only when AEC is disabled.
	 * Default: 0x01E0
	 */
	uint16_t coarse_sw_total = 0x01E0;

	/* image dimensions */
	uint16_t width  = img_param->size.x * img_param->binning;
	uint16_t height = img_param->size.y * img_param->binning;
	if (width  < 1 || width  > 752) return false;
	if (height < 1 || height > 480) return false;

	uint16_t col_start = 1 + (752 - width)  / 2;
	uint16_t row_start = 4 + (480 - height) / 2;
	
	/* 
	 * Horizontal Blanking
	 * Number of blank columns in a row. 
	 * Minimum horizontal blanking is 61 for normal mode, 
	 * 71 for column bin 2 mode, and 91 for column bin 4 mode.
	 * 
	 * The MT9V034 uses column parallel analog-digital converters, thus short row timing is not possible. 
	 * The minimum total row time is 690 columns (horizontal width + horizontal blanking). The minimum 
	 * horizontal blanking is 61( / 71 / 91). When the window width is set below 627, horizontal blanking 
	 * must be increased.
	 * 
	 * In old code for bin 4 and 64 pixels it was 350 + 91.
	 */
	uint16_t hor_blanking = 61;
	switch (img_param->binning) {
		case 1:
			/* init to minimum: */
			hor_blanking = 61;
			/* increase if window is smaller: */
			if (width < 627) {
				hor_blanking += 627 - width;
			}
			break;
		case 2:
			/* init to minimum: */
			hor_blanking = 71;
			/* increase if window is smaller: */
			if (width < 627) {
				hor_blanking += 627 - width;
			}
			break;
		case 4:
			/* init to minimum: */
			hor_blanking = 91;
			/* increase if window is smaller: */
			if (width < 627) {
				hor_blanking += 627 - width;
			}
			break;
	}
	
	/* 
	 * Vertical Blanking
	 * Number of blank rows in a frame. V-Blank value must meet the following minimums:
	 * Linear Mode:
	 * V-Blank (min) = SW_total - R0x08 + 7
	 * R0x08 (Coarse Shutter Width 1)
	 * If manual exposure    then SW_total = R0x0B. (Coarse Shutter Width Total)
	 * If auto-exposure mode then SW_total = R0xAD. (Maximum Coarse Shutter Width)
	 * 
	 * If Auto-Knee Point enabled, then V-Blank (min) = (t2 + t3 + 7).
	 * t2 = Tint_tot * (1/2) ^ T2 Ratio -> 4
	 * t3 = Tint_tot * (1/2) ^ T3 Ratio -> 1
	 */
	uint16_t ver_blanking = 4 + 1 + 7;

	/*
	 * Read Mode
	 *  [1:0] Row Bin:                   0 / 1 / 2 = Bin 1 / Bin 2 / Bin 4
	 *  [3:2] Col Bin:                   0 / 1 / 2 = Bin 1 / Bin 2 / Bin 4
	 *    [4] Row flip:                  0 = Disable.
	 *    [5] Column flip:               0 = Disable.
	 *    [6] Show Dark Rows:            0 = Disable.
	 *    [7] Show Dark Columns:         0 = Disable.
	 *  [9:8] Reserved:                  3 = Must be 3.
	 */
	uint16_t readmode = 0x0300;
	switch (img_param->binning) {
		case 1: readmode |= 0x0000; break;
		case 2: readmode |= 0x0005; break;
		case 4: readmode |= 0x000A; break;
		default: return false;
	}
	
	switch (context_idx) {
		case 0:
			if (update_size || update_binning) {
				mt9v034_WriteReg(MTV_WINDOW_WIDTH_REG_A, width);
				mt9v034_WriteReg(MTV_WINDOW_HEIGHT_REG_A, height);
				mt9v034_WriteReg(MTV_HOR_BLANKING_REG_A, hor_blanking);
				mt9v034_WriteReg(MTV_VER_BLANKING_REG_A, ver_blanking);
				mt9v034_WriteReg(MTV_READ_MODE_REG_A, readmode);
				mt9v034_WriteReg(MTV_COLUMN_START_REG_A, col_start);
				mt9v034_WriteReg(MTV_ROW_START_REG_A, row_start);
			}
			if (full_refresh) {
				mt9v034_WriteReg(MTV_COARSE_SW_1_REG_A, coarse_sw1);
				mt9v034_WriteReg(MTV_COARSE_SW_2_REG_A, coarse_sw2);
				mt9v034_WriteReg(MTV_COARSE_SW_CTRL_REG_A, coarse_sw_ctrl);
				mt9v034_WriteReg(MTV_COARSE_SW_TOTAL_REG_A, coarse_sw_total);
			}
			break;
		case 1:
			if (update_size || update_binning) {
				mt9v034_WriteReg(MTV_WINDOW_WIDTH_REG_B, width);
				mt9v034_WriteReg(MTV_WINDOW_HEIGHT_REG_B, height);
				mt9v034_WriteReg(MTV_HOR_BLANKING_REG_B, hor_blanking);
				mt9v034_WriteReg(MTV_VER_BLANKING_REG_B, ver_blanking);
				mt9v034_WriteReg(MTV_READ_MODE_REG_B, readmode);
				mt9v034_WriteReg(MTV_COLUMN_START_REG_B, col_start);
				mt9v034_WriteReg(MTV_ROW_START_REG_B, row_start);
			}
			if (full_refresh) {
				mt9v034_WriteReg(MTV_COARSE_SW_1_REG_B, coarse_sw1);
				mt9v034_WriteReg(MTV_COARSE_SW_2_REG_B, coarse_sw2);
				mt9v034_WriteReg(MTV_COARSE_SW_CTRL_REG_B, coarse_sw_ctrl);
				mt9v034_WriteReg(MTV_COARSE_SW_TOTAL_REG_B, coarse_sw_total);
			}
			break;
	}
	
	/* update the current settings: */
	*ctx_param = *img_param;
	
	return true;
}

static bool mt9v034_init_hw(mt9v034_sensor_ctx *ctx) {
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStruct;

	/* I2C2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

	/* GPIOB clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* Connect I2C2 pins to AF4 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);

	/* Configure I2C2 GPIOs */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* I2C DeInit */
	I2C_DeInit(I2C2);

	/* Enable the I2C peripheral */
	I2C_Cmd(I2C2, ENABLE);

	/* Set the I2C structure parameters */
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 = 0xFE;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStruct.I2C_ClockSpeed = 400000;

	/* Initialize the I2C peripheral w/ selected parameters */
	I2C_Init(I2C2, &I2C_InitStruct);

	/* Initialize GPIOs for EXPOSURE and STANDBY lines of the camera */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA, GPIO_Pin_2 | GPIO_Pin_3);
	
	/* test I2C: */
	uint16_t version = mt9v034_ReadReg(MTV_CHIP_VERSION_REG);
	if (version != 0x1324) return false;
	
	return true;
}

static bool mt9v034_I2cWaitEvent(uint32_t i2c_event) {
	uint32_t timeout = TIMEOUT_MAX; /* Initialize timeout value */
	while(!I2C_CheckEvent(I2C2, i2c_event)) {
		/* If the timeout delay is exceeded, exit with error code */
		if ((timeout--) == 0) return false;
	}
	return true;
}

static bool mt9v034_WriteReg(uint16_t Addr, uint16_t Data) {
	/* Generate the Start Condition */
	I2C_GenerateSTART(I2C2, ENABLE);

	/* Test on I2C2 EV5 and clear it */
	if (!mt9v034_I2cWaitEvent(I2C_EVENT_MASTER_MODE_SELECT))
		return false;

	/* Send DCMI selected device slave Address for write */
	I2C_Send7bitAddress(I2C2, mt9v034_DEVICE_WRITE_ADDRESS, I2C_Direction_Transmitter);

	/* Test on I2C2 EV6 and clear it */
	if (!mt9v034_I2cWaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		return false;

	/* Send I2C2 location address LSB */
	I2C_SendData(I2C2, (uint8_t)(Addr));

	/* Test on I2C2 EV8 and clear it */
	if (!mt9v034_I2cWaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		return false;
	
	/* Send Data MSB */
	I2C_SendData(I2C2, Data >> 8);

	/* Test on I2C2 EV8 and clear it */
	if (!mt9v034_I2cWaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		return false;

	/* Send Data LSB */
	I2C_SendData(I2C2, Data);

	/* Test on I2C2 EV8 and clear it */
	if (!mt9v034_I2cWaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		return false;

	/* Send I2C2 STOP Condition */
	I2C_GenerateSTOP(I2C2, ENABLE);

	/* If operation is OK, return true */
	return true;
}

static uint16_t mt9v034_ReadReg(uint16_t Addr) {
	uint16_t Data = 0;

	/* Generate the Start Condition */
	I2C_GenerateSTART(I2C2, ENABLE);

	/* Test on I2C2 EV5 and clear it */
	if (!mt9v034_I2cWaitEvent(I2C_EVENT_MASTER_MODE_SELECT))
		return 0xFFFF;

	/* Send DCMI selected device slave Address for write */
	I2C_Send7bitAddress(I2C2, mt9v034_DEVICE_READ_ADDRESS, I2C_Direction_Transmitter);

	/* Test on I2C2 EV6 and clear it */
	if (!mt9v034_I2cWaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		return 0xFFFF;
	
	/* Send I2C2 location address LSB */
	I2C_SendData(I2C2, (uint8_t)(Addr));

	/* Test on I2C2 EV8 and clear it */
	if (!mt9v034_I2cWaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		return 0xFFFF;

	/* Clear AF flag if set */
	I2C2->SR1 |= (uint16_t)0x0400;

	/* Generate the Start Condition */
	I2C_GenerateSTART(I2C2, ENABLE);

	/* Test on I2C2 EV6 and clear it */
	if (!mt9v034_I2cWaitEvent(I2C_EVENT_MASTER_MODE_SELECT))
		return 0xFFFF;

	/* Send DCMI selected device slave Address for write */
	I2C_Send7bitAddress(I2C2, mt9v034_DEVICE_READ_ADDRESS, I2C_Direction_Receiver);

	/* Test on I2C2 EV6 and clear it */
	if (!mt9v034_I2cWaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
		return 0xFFFF;

	/* Prepare an ACK for the next data received */
	I2C_AcknowledgeConfig(I2C2, ENABLE);

	/* Test on I2C2 EV7 and clear it */
	if (!mt9v034_I2cWaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED))
		return 0xFFFF;

	/* Receive the Data MSB */
	Data = ((uint16_t)I2C_ReceiveData(I2C2)) << 8;

	/* Prepare an NACK for the next data received */
	I2C_AcknowledgeConfig(I2C2, DISABLE);

	/* Test on I2C2 EV7 and clear it */
	if (!mt9v034_I2cWaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED))
		return 0xFFFF;

	/* Receive the Data LSB */
	Data |= I2C_ReceiveData(I2C2);

	/* Prepare Stop after receiving data */
	I2C_GenerateSTOP(I2C2, ENABLE);

	/* return the read data */
	return Data;
}
