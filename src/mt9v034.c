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

typedef struct _mt9v034_sensor_ctx {
	int cur_context;
	camera_img_param cur_param;
	camera_img_param exp_param;
	camera_img_param context_a;
	camera_img_param context_b;
} mt9v034_sensor_ctx;

int mt9v034_init(void *usr, const camera_img_param *img_param);
int mt9v034_prepare_update_param(void *usr, const camera_img_param *img_param);
void mt9v034_notify_readout_start(void *usr);
void mt9v034_get_current_param(void *usr, camera_img_param *img_param);

static int mt9v034_init_hw(mt9v034_sensor_ctx *ctx);
static void mt9v034_configure_general(mt9v034_sensor_ctx *ctx);
static void mt9v034_configure_context(mt9v034_sensor_ctx *ctx, int context_idx, const camera_img_param *img_param);

static mt9v034_sensor_ctx mt9v034_ctx;

const camera_sensor_interface mt9v034_sensor_interface = {
	.usr                  = &mt9v034_ctx,
	.init                 = mt9v034_init,
	.prepare_update_param = mt9v034_prepare_update_param,
	.notify_readout_start = mt9v034_notify_readout_start,
	.get_current_param    = mt9v034_get_current_param
};


int mt9v034_init(void *usr, const camera_img_param *img_param) {
	mt9v034_sensor_ctx *ctx = (mt9v034_sensor_ctx *)usr;
	memset(ctx, 0, sizeof(mt9v034_sensor_ctx));
	ctx->cur_context = 0;
	if (mt9v034_init_hw(ctx) != 0) return -1;
	mt9v034_configure_context(ctx, 0, img_param);
	mt9v034_configure_context(ctx, 1, img_param);
	mt9v034_configure_general(ctx);
}

int mt9v034_prepare_update_param(void *usr, const camera_img_param *img_param);

void mt9v034_notify_readout_start(void *usr) {
	// TODO: decide when to switch context:
	
	mt9v034_WriteReg16(MTV_AGC_AEC_PIXEL_COUNT_REG, pixel_count);
	
	uint16_t new_control;
	if ()
		new_control = 0x8188; // Context B
	else
		new_control = 0x0188; // Context A

	mt9v034_WriteReg16(MTV_CHIP_CONTROL_REG, new_control);
}

void mt9v034_get_current_param(void *usr, camera_img_param *img_param);


static int mt9v034_init_hw(mt9v034_sensor_ctx *ctx) {
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
	I2C_InitStruct.I2C_ClockSpeed = 100000;

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
	uint16_t version = mt9v034_ReadReg16(MTV_CHIP_VERSION_REG);
	if (version != 0x1324) return -1;
	
	return 0;
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
	mt9v034_WriteReg16(MTV_CHIP_CONTROL_REG, 0x0188);

	/* settings that are the same for both contexts: */
	
	/*
	 * ADC Companding Mode
	 * [1:0] ADC Mode Context A:        2 = 10-bit linear
	 * [9:8] ADC Mode Context B:        2 = 10-bit linear
	 */
	mt9v034_WriteReg16(MTV_ADC_RES_CTRL_REG, 0x0202);
	
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
	mt9v034_WriteReg16(MTV_ROW_NOISE_CORR_CTRL_REG, row_noise_correction);
	
	/*
	 * Minimum Coarse Shutter Width
	 * Set to minimum. (1)
	 */
	mt9v034_WriteReg16(MTV_MIN_COARSE_SW_REG, 0x0001);
	/*
	 * Maximum Coarse Shutter Width
	 */
	mt9v034_WriteReg16(MTV_MAX_COARSE_SW_REG, CONFIG_MAX_EXPOSURE_ROWS);
	/*
	 * Maximum Analog Gain
	 */
	mt9v034_WriteReg16(MTV_ANALOG_MAX_GAIN_REG, CONFIG_MAX_ANALOG_GAIN);
	
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
	mt9v034_WriteReg16(MTV_AGC_AEC_DESIRED_BIN_REG, desired_brightness);
	/*
	 * AEC Update Frequency
	 * Number of frames to skip between changes in AEC
	 * Range: 0-15
	 */
	mt9v034_WriteReg16(MTV_AEC_UPDATE_FREQ_REG, 2);
	/*
	 * AEC Low Pass Filter
	 * Range: 0-2
	 */
	mt9v034_WriteReg16(MTV_AEC_LPF_REG, 1);
	/*
	 * AGC Output Update Frequency
	 * Number of frames to skip between changes in AGC
	 * Range: 0-15
	 */
	mt9v034_WriteReg16(MTV_AGC_UPDATE_FREQ_REG, 2);
	/*
	 * AGC Low Pass Filter
	 * Range: 0-2
	 */
	mt9v034_WriteReg16(MTV_AGC_LPF_REG, 2);
	/*
	 * AGC/AEC Enable
	 *   [0] AEC Enable Ctx A:                   1 = Enable
	 *   [1] AGC Enable Ctx A:                   1 = Enable
	 *   [8] AEC Enable Ctx B:                   1 = Enable
	 *   [9] AGC Enable Ctx B:                   1 = Enable
	 */
	mt9v034_WriteReg16(MTV_AEC_AGC_ENABLE_REG, 0x0303);
	/*
	 * Sensor Type Control
	 *   [0] HDR Enable Ctx A:                   0 = Disable
	 *   [1] Color/Mono Sensor Control:          0 = Monochrome
	 *   [8] HDR Enable Ctx B:                   0 = Disable
	 */
	mt9v034_WriteReg16(MTV_SENSOR_TYPE_CTRL_REG, 0x0000);
	
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
	mt9v034_WriteReg16(MTV_TEST_PATTERN_REG, test_data);//enable test pattern

	/* Reset */
	mt9v034_WriteReg16(MTV_SOFT_RESET_REG, 0x01);
	
	/*
	 * NOTES:
	 * Old code unexpectedly used:
	 *  - 12 to 10bit companding mode on 64x64 pixel image.
	 *  - Enabled AGC & AEC only on 64x64pixel image.
	 *  - Enabled HDR mode on big image.
	 *  - Used 64 x 64 = 4096 Pixels for AGC & AEC
	 */
}

static void mt9v034_configure_context(mt9v034_sensor_ctx *ctx, int context_idx, const camera_img_param *img_param) {
	/* image dimensions */
	uint16_t new_width_context_a  = global_data.param[PARAM_IMAGE_WIDTH] * 4; // windowing off, row + col bin reduce size
	uint16_t new_height_context_a = global_data.param[PARAM_IMAGE_HEIGHT] * 4;
	uint16_t new_width_context_b  = FULL_IMAGE_ROW_SIZE * 4; // windowing off, row + col bin reduce size
	uint16_t new_height_context_b = FULL_IMAGE_COLUMN_SIZE * 4;

	/* blanking settings */
	uint16_t new_hor_blanking_context_a = 350 + MINIMUM_HORIZONTAL_BLANKING;// 350 is minimum value without distortions
	uint16_t new_ver_blanking_context_a = 10; // this value is the first without image errors (dark lines)
	uint16_t new_hor_blanking_context_b = MAX_IMAGE_WIDTH - new_width_context_b + MINIMUM_HORIZONTAL_BLANKING;
	uint16_t new_ver_blanking_context_b = 10;

	/* Read Mode
	 *
	 * bits           | ... | 10 | 9 | 8 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
	 * -------------------------------------------------------------------
	 * current values | ... |  0 | 1 | 1 | 0 | 0 | 0 | 0 | 1 | 0 | 1 | 0 |
	 *
	 * (1:0) Row Bin
	 * (3:2) Column Bin
	 * (9:8) Reserved
	 *
	 */
	uint16_t new_readmode_context_a = 0x30A ; // row + col bin 4 enable, (9:8) default
	uint16_t new_readmode_context_b = 0x305 ; // row bin 2 col bin 2 enable, (9:8) default

	/*
	 * Settings for both context:
	 *
	 * Exposure time should not affect frame time
	 * so we set max on 64 (lines) = 0x40
	 */
	uint16_t min_exposure = 0x0001; // default
	uint16_t max_exposure = 0x01E0; // default

	uint16_t coarse_sw1 = 0x01BB; // default from context A
	uint16_t coarse_sw2 = 0x01D9; // default from context A
	uint16_t shutter_width_ctrl = 0x0164; // default from context A
	uint16_t total_shutter_width = 0x01E0; // default from context A

	min_exposure = 0x0001;
	max_exposure = 0x0040;
	coarse_sw1 = 0x01BB; // default from context A
	coarse_sw2 = 0x01D9; // default from context A
	shutter_width_ctrl = 0x0164; // default from context A
	total_shutter_width = 0x01E0; // default from context A
	
	/* Context A */
	mt9v034_WriteReg16(MTV_WINDOW_WIDTH_REG_A, new_width_context_a);
	mt9v034_WriteReg16(MTV_WINDOW_HEIGHT_REG_A, new_height_context_a);
	mt9v034_WriteReg16(MTV_HOR_BLANKING_REG_A, new_hor_blanking_context_a);
	mt9v034_WriteReg16(MTV_VER_BLANKING_REG_A, new_ver_blanking_context_a);
	mt9v034_WriteReg16(MTV_READ_MODE_REG_A, new_readmode_context_a);
	mt9v034_WriteReg16(MTV_COLUMN_START_REG_A, (MAX_IMAGE_WIDTH - new_width_context_a) / 2 + MINIMUM_COLUMN_START); // Set column/row start point for lower resolutions (center window)
	mt9v034_WriteReg16(MTV_ROW_START_REG_A, (MAX_IMAGE_HEIGHT - new_height_context_a) / 2 + MINIMUM_ROW_START);
	mt9v034_WriteReg16(MTV_COARSE_SW_1_REG_A, coarse_sw1);
	mt9v034_WriteReg16(MTV_COARSE_SW_2_REG_A, coarse_sw2);
	mt9v034_WriteReg16(MTV_COARSE_SW_CTRL_REG_A, shutter_width_ctrl);
	mt9v034_WriteReg16(MTV_COARSE_SW_TOTAL_REG_A, total_shutter_width);

	/* Context B */
	mt9v034_WriteReg16(MTV_WINDOW_WIDTH_REG_B, new_width_context_b);
	mt9v034_WriteReg16(MTV_WINDOW_HEIGHT_REG_B, new_height_context_b);
	mt9v034_WriteReg16(MTV_HOR_BLANKING_REG_B, new_hor_blanking_context_b);
	mt9v034_WriteReg16(MTV_VER_BLANKING_REG_B, new_ver_blanking_context_b);
	mt9v034_WriteReg16(MTV_READ_MODE_REG_B, new_readmode_context_b);
	mt9v034_WriteReg16(MTV_COLUMN_START_REG_B, MINIMUM_COLUMN_START); // default
	mt9v034_WriteReg16(MTV_ROW_START_REG_B, MINIMUM_ROW_START);
	mt9v034_WriteReg16(MTV_COARSE_SW_1_REG_B, coarse_sw1);
	mt9v034_WriteReg16(MTV_COARSE_SW_2_REG_B, coarse_sw2);
	mt9v034_WriteReg16(MTV_COARSE_SW_CTRL_REG_B, shutter_width_ctrl);
	mt9v034_WriteReg16(MTV_COARSE_SW_TOTAL_REG_B, total_shutter_width);
}

/**
  * @brief  Writes a byte at a specific Camera register
  * @param  Addr: mt9v034 register address.
  * @param  Data: Data to be written to the specific register
  * @retval 0x00 if write operation is OK.
  *       0xFF if timeout condition occured (device not connected or bus error).
  */
uint8_t mt9v034_WriteReg(uint16_t Addr, uint8_t Data)
{
  uint32_t timeout = TIMEOUT_MAX;

  /* Generate the Start Condition */
  I2C_GenerateSTART(I2C2, ENABLE);

  /* Test on I2C2 EV5 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
  {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Send DCMI selected device slave Address for write */
  I2C_Send7bitAddress(I2C2, mt9v034_DEVICE_WRITE_ADDRESS, I2C_Direction_Transmitter);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Send I2C2 location address LSB */
  I2C_SendData(I2C2, (uint8_t)(Addr));

  /* Test on I2C2 EV8 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Send Data */
  I2C_SendData(I2C2, Data);

  /* Test on I2C2 EV8 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Send I2C2 STOP Condition */
  I2C_GenerateSTOP(I2C2, ENABLE);

  /* If operation is OK, return 0 */
  return 0;
}

/**
  * @brief  Writes to a specific Camera register
  */
uint8_t mt9v034_WriteReg16(uint16_t address, uint16_t Data)
{
	uint8_t result = mt9v034_WriteReg(address, (uint8_t)( Data >> 8)); // write upper byte
	result |= mt9v034_WriteReg(0xF0, (uint8_t) Data); // write lower byte
	return result;
}

/**
  * @brief  Reads a byte from a specific Camera register
  * @param  Addr: mt9v034 register address.
  * @retval data read from the specific register or 0xFF if timeout condition
  *         occurred.
  */
uint8_t mt9v034_ReadReg(uint16_t Addr)
{
  uint32_t timeout = TIMEOUT_MAX;
  uint8_t Data = 0;

  /* Generate the Start Condition */
  I2C_GenerateSTART(I2C2, ENABLE);

  /* Test on I2C2 EV5 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
  {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Send DCMI selected device slave Address for write */
  I2C_Send7bitAddress(I2C2, mt9v034_DEVICE_READ_ADDRESS, I2C_Direction_Transmitter);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Send I2C2 location address LSB */
  I2C_SendData(I2C2, (uint8_t)(Addr));

  /* Test on I2C2 EV8 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Clear AF flag if set */
  I2C2->SR1 |= (uint16_t)0x0400;

  /* Generate the Start Condition */
  I2C_GenerateSTART(I2C2, ENABLE);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
  {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Send DCMI selected device slave Address for write */
  I2C_Send7bitAddress(I2C2, mt9v034_DEVICE_READ_ADDRESS, I2C_Direction_Receiver);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
  {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Prepare an NACK for the next data received */
  I2C_AcknowledgeConfig(I2C2, DISABLE);

  /* Test on I2C2 EV7 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED))
  {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Prepare Stop after receiving data */
  I2C_GenerateSTOP(I2C2, ENABLE);

  /* Receive the Data */
  Data = I2C_ReceiveData(I2C2);

  /* return the read data */
  return Data;
}

/**
  * @brief  Reads from a specific Camera register
  */
uint16_t mt9v034_ReadReg16(uint8_t address)
{
	uint16_t result = mt9v034_ReadReg(address) << 8; // read upper byte
	result |= mt9v034_ReadReg(0xF0); // read lower byte
	return result;
}
