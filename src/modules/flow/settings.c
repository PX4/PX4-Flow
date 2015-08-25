/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Samuel Zihlmann <samuezih@ee.ethz.ch>
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

#include <mavlink.h>
#include "settings.h"
#include "mt9v034.h"
#include "dcmi.h"
#include "gyro.h"

extern uint8_t debug_int_message_buffer(const char* string, int32_t num);

uint32_t param_system_id = 81;
uint32_t param_component_id = 50;
uint32_t param_sensor_id = 77;
uint32_t param_system_type = MAV_TYPE_GENERIC;
uint32_t param_autopilot_type = MAV_AUTOPILOT_GENERIC;
uint32_t param_sw_version  = 1300;
bool param_system_send_state = 1;
bool param_system_send_lpos = 0;
uint32_t param_usart2_baud = 115200;
uint32_t param_usart3_baud = 115200;
float param_focal_length_mm = 16.0f;
uint32_t param_image_width = BOTTOM_FLOW_IMAGE_WIDTH;
uint32_t param_image_height = BOTTOM_FLOW_IMAGE_HEIGHT;
uint32_t param_max_flow_pixel = BOTTOM_FLOW_SEARCH_WINDOW_SIZE;
bool param_image_low_light = 0;
bool param_image_row_noise_corr = 1;
bool param_image_test_pattern = 0;
uint32_t param_gyro_sensitivity_dps = 250;
float param_gyro_compensation_threshold = 0.01;
bool param_sonar_filtered = 0;
float param_sonar_kalman_l1 = 0.8461f;
float param_sonar_kalman_l2 = 6.2034f;
bool param_usb_send_video = 1;
bool param_usb_send_flow = 1;
bool param_usb_send_gyro = 1;
bool param_usb_send_forward = 0;
bool param_usb_send_debug = 1;
bool param_video_only = 0;
float param_video_rate = 150;
float param_bottom_flow_feature_threshold = 30;
float param_bottom_flow_value_threshold = 5000;
bool param_bottom_flow_hist_filter = 0;
bool param_bottom_flow_gyro_compensation = 0;
bool param_bottom_flow_lp_filtered = 0;
float param_bottom_flow_weight_new = 0.3f;
float param_bottom_flow_serial_throttle_factor = 10.0f;
uint32_t param_sensor_position = 0; //BOTTOM
float debug_variable = 0;

extern void buffer_reset(void);
static void reconfigure(void) {
	mt9v034_context_configuration();
	dma_reconfigure();
	buffer_reset();
}

/**
 * @brief changes read only settings depending on sensor position
 */
static void on_sensor_position_change(void)
{
	switch(param_sensor_position) {
		case(BOTTOM):
			param_image_width = BOTTOM_FLOW_IMAGE_WIDTH;
			param_image_height = BOTTOM_FLOW_IMAGE_HEIGHT;
			break;

		default:
			debug_int_message_buffer("Unused sensor position:", param_sensor_position);
			return;
	}

	debug_int_message_buffer("Set sensor position:", param_sensor_position);
	reconfigure();
}


const param_info_item param_info[] = {
	{ .name = "SYS_ID",
		.type = PARAM_UINT,
		.access = READ_WRITE,
		.variable = &param_system_id,
	},
	{ .name = "SYS_COMP_ID",
		.type = PARAM_UINT,
		.access = READ_WRITE,
		.variable = &param_component_id,
	},
	{ .name = "SYS_SENSOR_ID",
		.type = PARAM_UINT,
		.access = READ_WRITE,
		.variable = &param_sensor_id,
	},
	{ .name = "SYS_TYPE",
		.type = PARAM_UINT,
		.access = READ_WRITE,
		.variable = &param_system_type,
	},
	{ .name = "SYS_AP_TYPE",
		.type = PARAM_UINT,
		.access = READ_WRITE,
		.variable = &param_autopilot_type,
	},
	{ .name = "SYS_SW_VER",
		.type = PARAM_UINT,
		.access = READ_WRITE,
		.variable = &param_sw_version,
	},
	{ .name = "SYS_SEND_STATE",
		.type = PARAM_BOOL,
		.access = READ_WRITE,
		.variable = &param_system_send_state,
	},
	{ .name = "SYS_SEND_LPOS",
		.type = PARAM_BOOL,
		.access = READ_WRITE,
		.variable = &param_system_send_lpos,
	},
	{ .name = "POSITION",
		.type = PARAM_UINT,
		.access = READ_WRITE,
		.variable = &param_sensor_position,
		.on_change = on_sensor_position_change,
	},
	{ .name = "USART_2_BAUD",
		.type = PARAM_UINT,
		.access = READ_ONLY,
		.variable = &param_usart2_baud,
	},
	{ .name = "USART_3_BAUD",
		.type = PARAM_UINT,
		.access = READ_ONLY,
		.variable = &param_usart3_baud,
	},
	{ .name = "LENS_FOCAL_LEN",
		.type = PARAM_FLOAT,
		.access = READ_WRITE,
		.variable = &param_focal_length_mm,
	},
	{ .name = "IMAGE_WIDTH",
		.type = PARAM_UINT,
		.access = READ_ONLY,
		.variable = &param_image_width,
	},
	{ .name = "IMAGE_HEIGHT",
		.type = PARAM_UINT,
		.access = READ_ONLY,
		.variable = &param_image_height,
	},
	{ .name = "IMAGE_L_LIGHT",
		.type = PARAM_BOOL,
		.access = READ_WRITE,
		.variable = &param_image_low_light,
		.on_change = reconfigure,
	},
	{ .name = "IMAGE_NOISE_C",
		.type = PARAM_BOOL,
		.access = READ_WRITE,
		.variable = &param_image_row_noise_corr,
		.on_change = reconfigure,
	},
	{ .name = "IMAGE_TEST_PAT",
		.type = PARAM_BOOL,
		.access = READ_WRITE,
		.variable = &param_image_test_pattern,
		.on_change = reconfigure,
	},
	{ .name = "GYRO_SENS_DPS",
		.type = PARAM_UINT,
		.access = READ_WRITE,
		.variable = &param_gyro_sensitivity_dps,
		.on_change = l3gd20_config,
	},
	{ .name = "GYRO_COMP_THR",
		.type = PARAM_FLOAT,
		.access = READ_WRITE,
		.variable = &param_gyro_compensation_threshold,
	},
	{ .name = "SONAR_FILTERED",
		.type = PARAM_BOOL,
		.access = READ_WRITE,
		.variable = &param_sonar_filtered,
	},
	{ .name = "SONAR_KAL_L1",
		.type = PARAM_FLOAT,
		.access = READ_WRITE,
		.variable = &param_sonar_kalman_l1,
	},
	{ .name = "SONAR_KAL_L2",
		.type = PARAM_FLOAT,
		.access = READ_WRITE,
		.variable = &param_sonar_kalman_l2,
	},
	{ .name = "USB_SEND_VIDEO",
		.type = PARAM_BOOL,
		.access = READ_WRITE,
		.variable = &param_usb_send_video,
	},
	{ .name = "USB_SEND_FLOW",
		.type = PARAM_BOOL,
		.access = READ_WRITE,
		.variable = &param_usb_send_flow,
	},
	{ .name = "USB_SEND_GYRO",
		.type = PARAM_BOOL,
		.access = READ_WRITE,
		.variable = &param_usb_send_gyro,
	},
	{ .name = "USB_SEND_FWD",
		.type = PARAM_BOOL,
		.access = READ_WRITE,
		.variable = &param_usb_send_forward,
	},
	{ .name = "USB_SEND_DEBUG",
		.type = PARAM_BOOL,
		.access = READ_WRITE,
		.variable = &param_usb_send_debug,
	},
	{ .name = "VIDEO_ONLY",
		.type = PARAM_BOOL,
		.access = READ_WRITE,
		.variable = &param_video_only,
		.on_change = reconfigure,
	},
	{ .name = "VIDEO_RATE",
		.type = PARAM_FLOAT,
		.access = READ_WRITE,
		.variable = &param_video_rate,
	},
	{ .name = "BFLOW_MAX_PIX",
		.type = PARAM_UINT,
		.access = READ_ONLY,
		.variable = &param_max_flow_pixel,
	},
	{ .name = "BFLOW_V_THLD",
		.type = PARAM_FLOAT,
		.access = READ_WRITE,
		.variable = &param_bottom_flow_value_threshold,
	},
	{ .name = "BFLOW_F_THLD",
		.type = PARAM_FLOAT,
		.access = READ_WRITE,
		.variable = &param_bottom_flow_feature_threshold,
	},
	{ .name = "BFLOW_HIST_FIL",
		.type = PARAM_BOOL,
		.access = READ_WRITE,
		.variable = &param_bottom_flow_hist_filter,
	},
	{ .name = "BFLOW_GYRO_COM",
		.type = PARAM_BOOL,
		.access = READ_WRITE,
		.variable = &param_bottom_flow_gyro_compensation,
	},
	{ .name = "BFLOW_LP_FIL",
		.type = PARAM_BOOL,
		.access = READ_WRITE,
		.variable = &param_bottom_flow_lp_filtered,
	},
	{ .name = "BFLOW_W_NEW",
		.type = PARAM_FLOAT,
		.access = READ_WRITE,
		.variable = &param_bottom_flow_weight_new,
	},
	{ .name = "BFLOW_THROTT",
		.type = PARAM_FLOAT,
		.access = READ_WRITE,
		.variable = &param_bottom_flow_serial_throttle_factor,
	},
	{ .name = "DEBUG",
		.type = PARAM_FLOAT,
		.access = READ_WRITE,
		.variable = &debug_variable,
	},
};

const uint32_t params_count = sizeof(param_info) / sizeof(*param_info);


