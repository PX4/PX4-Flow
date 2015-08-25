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

float param_system_id = 81;
float param_component_id = 50;
float param_sensor_id = 77;
float param_system_type = MAV_TYPE_GENERIC;
float param_autopilot_type = MAV_AUTOPILOT_GENERIC;
float param_sw_version  = 1300;
float param_system_send_state = 1;
float param_system_send_lpos = 0;
float param_usart2_baud = 115200;
float param_usart3_baud = 115200;
float param_focal_length_mm = 16.0f;
float param_image_width = BOTTOM_FLOW_IMAGE_WIDTH;
float param_image_height = BOTTOM_FLOW_IMAGE_HEIGHT;
float param_max_flow_pixel = BOTTOM_FLOW_SEARCH_WINDOW_SIZE;
float param_image_low_light = 0;
float param_image_row_noise_corr = 1;
float param_image_test_pattern = 0;
float param_gyro_sensitivity_dps = 250;
float param_gyro_compensation_threshold = 0.01;
float param_sonar_filtered = 0;
float param_sonar_kalman_l1 = 0.8461f;
float param_sonar_kalman_l2 = 6.2034f;
float param_usb_send_video = 1;
float param_usb_send_flow = 1;
float param_usb_send_gyro = 1;
float param_usb_send_forward = 0;
float param_usb_send_debug = 1;
float param_video_only = 0;
float param_video_rate = 150;
float param_bottom_flow_feature_threshold = 30;
float param_bottom_flow_value_threshold = 5000;
float param_bottom_flow_hist_filter = 0;
float param_bottom_flow_gyro_compensation = 0;
float param_bottom_flow_lp_filtered = 0;
float param_bottom_flow_weight_new = 0.3f;
float param_bottom_flow_serial_throttle_factor = 10.0f;
float param_sensor_position = 0; //BOTTOM
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
	switch((int32_t) param_sensor_position) {
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
		.access = READ_WRITE,
		.variable = &param_system_id,
	},
	{ .name = "SYS_COMP_ID",
		.access = READ_WRITE,
		.variable = &param_component_id,
	},
	{ .name = "SYS_SENSOR_ID",
		.access = READ_WRITE,
		.variable = &param_sensor_id,
	},
	{ .name = "SYS_TYPE",
		.access = READ_WRITE,
		.variable = &param_system_type,
	},
	{ .name = "SYS_AP_TYPE",
		.access = READ_WRITE,
		.variable = &param_autopilot_type,
	},
	{ .name = "SYS_SW_VER",
		.access = READ_WRITE,
		.variable = &param_sw_version,
	},
	{ .name = "SYS_SEND_STATE",
		.access = READ_WRITE,
		.variable = &param_system_send_state,
	},
	{ .name = "SYS_SEND_LPOS",
		.access = READ_WRITE,
		.variable = &param_system_send_lpos,
	},
	{ .name = "POSITION",
		.access = READ_WRITE,
		.variable = &param_sensor_position,
		.on_change = on_sensor_position_change,
	},
	{ .name = "USART_2_BAUD",
		.access = READ_ONLY,
		.variable = &param_usart2_baud,
	},
	{ .name = "USART_3_BAUD",
		.access = READ_ONLY,
		.variable = &param_usart3_baud,
	},
	{ .name = "LENS_FOCAL_LEN",
		.access = READ_WRITE,
		.variable = &param_focal_length_mm,
	},
	{ .name = "IMAGE_WIDTH",
		.access = READ_ONLY,
		.variable = &param_image_width,
	},
	{ .name = "IMAGE_HEIGHT",
		.access = READ_ONLY,
		.variable = &param_image_height,
	},
	{ .name = "IMAGE_L_LIGHT",
		.access = READ_WRITE,
		.variable = &param_image_low_light,
		.on_change = reconfigure,
	},
	{ .name = "IMAGE_NOISE_C",
		.access = READ_WRITE,
		.variable = &param_image_row_noise_corr,
		.on_change = reconfigure,
	},
	{ .name = "IMAGE_TEST_PAT",
		.access = READ_WRITE,
		.variable = &param_image_test_pattern,
		.on_change = reconfigure,
	},
	{ .name = "GYRO_SENS_DPS",
		.access = READ_WRITE,
		.variable = &param_gyro_sensitivity_dps,
		.on_change = l3gd20_config,
	},
	{ .name = "GYRO_COMP_THR",
		.access = READ_WRITE,
		.variable = &param_gyro_compensation_threshold,
	},
	{ .name = "SONAR_FILTERED",
		.access = READ_WRITE,
		.variable = &param_sonar_filtered,
	},
	{ .name = "SONAR_KAL_L1",
		.access = READ_WRITE,
		.variable = &param_sonar_kalman_l1,
	},
	{ .name = "SONAR_KAL_L2",
		.access = READ_WRITE,
		.variable = &param_sonar_kalman_l2,
	},
	{ .name = "USB_SEND_VIDEO",
		.access = READ_WRITE,
		.variable = &param_usb_send_video,
	},
	{ .name = "USB_SEND_FLOW",
		.access = READ_WRITE,
		.variable = &param_usb_send_flow,
	},
	{ .name = "USB_SEND_GYRO",
		.access = READ_WRITE,
		.variable = &param_usb_send_gyro,
	},
	{ .name = "USB_SEND_FWD",
		.access = READ_WRITE,
		.variable = &param_usb_send_forward,
	},
	{ .name = "USB_SEND_DEBUG",
		.access = READ_WRITE,
		.variable = &param_usb_send_debug,
	},
	{ .name = "VIDEO_ONLY",
		.access = READ_WRITE,
		.variable = &param_video_only,
		.on_change = reconfigure,
	},
	{ .name = "VIDEO_RATE",
		.access = READ_WRITE,
		.variable = &param_video_rate,
	},
	{ .name = "BFLOW_MAX_PIX",
		.access = READ_ONLY,
		.variable = &param_max_flow_pixel,
	},
	{ .name = "BFLOW_V_THLD",
		.access = READ_WRITE,
		.variable = &param_bottom_flow_value_threshold,
	},
	{ .name = "BFLOW_F_THLD",
		.access = READ_WRITE,
		.variable = &param_bottom_flow_feature_threshold,
	},
	{ .name = "BFLOW_HIST_FIL",
		.access = READ_WRITE,
		.variable = &param_bottom_flow_hist_filter,
	},
	{ .name = "BFLOW_GYRO_COM",
		.access = READ_WRITE,
		.variable = &param_bottom_flow_gyro_compensation,
	},
	{ .name = "BFLOW_LP_FIL",
		.access = READ_WRITE,
		.variable = &param_bottom_flow_lp_filtered,
	},
	{ .name = "BFLOW_W_NEW",
		.access = READ_WRITE,
		.variable = &param_bottom_flow_weight_new,
	},
	{ .name = "BFLOW_THROTT",
		.access = READ_WRITE,
		.variable = &param_bottom_flow_serial_throttle_factor,
	},
	{ .name = "DEBUG",
		.access = READ_WRITE,
		.variable = &debug_variable,
	},
};

const uint32_t params_count = sizeof(param_info) / sizeof(*param_info);


