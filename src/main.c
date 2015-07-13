/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Laurens Mackay <mackayl@student.ethz.ch>
 *   		 Dominik Honegger <dominik.honegger@inf.ethz.ch>
 *   		 Petri Tanskanen <tpetri@inf.ethz.ch>
 *   		 Samuel Zihlmann <samuezih@ee.ethz.ch>
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

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"

#include "mavlink_bridge_header.h"
#include <mavlink.h>
#include "settings.h"
#include "utils.h"
#include "led.h"
#include "filter.h"
#include "result_accumulator.h"
#include "quality_measurement.h"
#include "flow.h"
#include "timer.h"
#include "dcmi.h"
#include "mt9v034.h"
#include "gyro.h"
#include "i2c.h"
#include "usart.h"
#include "sonar.h"
#include "communication.h"
#include "debug.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"
#include "main.h"

/* coprocessor control register (fpu) */
#ifndef SCB_CPACR
#define SCB_CPACR (*((uint32_t*) (((0xE000E000UL) + 0x0D00UL) + 0x088)))
#endif


/* prototypes */
void buffer_reset(void);

__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

/* fast image buffers for calculations */
uint8_t* image_buffer_8bit_1 = ((uint8_t*) 0x10000000);
uint8_t* image_buffer_8bit_2 = ((uint8_t*) ( 0x10000000 | FULL_IMAGE_SIZE ));
uint8_t buffer_reset_needed;

/* timer constants */
#define SONAR_POLL_MS	 	100	/* steps in milliseconds ticks */
#define SYSTEM_STATE_MS		1000/* steps in milliseconds ticks */
#define PARAMS_MS			100	/* steps in milliseconds ticks */
#define LPOS_TIMER_COUNT 	100	/* steps in milliseconds ticks */

static uint8_t *current_image;
static uint8_t *previous_image;

/* local position estimate without orientation, useful for unit testing w/o FMU */
struct lpos_t {
	float x;
	float y;
	float z;
	float vx;
	float vy;
	float vz;
} lpos = {0};


void sonar_update_fn(void) {
	sonar_trigger();
}

void system_state_send_fn(void) {
	/* every second */
	if (global_data.param[PARAM_SYSTEM_SEND_STATE])
	{
		communication_system_state_send();
	}
}

void system_receive_fn(void) {
	/* test every 0.5s */
	communication_receive();
	communication_receive_usb();
}

void send_video_fn(void) {
	/*  transmit raw 8-bit image */
	if (global_data.param[PARAM_USB_SEND_VIDEO])
	{
		/* get size of image to send */
		uint16_t image_size_send;
		uint16_t image_width_send;
		uint16_t image_height_send;

		uint16_t image_size = global_data.param[PARAM_IMAGE_WIDTH] * global_data.param[PARAM_IMAGE_HEIGHT];

		image_size_send = image_size;
		image_width_send = global_data.param[PARAM_IMAGE_WIDTH];
		image_height_send = global_data.param[PARAM_IMAGE_HEIGHT];

		mavlink_msg_data_transmission_handshake_send(
				MAVLINK_COMM_2,
				MAVLINK_DATA_STREAM_IMG_RAW8U,
				image_size_send,
				image_width_send,
				image_height_send,
				image_size_send / MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN + 1,
				MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN,
				100);
		LEDToggle(LED_COM);
		uint16_t frame = 0;
		for (frame = 0; frame < image_size_send / MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN + 1; frame++)
		{
			mavlink_msg_encapsulated_data_send(MAVLINK_COMM_2, frame, &((uint8_t *) previous_image)[frame * MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN]);
		}
	}
	else if (!global_data.param[PARAM_USB_SEND_VIDEO])
	{
		LEDOff(LED_COM);
	}
}

void send_params_fn(void) {
	debug_message_send_one();
	communication_parameter_send();
}

void buffer_reset(void) {
	buffer_reset_needed = 1;
}

void enter_image_calibration_mode() {
	while(global_data.param[PARAM_VIDEO_ONLY])
	{
		dcmi_restart_calibration_routine();

		/* waiting for first quarter of image */
		while(get_frame_counter() < 2){}
		dma_copy_image_buffers(&current_image, &previous_image, FULL_IMAGE_SIZE, 1);

		/* waiting for second quarter of image */
		while(get_frame_counter() < 3){}
		dma_copy_image_buffers(&current_image, &previous_image, FULL_IMAGE_SIZE, 1);

		/* waiting for all image parts */
		while(get_frame_counter() < 4){}

		send_calibration_image(&previous_image, &current_image);

		/* check timers */
		timer_check();

		LEDToggle(LED_COM);
	}

	dcmi_restart_calibration_routine();
	LEDOff(LED_COM);
}

/**
  * @brief  Main function.
  */
int main(void)
{
	current_image  = image_buffer_8bit_1;
	previous_image = image_buffer_8bit_2;

	/* load settings and parameters */
	global_data_reset_param_defaults();
	global_data_reset();

	/* init led */
	LEDInit(LED_ACT);
	LEDInit(LED_COM);
	LEDInit(LED_ERR);
	LEDOff(LED_ACT);
	LEDOff(LED_COM);
	LEDOff(LED_ERR);

	/* enable FPU on Cortex-M4F core */
	SCB_CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 Full Access and set CP11 Full Access */

	/* init timers */
	timer_init();

	/* init usb */
	USBD_Init(	&USB_OTG_dev,
				USB_OTG_FS_CORE_ID,
				&USR_desc,
				&USBD_CDC_cb,
				&USR_cb);

	/* init mavlink */
	communication_init();

	/* enable image capturing */
	enable_image_capture();

	/* gyro config */
	gyro_config();

	/* init and clear fast image buffers */
	for (int i = 0; i < global_data.param[PARAM_IMAGE_WIDTH] * global_data.param[PARAM_IMAGE_HEIGHT]; i++)
	{
		image_buffer_8bit_1[i] = 0;
		image_buffer_8bit_2[i] = 0;
	}

	/* usart config*/
	usart_init();

    /* i2c config*/
    i2c_init();

	/* sonar config*/
	float sonar_distance_filtered = 0.0f; // distance in meter
	float sonar_distance_raw = 0.0f; // distance in meter
	bool distance_valid = false;
	sonar_config();

	/* reset/start timers */
	timer_register(sonar_update_fn, SONAR_POLL_MS);
	timer_register(system_state_send_fn, SYSTEM_STATE_MS);
	timer_register(system_receive_fn, SYSTEM_STATE_MS / 2);
	timer_register(send_params_fn, PARAMS_MS);
	timer_register(send_video_fn, global_data.param[PARAM_VIDEO_RATE]);

	/* variables */
	uint32_t counter = 0;

	result_accumulator_ctx mavlink_accumulator;

	result_accumulator_init(&mavlink_accumulator);
	
	uint32_t fps_timing_start = get_boot_time_us();
	uint16_t fps_counter = 0;
	uint16_t fps_skipped_counter = 0;
	/* main loop */
	while (1)
	{
		/* check timers */
		timer_check();

		/* reset flow buffers if needed */
		if(buffer_reset_needed)
		{
			buffer_reset_needed = 0;
			/* get two new fresh (full) images: (or 8 small images ..) */
			dma_copy_image_buffers(&current_image, &previous_image, FULL_IMAGE_SIZE, 4);
			dma_copy_image_buffers(&current_image, &previous_image, FULL_IMAGE_SIZE, 4);
			continue;
		}

		/* calibration routine */
		if(global_data.param[PARAM_VIDEO_ONLY])
		{
			enter_image_calibration_mode();
			continue;
		}

		uint16_t image_size = global_data.param[PARAM_IMAGE_WIDTH] * global_data.param[PARAM_IMAGE_HEIGHT];

		/* calculate focal_length in pixel */
		const float focal_length_px = (global_data.param[PARAM_FOCAL_LENGTH_MM]) / (4.0f * 0.006f); //original focal lenght: 12mm pixelsize: 6um, binning 4 enabled

		/* new gyroscope data */
		float x_rate_sensor, y_rate_sensor, z_rate_sensor;
		int16_t gyro_temp;
		gyro_read(&x_rate_sensor, &y_rate_sensor, &z_rate_sensor,&gyro_temp);

		/* gyroscope coordinate transformation to flow sensor coordinates */
		float x_rate =   y_rate_sensor; // change x and y rates
		float y_rate = - x_rate_sensor;
		float z_rate =   z_rate_sensor; // z is correct

		/* get sonar data */
		distance_valid = sonar_read(&sonar_distance_filtered, &sonar_distance_raw);
		/* reset to zero for invalid distances */
		if (!distance_valid) {
			sonar_distance_filtered = 0.0f;
			sonar_distance_raw = 0.0f;
		}

		bool use_klt = global_data.param[PARAM_ALGORITHM_CHOICE] != 0;

		uint32_t start_computations = 0;

		/* copy recent image to faster ram */
		int skipped_frames = 0;
		int loop_count = 0;		//< make sure that we dont end up in an infinite loop inside here ..
		do { 
			skipped_frames = dma_copy_image_buffers(&current_image, &previous_image, image_size, (int)(global_data.param[PARAM_FRAME_INTERVAL] + 0.5));

			start_computations = get_boot_time_us();

			/* filter the new image */
			if (global_data.param[PARAM_USE_IMAGE_FILTER]) {
				filter_image(current_image, global_data.param[PARAM_IMAGE_WIDTH]);
			}

			/* Preprocessing needed by the klt algorithm: */
			if (use_klt) {
				klt_preprocess_image(current_image);
			}
			/* make sure no frames have been skipped. request a new image if a frame was skipped to ensure minimal time delta between images */
			fps_skipped_counter += skipped_frames;
			loop_count++;
		} while (skipped_frames > 0 && loop_count < 2);
		float frame_dt = get_time_between_images() * 0.000001f;

		/* compute gyro rate in pixels and change to image coordinates */
		float x_rate_px = - y_rate * (focal_length_px * frame_dt);
		float y_rate_px =   x_rate * (focal_length_px * frame_dt);
		float z_rate_fr = - z_rate * frame_dt;

		/* compute optical flow in pixels */
		flow_raw_result flow_rslt[32];
		uint16_t flow_rslt_count = 0;
		if (!use_klt) {
			flow_rslt_count = compute_flow(previous_image, current_image, x_rate_px, y_rate_px, z_rate_fr, flow_rslt, 32);
		} else {
			flow_rslt_count =  compute_klt(previous_image, current_image, x_rate_px, y_rate_px, z_rate_fr, flow_rslt, 32);
		}

		/* calculate flow value from the raw results */
		float pixel_flow_x;
		float pixel_flow_y;
		float outlier_threshold = global_data.param[PARAM_OUTLIER_THRESHOLD];
		float min_outlier_threshold = 0;
		if(global_data.param[PARAM_ALGORITHM_CHOICE] == 0)
		{
			min_outlier_threshold = global_data.param[PARAM_MINIMUM_OUTLIER_THRESHOLD_AlGO_0];
		}else
		{
			min_outlier_threshold = global_data.param[PARAM_MINIMUM_OUTLIER_THRESHOLD_AlGO_1];
		}
		uint8_t qual = flow_extract_result(flow_rslt, flow_rslt_count, &pixel_flow_x, &pixel_flow_y, 
							outlier_threshold,  min_outlier_threshold);

		/* create flow image if needed (previous_image is not needed anymore)
		 * -> can be used for debugging purpose
		 */
		if (global_data.param[PARAM_USB_SEND_VIDEO])
		{
			uint16_t frame_size = global_data.param[PARAM_IMAGE_WIDTH];
			for (int i = 0; i < flow_rslt_count; i++) {
				if (flow_rslt[i].quality > 0) {
					previous_image[flow_rslt[i].at_y * frame_size + flow_rslt[i].at_x] = 255;
					int ofs = (int)floor(flow_rslt[i].at_y + flow_rslt[i].y * 2 + 0.5f) * frame_size + (int)floor(flow_rslt[i].at_x + flow_rslt[i].x * 2 + 0.5f);
					if (ofs >= 0 && ofs < frame_size * frame_size) {
						previous_image[ofs] = 200;
					}
				}
			}
		}

		/* decide which distance to use */
		float ground_distance = 0.0f;

		if(global_data.param[PARAM_SONAR_FILTERED])
		{
			ground_distance = sonar_distance_filtered;
		}
		else
		{
			ground_distance = sonar_distance_raw;
		}

		/* update I2C transmitbuffer */
		update_TX_buffer(frame_dt, 
						 x_rate, y_rate, z_rate, gyro_temp, 
						 qual, pixel_flow_x, pixel_flow_y, 1.0f / focal_length_px, 
						 distance_valid, ground_distance, get_time_delta_us(get_sonar_measure_time()));

		/* accumulate the results */
		result_accumulator_feed(&mavlink_accumulator, frame_dt, 
								x_rate, y_rate, z_rate, gyro_temp, 
								qual, pixel_flow_x, pixel_flow_y, 1.0f / focal_length_px, 
								distance_valid, ground_distance, get_time_delta_us(get_sonar_measure_time()));

		uint32_t computaiton_time_us = get_time_delta_us(start_computations);

		counter++;
		fps_counter++;

        /* serial mavlink  + usb mavlink output throttled */
		if (counter % (uint32_t)global_data.param[PARAM_BOTTOM_FLOW_SERIAL_THROTTLE_FACTOR] == 0)//throttling factor
		{
			float fps = 0;
			float fps_skip = 0;
			if (fps_counter + fps_skipped_counter > 100) {
				uint32_t dt = get_time_delta_us(fps_timing_start);
				fps_timing_start += dt;
				fps = (float)fps_counter / ((float)dt * 1e-6f);
				fps_skip = (float)fps_skipped_counter / ((float)dt * 1e-6f);
				fps_counter = 0;
				fps_skipped_counter = 0;

				mavlink_msg_debug_vect_send(MAVLINK_COMM_2, "TIMING", get_boot_time_us(), computaiton_time_us, fps, fps_skip);
			}
			
			/* recalculate the output values */
			result_accumulator_output_flow output_flow;
			result_accumulator_output_flow_rad output_flow_rad;
			int min_valid_ratio = global_data.param[PARAM_MIN_VALID_RATIO];
			result_accumulator_calculate_output_flow(&mavlink_accumulator, min_valid_ratio, &output_flow);
			result_accumulator_calculate_output_flow_rad(&mavlink_accumulator, min_valid_ratio, &output_flow_rad);

			// send flow
			mavlink_msg_optical_flow_send(MAVLINK_COMM_0, get_boot_time_us(), global_data.param[PARAM_SENSOR_ID],
					output_flow.flow_x, output_flow.flow_y,
					output_flow.flow_comp_m_x, output_flow.flow_comp_m_y, 
					output_flow.quality, output_flow.ground_distance);

			mavlink_msg_optical_flow_rad_send(MAVLINK_COMM_0, get_boot_time_us(), global_data.param[PARAM_SENSOR_ID],
					output_flow_rad.integration_time, 
					output_flow_rad.integrated_x, output_flow_rad.integrated_y,
					output_flow_rad.integrated_xgyro, output_flow_rad.integrated_ygyro, output_flow_rad.integrated_zgyro,
					output_flow_rad.temperature, output_flow_rad.quality,
					output_flow_rad.time_delta_distance_us,output_flow_rad.ground_distance);

			if (global_data.param[PARAM_USB_SEND_FLOW] && (output_flow.quality > 0 || global_data.param[PARAM_USB_SEND_QUAL_0]))
			{
				mavlink_msg_optical_flow_send(MAVLINK_COMM_2, get_boot_time_us(), global_data.param[PARAM_SENSOR_ID],
						output_flow.flow_x, output_flow.flow_y,
						output_flow.flow_comp_m_x, output_flow.flow_comp_m_y, 
						output_flow.quality, output_flow.ground_distance);

				mavlink_msg_optical_flow_rad_send(MAVLINK_COMM_2, get_boot_time_us(), global_data.param[PARAM_SENSOR_ID],
						output_flow_rad.integration_time, 
						output_flow_rad.integrated_x, output_flow_rad.integrated_y,
						output_flow_rad.integrated_xgyro, output_flow_rad.integrated_ygyro, output_flow_rad.integrated_zgyro,
						output_flow_rad.temperature, output_flow_rad.quality,
						output_flow_rad.time_delta_distance_us,output_flow_rad.ground_distance);
			}

			if(global_data.param[PARAM_USB_SEND_GYRO])
			{
				mavlink_msg_debug_vect_send(MAVLINK_COMM_2, "GYRO", get_boot_time_us(), x_rate, y_rate, z_rate);
			}

			result_accumulator_reset(&mavlink_accumulator);
		}

		/* forward flow from other sensors */
		if (counter % 2)
		{
			communication_receive_forward();
		}
	}
}
