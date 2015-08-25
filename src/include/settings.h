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

#ifndef SETTINGS_H_
#define SETTINGS_H_

#include <stdint.h>
#include <stdbool.h>

#define BOTTOM_FLOW_IMAGE_HEIGHT		64
#define BOTTOM_FLOW_IMAGE_WIDTH			64
#define BOTTOM_FLOW_SEARCH_WINDOW_SIZE 	4

/******************************************************************
  * ALL TYPE DEFINITIONS
  */

/**
  * @brief  parameter access
  */
typedef enum
{
  READ_ONLY   = 0,
  READ_WRITE  = 1,
} ParameterAccess_TypeDef;

typedef enum
{
  PARAM_FLOAT,
  PARAM_BOOL,
  PARAM_UINT,
} ParameterType_TypeDef;

/**
  * @brief  sensor position enumeration
  */
typedef enum
{
  BOTTOM = 0x00, 	/*!< at bottom position */
  FRONT  = 0x01, 	/*!< at front position */
  TOP    = 0x02, 	/*!< at top position */
  BACK   = 0x03,  	/*!< at back position */
  RIGHT  = 0x04,  	/*!< at right position */
  LEFT   = 0x05  	/*!< at left position */
} SensorPosition_TypeDef;

/**
  * @brief  sensor position enumeration
  */
typedef enum
{
  NO_VIDEO   = 0,
  CAM_VIDEO  = 1,
  FLOW_VIDEO = 2,
} VideoStreamMode_TypeDef;

/**
  * @brief  gyro sensitivity enumeration
  */
typedef enum
{
  DPS250  = 250, 	/*!< 250 dps */
  DPS500  = 500, 	/*!< 500 dps */
  DPS2000 = 2000	/*!< 2000 dps */
} GyroSensitivity_TypeDef;

/******************************************************************
  * ALL SETTINGS VARIABLES
  */
extern uint32_t param_system_id;
extern uint32_t param_component_id;
extern uint32_t param_sensor_id;
extern uint32_t param_system_type;
extern uint32_t param_autopilot_type;
extern uint32_t param_sw_version;
extern bool param_system_send_state;
extern bool param_system_send_lpos;
extern uint32_t param_usart2_baud;
extern uint32_t param_usart3_baud;
extern float param_focal_length_mm;
extern uint32_t param_image_width;
extern uint32_t param_image_height;
extern uint32_t param_max_flow_pixel;
extern bool param_image_low_light;
extern bool param_image_row_noise_corr;
extern bool param_image_test_pattern;
extern uint32_t param_gyro_sensitivity_dps;
extern float param_gyro_compensation_threshold;
extern bool param_sonar_filtered;
extern float param_sonar_kalman_l1;
extern float param_sonar_kalman_l2;
extern bool param_usb_send_video;
extern bool param_usb_send_flow;
extern bool param_usb_send_gyro;
extern bool param_usb_send_forward;
extern bool param_usb_send_debug;
extern bool param_video_only;
extern float param_video_rate;
extern float param_bottom_flow_feature_threshold;
extern float param_bottom_flow_value_threshold;
extern bool param_bottom_flow_hist_filter;
extern bool param_bottom_flow_gyro_compensation;
extern bool param_bottom_flow_lp_filtered;
extern float param_bottom_flow_weight_new;
extern float param_bottom_flow_serial_throttle_factor;
extern uint32_t param_sensor_position;
extern float debug_variable;

typedef struct param_info_item {
  const char* name;
  ParameterType_TypeDef type;
  ParameterAccess_TypeDef access;
  void* variable;
  void (*on_change)(void);
} param_info_item;

extern const param_info_item param_info[];
extern const uint32_t params_count;

/******************************************************************
  * ALL SETTINGS FUNCTIONS
  */

void set_sensor_position_settings(uint8_t sensor_position);

#endif /* SETTINGS_H_ */
