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

#ifndef RESULT_ACCUMULATOR_H_
#define RESULT_ACCUMULATOR_H_

#include <stdint.h>
#include <stdbool.h>
#include "i2c_frame.h"

typedef struct _result_accumulator_frame {
	float dt;					///< The time delta of this sample.
	float dropped_dt; ///< The time delta of samples that have been dropped before this sample
	float x_rate;				///< The current x_rate of the gyro in rad / sec. (Image/Flow coordinates)
	float y_rate;				///< The current y_rate of the gyro in rad / sec. (Image/Flow coordinates)
	float z_rate;				///< The current z_rate of the gyro in rad / sec. (Image/Flow coordinates)
	int16_t gyro_temp;		///< Temperature * 100 in centi-degrees Celsius
	uint8_t qual;				///< The quality output of the flow algorithm.
	float rad_per_pixel;  ///< Pixel to radian conversion factor.
	float pixel_flow_x;			///< The measured x-flow in the current image in pixel. Sensor linear motion along the positive X axis induces a negative flow.
	float pixel_flow_y;			///< The measured y-flow in the current image in pixel. Sensor linear motion along the positive Y axis induces a negative flow.
	float ground_distance;		///< The measured distance to the ground in meter.
	uint32_t distance_age;		///< Age of the distance measurement in us.
	float max_px_frame; ///< Max velocity, in pixels per frame.
} result_accumulator_frame;

typedef struct _result_accumulator_ctx {
	uint32_t frame_count;
	float px_flow_x_accu;
	float px_flow_y_accu;
	float rad_flow_x_accu;
	float rad_flow_y_accu;
	float m_flow_x_accu;
	float m_flow_y_accu;
	uint8_t min_quality;
	uint16_t data_count;
	uint16_t valid_data_count;
	float valid_dist_time;
	float valid_time;
	float full_time;
	float gyro_x_accu;
	float gyro_y_accu;
	float gyro_z_accu;
	int16_t last_gyro_temp;		///< Temperature * 100 in centi-degrees Celsius
	float last_ground_distance;		///< The measured distance to the ground in meter.
	uint32_t last_distance_age;		///< Age of the distance measurement in us.
	float flow_cap_mv_rad; /**< The maximum velocity that could be measured by all datasets together in one accumulation period. 
	                        *   This is the minimum of all max velocities. In rad / s. */
} result_accumulator_ctx;

typedef struct _result_accumulator_output_flow {
	int16_t flow_x;		///< Flow in pixels * 10 in x-sensor direction (dezi-pixels)
	int16_t flow_y;		///< Flow in pixels * 10 in y-sensor direction (dezi-pixels)
	float flow_comp_m_x;  ///< Flow in meters in x-sensor direction, angular-speed compensated
	float flow_comp_m_y;  ///< Flow in meters in y-sensor direction, angular-speed compensated
	uint8_t quality;	///< Optical flow quality / confidence. 0: bad, 255: maximum quality
	float ground_distance;	///< Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
} result_accumulator_output_flow;

typedef struct _result_accumulator_output_flow_rad {
	uint32_t integration_time;	///< Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain average flow.
	float integrated_x;			///< Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
	float integrated_y;			///< Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
	float integrated_xgyro;		///< RH rotation around X axis (rad)
	float integrated_ygyro;		///< RH rotation around X axis (rad)
	float integrated_zgyro;		///< RH rotation around X axis (rad)
	uint8_t quality;			///< Optical flow quality / confidence. 0: bad, 255: maximum quality
	int16_t temperature;		///< Temperature * 100 in centi-degrees Celsius
	uint32_t time_delta_distance_us;	///< Time in microseconds since the distance was sampled.
	float ground_distance;		///< Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative value: Unknown distance.
} result_accumulator_output_flow_rad;


typedef struct _result_accumulator_output_flow_i2c {
	uint32_t frame_count;		///< Frame counter.
	uint16_t valid_frames;		///< Number of valid frames.
	uint32_t integration_time;	///< Integration time in microseconds.
	int16_t rad_flow_x;			///< Flow in 10 * mrad around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
	int16_t rad_flow_y;			///< Flow in 10 * mrad around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
	int16_t integrated_gyro_x;	///< RH rotation around X axis (10 * mrad)
	int16_t integrated_gyro_y;	///< RH rotation around Y axis (10 * mrad)
	int16_t integrated_gyro_z;	///< RH rotation around Z axis (10 * mrad)
	int16_t gyro_x;				///< RH rotation around X axis (10 * mrad)
	int16_t gyro_y;				///< RH rotation around Y axis (10 * mrad)
	int16_t gyro_z;				///< RH rotation around Z axis (10 * mrad)
	uint8_t quality;			///< Optical flow quality / confidence. 0: bad, 255: maximum quality
	int16_t temperature;		///< Temperature * 100 in centi-degrees Celsius
	uint32_t time_delta_distance_us;	///< Time in microseconds since the distance was sampled.
	uint16_t ground_distance;	///< Distance to the center of the flow field in mmeters.
} result_accumulator_output_flow_i2c;

/**	Initializes the result accumulator.
 */
void result_accumulator_init(result_accumulator_ctx *ctx);
void result_accumulator_feed(result_accumulator_ctx *ctx, const result_accumulator_frame* frame);

/**	Recalculates the output values of the result_accumulator. Call this before using any of the output values.
 */
void result_accumulator_calculate_output_flow(result_accumulator_ctx *ctx, uint16_t min_valid_data_count_percent, result_accumulator_output_flow *out);
void result_accumulator_calculate_output_flow_rad(result_accumulator_ctx *ctx, uint16_t min_valid_data_count_percent, result_accumulator_output_flow_rad *out);
void result_accumulator_calculate_output_flow_i2c(result_accumulator_ctx *ctx, uint16_t min_valid_data_count_percent, result_accumulator_output_flow_i2c *out);
void result_accumulator_fill_i2c_data(result_accumulator_ctx *ctx, i2c_frame* f, i2c_integral_frame* f_integral);

/**	Resets the accumulator to prepare it for the next accumulation phase.
 */
void result_accumulator_reset(result_accumulator_ctx *ctx);

#endif /* RESULT_ACCUMULATOR_H_ */
