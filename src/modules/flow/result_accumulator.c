/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Petri Tanskanen <tpetri@inf.ethz.ch>
 *   		 Lorenz Meier <lm@inf.ethz.ch>
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

#include <string.h>
#include <stdint.h>
#include <math.h>

#include "result_accumulator.h"
#include "i2c_frame.h"
#include "settings.h"

void result_accumulator_init(result_accumulator_ctx *ctx)
{
	/* set everything to zero */
	memset(ctx, 0, sizeof(result_accumulator_ctx));
	/* set non-zero initial values */
	ctx->min_quality = 255;
	ctx->last_ground_distance = -1;
}

void result_accumulator_feed(result_accumulator_ctx *ctx, const result_accumulator_frame* frame)
{	
	ctx->last_ground_distance = frame->ground_distance;
	ctx->last_distance_age    = frame->distance_age;
	ctx->last_gyro_temp       = frame->gyro_temp;
	
	/* accumulate the values */
	if (frame->qual > 0) {
		ctx->px_flow_x_accu += frame->pixel_flow_x;
		ctx->px_flow_y_accu += frame->pixel_flow_y;
		
		/* Convert the flow from pixels to rad:
		 * This is done with a linear relationship because the involved angles are low.
		 * 
		 * Calculation for maximum possible error: (for an assumed drone configuration)
		 * With a 10Hz accumulation period and 5m/s at 1m distance the real speed in rad is 0.463rad per accumulation period.
		 * Each frame (taken at 400Hz) measures a distance of 4.16px (at 8mm focal length, 4x binning -> 3mm/px).
		 * With the linear formula this equates to 0.0125rad per frame.
		 * Accumulated value in rad is thus 40 * 0.0125 = 5rad. Error: 8%.
		 * At half the speed the error is just 2%. */
		float flow_x_rad = frame->pixel_flow_y * frame->rad_per_pixel;
		float flow_y_rad = - frame->pixel_flow_x * frame->rad_per_pixel;
		ctx->rad_flow_x_accu += flow_x_rad;
		ctx->rad_flow_y_accu += flow_y_rad;

		ctx->gyro_x_accu += frame->x_rate * frame->dt;
		ctx->gyro_y_accu += frame->y_rate * frame->dt;
		ctx->gyro_z_accu += frame->z_rate * frame->dt;
		
		/* the quality is the minimum of all quality measurements */
		if (frame->qual < ctx->min_quality || ctx->valid_data_count == 0) {
			ctx->min_quality = frame->qual;
		}
		
		ctx->valid_data_count++;
		ctx->valid_time += frame->dt;
		
		if(ctx->last_ground_distance >= 0){
			ctx->m_flow_x_accu += flow_x_rad * ctx->last_ground_distance; 
			ctx->m_flow_y_accu += flow_y_rad * ctx->last_ground_distance; 
			ctx->valid_dist_time += frame->dt;
		}
	}
	
	/* convert the algorithm's capability into a velocity in rad / s: */
	float flow_cap_mv_rad = frame->max_px_frame * frame->rad_per_pixel / frame->dt;

	if (ctx->data_count > 0) {
		if (flow_cap_mv_rad < ctx->flow_cap_mv_rad) {
			ctx->flow_cap_mv_rad = flow_cap_mv_rad;
		}
	} else {
		ctx->flow_cap_mv_rad = flow_cap_mv_rad;
	}

	ctx->data_count++;
	ctx->frame_count++;
	ctx->full_time += frame->dt + frame->dropped_dt;
}

void result_accumulator_calculate_output_flow(result_accumulator_ctx *ctx, uint16_t min_valid_data_count_percent, result_accumulator_output_flow *out)
{
	if (ctx->valid_data_count * 100u >= ctx->data_count * min_valid_data_count_percent && ctx->valid_data_count > 0) {
		/*** calculate the output values for the flow mavlink message ***/
		/* scale the averaged values from valid time to full time (we extrapolate the invalid data sets) */
		float time_scaling_f = ctx->full_time / ctx->valid_time;
		out->flow_x = floor(ctx->px_flow_x_accu * time_scaling_f * 10.0f + 0.5f);
		out->flow_y = floor(ctx->px_flow_y_accu * time_scaling_f * 10.0f + 0.5f);
		if(ctx->valid_dist_time > 0){
			float time_scaling_dist = ctx->full_time / ctx->valid_dist_time;
			out->flow_comp_m_x = ctx->m_flow_x_accu * time_scaling_dist;
			out->flow_comp_m_y = ctx->m_flow_y_accu * time_scaling_dist;
		}else{
			out->flow_comp_m_x = 0;
			out->flow_comp_m_y = 0;
		}
		out->quality = ctx->min_quality;
	} else {
		/* not enough valid data */
		out->flow_x = 0;
		out->flow_y = 0;
		out->flow_comp_m_x = 0;
		out->flow_comp_m_y = 0;
		out->quality = 0;
	}
	/* averaging the distance is no use */
	out->ground_distance = ctx->last_ground_distance;
}

void result_accumulator_calculate_output_flow_rad(result_accumulator_ctx *ctx, uint16_t min_valid_data_count_percent, result_accumulator_output_flow_rad *out)
{
	if (ctx->valid_data_count * 100u >= ctx->data_count * min_valid_data_count_percent && ctx->valid_data_count > 0) {
		/*** calculate the output values for the flow_rad mavlink message ***/
		out->integration_time = ctx->valid_time * 1000000.0f;
		out->integrated_x = ctx->rad_flow_x_accu;
		out->integrated_y = ctx->rad_flow_y_accu;
		out->integrated_xgyro = ctx->gyro_x_accu;
		out->integrated_ygyro = ctx->gyro_y_accu;
		out->integrated_zgyro = ctx->gyro_z_accu;
		out->quality = ctx->min_quality;
		/* averaging the distance and temperature is no use */
		out->temperature = ctx->last_gyro_temp;
	} else {
		/* not enough valid data */
		out->integration_time = 0;
		out->integrated_x = 0;
		out->integrated_y = 0;
		out->integrated_xgyro = 0;
		out->integrated_ygyro = 0;
		out->integrated_zgyro = 0;
		out->quality = 0;
		out->temperature = ctx->last_gyro_temp;
	}
	out->time_delta_distance_us = ctx->last_distance_age;
	out->ground_distance = ctx->last_ground_distance;
}

void result_accumulator_calculate_output_flow_i2c(result_accumulator_ctx *ctx, uint16_t min_valid_data_count_percent, result_accumulator_output_flow_i2c *out)
{
	if (ctx->valid_data_count * 100u >= ctx->data_count * min_valid_data_count_percent && ctx->valid_data_count > 0) {
		/*** calculate the output values for the i2c message ***/
		out->frame_count = ctx->frame_count;
		out->valid_frames = ctx->valid_data_count;
		out->integration_time = ctx->valid_time * 1000000.0f;
		out->rad_flow_x = floor(ctx->rad_flow_x_accu * 10000.0f + 0.5f);
		out->rad_flow_y = floor(ctx->rad_flow_y_accu * 10000.0f + 0.5f);
		out->integrated_gyro_x = floor(ctx->gyro_x_accu * 10000.0f + 0.5f);
		out->integrated_gyro_y = floor(ctx->gyro_y_accu * 10000.0f + 0.5f);
		out->integrated_gyro_z = floor(ctx->gyro_z_accu * 10000.0f + 0.5f);
		out->gyro_x = floor(ctx->gyro_x_accu * (100.0f / ctx->valid_time) + 0.5f);
		out->gyro_y = floor(ctx->gyro_y_accu * (100.0f / ctx->valid_time) + 0.5f);
		out->gyro_z = floor(ctx->gyro_z_accu * (100.0f / ctx->valid_time) + 0.5f);
		out->quality = ctx->min_quality;
		out->temperature = ctx->last_gyro_temp;
	} else {
		/* not enough valid data */
		out->frame_count = ctx->frame_count;
		out->valid_frames = 0;
		out->integration_time = 0;
		out->rad_flow_x = 0;
		out->rad_flow_y = 0;
		out->integrated_gyro_x = 0;
		out->integrated_gyro_y = 0;
		out->integrated_gyro_z = 0;
		out->gyro_x = 0;
		out->gyro_y = 0;
		out->gyro_z = 0;
		out->quality = 0;
		out->temperature = ctx->last_gyro_temp;
	}
	out->time_delta_distance_us = ctx->last_distance_age;
	if (ctx->last_ground_distance >= 0) {
		out->ground_distance = floor(ctx->last_ground_distance * 1000.0f + 0.5f);
	} else {
		out->ground_distance = 0;
	}
}

// TODO: why doesn't the above function write these structs directly?
void result_accumulator_fill_i2c_data(result_accumulator_ctx *ctx, i2c_frame* f, i2c_integral_frame* f_integral) {
	result_accumulator_output_flow output_flow;
	result_accumulator_output_flow_i2c output_i2c;
	int min_valid_ratio = global_data.param[PARAM_ALGORITHM_MIN_VALID_RATIO];
	result_accumulator_calculate_output_flow(ctx, min_valid_ratio, &output_flow);
	result_accumulator_calculate_output_flow_i2c(ctx, min_valid_ratio, &output_i2c);

	/* write the i2c_frame */
	f->frame_count = ctx->frame_count;
	f->pixel_flow_x_sum = output_flow.flow_x;
	f->pixel_flow_y_sum = output_flow.flow_y;
	f->flow_comp_m_x = floor(output_flow.flow_comp_m_x * 1000.0f + 0.5f);
	f->flow_comp_m_y = floor(output_flow.flow_comp_m_y * 1000.0f + 0.5f);
	f->qual = output_flow.quality;
	f->ground_distance = output_i2c.ground_distance;

	f->gyro_x_rate = output_i2c.gyro_x;
	f->gyro_y_rate = output_i2c.gyro_y;
	f->gyro_z_rate = output_i2c.gyro_z;
	f->gyro_range = getGyroRange();

	if (output_i2c.time_delta_distance_us < 255 * 1000) {
		f->distance_timestamp = output_i2c.time_delta_distance_us / 1000; //convert to ms
	} else {
		f->distance_timestamp = 255;
	}

	/* write the i2c_integral_frame */
	f_integral->frame_count_since_last_readout = output_i2c.valid_frames;
	f_integral->gyro_x_rate_integral = output_i2c.integrated_gyro_x;		//mrad*10
	f_integral->gyro_y_rate_integral = output_i2c.integrated_gyro_y;		//mrad*10
	f_integral->gyro_z_rate_integral = output_i2c.integrated_gyro_z; 	//mrad*10
	f_integral->pixel_flow_x_integral = output_i2c.rad_flow_x; 			//mrad*10
	f_integral->pixel_flow_y_integral = output_i2c.rad_flow_y; 			//mrad*10
	f_integral->integration_timespan = output_i2c.integration_time;      //microseconds
	f_integral->ground_distance = output_i2c.ground_distance;		    //mmeters
	f_integral->distance_timestamp = output_i2c.time_delta_distance_us;  	//microseconds
	f_integral->qual = output_i2c.quality;										//0-255 linear quality measurement 0=bad, 255=best
	f_integral->gyro_temperature = output_i2c.temperature;						//Temperature * 100 in centi-degrees Celsius
}

void result_accumulator_reset(result_accumulator_ctx *ctx)
{
	ctx->px_flow_x_accu = 0;
	ctx->px_flow_y_accu = 0;

	ctx->rad_flow_x_accu = 0;
	ctx->rad_flow_y_accu = 0;

	ctx->m_flow_x_accu = 0;
	ctx->m_flow_y_accu = 0;

	ctx->gyro_x_accu = 0;
	ctx->gyro_y_accu = 0;
	ctx->gyro_z_accu = 0;

	ctx->min_quality = 255;

	ctx->valid_data_count = 0;
	ctx->data_count = 0;
	ctx->valid_time = 0;
	ctx->full_time  = 0;
	ctx->valid_dist_time = 0;
}
