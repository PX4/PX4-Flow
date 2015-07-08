
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

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "settings.h"

#define __INLINE inline
#define __ASM asm
#include "core_cm4_simd.h"

#include "quality_measurement.h"

#define QUEUE_SIZE 80

typedef struct _qual_datapoint {
	float x;
	float y;
	float x_orig;
	float y_orig;
	float qual_iqr;
} qual_datapoint;

static qual_datapoint in = {};
static qual_datapoint arr[QUEUE_SIZE] = {};
static int head = 0; //starting index of the queue

static qual_datapoint input_flow_filter_state = {};

static qual_output qual_output_point = {};

static void enqueue(qual_datapoint *q, int *head, qual_datapoint data) {
	q[(*head)] = data;
	(*head)++;
	if((*head) >= QUEUE_SIZE)
		(*head) = 0;
}

int floatcompare(const void* elem1, const void* elem2)
{
    if(*(const float*)elem1 < *(const float*)elem2)
        return -1;
    return *(const float*)elem1 > *(const float*)elem2;
}

static struct _qual_output quality_calculate(){
	float sum_flow_x = 0;
	float sum_flow_y = 0;
	float sum2_flow_x = 0;
	float sum2_flow_y = 0;
	float mean_qual_iqr = 0;
	float sum_qual_iqr = 0;
	float x_array[QUEUE_SIZE] = {};
	float y_array[QUEUE_SIZE] = {};

	for(int i = 0; i < QUEUE_SIZE; i++)
	{
		x_array[i] = arr[i].x;
		y_array[i] = arr[i].y;
	}

	//calculate median of flow
	qsort(x_array, QUEUE_SIZE, sizeof (float), floatcompare);
	qsort(y_array, QUEUE_SIZE, sizeof (float), floatcompare);
	if(QUEUE_SIZE % 2 == 0)
	{
		qual_output_point.median_x = (x_array[QUEUE_SIZE / 2 - 1] + x_array[QUEUE_SIZE / 2]) / 2;
		qual_output_point.median_y = (x_array[QUEUE_SIZE / 2 - 1] + x_array[QUEUE_SIZE / 2]) / 2;
	}
	else
	{
		qual_output_point.median_x = x_array[(QUEUE_SIZE - 1) / 2];
		qual_output_point.median_y = x_array[(QUEUE_SIZE - 1) / 2];
	}

	//calculate mean
	for(int i = 0; i < QUEUE_SIZE; i++){
		sum_flow_x += arr[i].x;
		sum_flow_y += arr[i].y;
		sum_qual_iqr += arr[i].qual_iqr;
	}
	qual_output_point.mean_x = sum_flow_x/QUEUE_SIZE;
	qual_output_point.mean_y = sum_flow_y/QUEUE_SIZE;
	mean_qual_iqr = sum_qual_iqr/QUEUE_SIZE;

	//calculate variance
	for(int i = 0; i < QUEUE_SIZE; i++){
		sum2_flow_x += (arr[i].x - qual_output_point.mean_x) * (arr[i].x - qual_output_point.mean_x);
		sum2_flow_y += (arr[i].y - qual_output_point.mean_y) * (arr[i].y - qual_output_point.mean_y);
	}
	qual_output_point.var = ((sum2_flow_x + sum2_flow_y) / QUEUE_SIZE) / (1.0 + sqrt(in.x_orig * in.x_orig + 
									in.y_orig * in.y_orig));

	//calculate covariance between flow_y and flow_x
	float total = 0.0f;
	for(int i = 0; i < QUEUE_SIZE; i++){
		total += (arr[i].x - qual_output_point.mean_x) * (arr[i].y - qual_output_point.mean_y);
	}
	qual_output_point.covar = total/QUEUE_SIZE;

	qual_output_point.qual = (int)sum_flow_y/QUEUE_SIZE;
	qual_output_point.qual_iqr = in.qual_iqr;

	return qual_output_point;
}

#define FILTER_TAU 4910

qual_output quality_new_measurement(float pixel_flow_x, float pixel_flow_y, float dt, float qual_iqr) {

	// high pass filter with exponential filter:
	in.x_orig = pixel_flow_x;
	in.y_orig = pixel_flow_y;
	if(global_data.param[PARAM_QUALITY_FILTER] > 0)
	{
		float alpha = 1.0 - exp(- dt / FILTER_TAU);
		input_flow_filter_state.x = alpha*(pixel_flow_x/dt) + (1.0 - alpha)*input_flow_filter_state.x;
		input_flow_filter_state.y = alpha*(pixel_flow_y/dt) + (1.0 - alpha)*input_flow_filter_state.y;
		in.x = pixel_flow_x - input_flow_filter_state.x;
		in.y = pixel_flow_y - input_flow_filter_state.y;
		in.qual_iqr = qual_iqr;
		enqueue(arr, &head, in);
	}
	else
	{
		in.x = pixel_flow_x/dt;
		in.y = pixel_flow_y/dt;
		in.qual_iqr = qual_iqr;
		enqueue(arr, &head, in);
	}

	// queue into circular buffer:
	return quality_calculate();
}


