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

#ifndef FLOW_H_
#define FLOW_H_

#include <stdint.h>

typedef struct _flow_raw_result {
	float x;		///< The flow in x direction
	float y;		///< The flow in y direction
	float quality;	///< The quality of this result. 0 = bad
	uint8_t at_x;	///< The mid-position of the patch that was used to calculate the flow.
	uint8_t at_y;	///< The mid-position of the patch that was used to calculate the flow.
} flow_raw_result;

/**
 *  @brief Computes pixel flow from image1 to image2
 *  Searches the corresponding position in the new image (image2) of max. 64 pixels from the old image (image1).
 *	@param image1 The older image
 *	@param image2 The new image
 *	@param x_rate The gyro x-rate during the frame interval in pixels. (In the image x direction)
 *	@param y_rate The gyro y-rate during the frame interval in pixels. (In the image y direction)
 *	@param z_rate The gyro z-rate during the frame interval in radians.
 *	@param out    Array which receives the raw result vectors computed for the blocks in the image.
 *	@param result_count The available space in the out buffer.
 *	@return The number of results written to the out buffer.
 */
uint16_t compute_flow(uint8_t *image1, uint8_t *image2, float x_rate, float y_rate, float z_rate,
					  flow_raw_result *out, uint16_t max_out);

/**
 *  @brief Computes pixel flow from image1 to image2
 *  Searches the corresponding position in the new image (image2) of max. 64 pixels from the old image (image1)
 *  with the KLT method and outputs the value of all flow vectors.
 *	@param image1 The older image
 *	@param image2 The new image
 *	@param x_rate The gyro x-rate during the frame interval in pixels. (In the image x direction)
 *	@param y_rate The gyro y-rate during the frame interval in pixels. (In the image y direction)
 *	@param z_rate The gyro z-rate during the frame interval in radians.
 *	@param out    Array which receives the raw result vectors computed for the blocks in the image.
 *	@param result_count The available space in the out buffer.
 *	@return The number of results written to the out buffer.
 */
uint16_t compute_klt(uint8_t *image1, uint8_t *image2, float x_rate, float y_rate, float z_rate,
					 flow_raw_result *out, uint16_t max_out);

/**
* 
*	@brief Extracts pixel flow from the result vector
*	@param in Raw result vector from flow calculation.
*	@param result_count Number of results in flow_raw_result.
*	@param px_flow_x Receives the pixel flow in x direction.
*	@param px_flow_y Receives the pixel flow in y direction.
*	@param accuracy_p Outlier detection threshold in percent. (0 - 1).
*	@param accuracy_px Minimum outlier detection threshold in absolute pixel flow values.
*/
uint8_t flow_extract_result(flow_raw_result *in, uint16_t result_count, float *px_flow_x, float *px_flow_y,
				float accuracy_p, float accuracy_px);

#endif /* FLOW_H_ */
