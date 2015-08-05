/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Simon Laube <simon@leitwert.ch>
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

#include "camera.h"
#include "timer.h"

#include <string.h>

void camera_transport_transfer_done_fn(void *usr, const void *buffer, size_t size);
void camera_transport_frame_done_fn(void *usr);

bool camera_init(camera_ctx *ctx, const camera_sensor_interface *sensor, const camera_transport_interface *transport,
				 const  camera_img_param *img_param,
				 camera_image_buffer buffers[], size_t buffer_count) {
	memset(ctx, 0, sizeof(camera_ctx));
	ctx->sensor    = sensor;
	ctx->transport = transport;
	// check parameter:
	if (buffer_count > CAMERA_MAX_BUFFER_COUNT && buffer_count < 2) {
		return false;
	}
	uint32_t img_size = (uint32_t)img_param->size.x * (uint32_t)img_param->size.y;
	if (img_size % ctx->transport->transfer_size != 0 || img_size / ctx->transport->transfer_size < 2) {
		// invalid size parameter!
		return false;
	}
	// initialize state:
	ctx->img_stream_param = *img_param;
	int i;
	for (i = 0; i < buffer_count; ++i) {
		// check the buffer:
		if (buffers[i].buffer_size < img_size || buffers[i].buffer == NULL) {
			return false;
		}
		// init the buffer:
		ctx->buffers[i] = buffers[i];
		ctx->buffers[i].frame_number = 0;
		ctx->buffers[i].timestamp = get_boot_time_us();
		ctx->buffers[i].param = *img_param;
		memset(ctx->buffers[i].buffer, 0, img_size);
		// init the avail_bufs array:
		ctx->avail_bufs[i] = i;
	}
	ctx->avail_buf_count = buffer_count;
	ctx->new_frame_arrived = false;
	
	ctx->snapshot_buffer = NULL;
	
	ctx->last_read_frame_index = 0;
	
	ctx->cur_frame_index = buffer_count + 1;
	ctx->receiving_frame = false;
	ctx->target_buffer   = NULL;
	
	ctx->seq_snapshot_active                 = false;
	ctx->seq_updating_img_stream             = false;
	ctx->seq_write_img_stream_param_yourself = false;
	// initialize hardware:
	if (!ctx->transport->init(ctx->transport->usr,
							  camera_transport_transfer_done_fn,
							  camera_transport_frame_done_fn,
							  ctx)) {
		return false;
	}
	if (!ctx->sensor->init(ctx->sensor->usr, img_param)) {
		return false;
	}
	return true;
}

static void uint32_t_memcpy(uint32_t *dst, const uint32_t *src, size_t count) {
	int i;
	for (i = 0; i < count; ++i) {
		dst[i] = src[i];
	}
}

void camera_transport_transfer_done_fn(void *usr, const void *buffer, size_t size) {
	camera_ctx *ctx = (camera_ctx *)usr;
	if (ctx->receiving_frame) {
		// check if we have a target buffer:
		if (ctx->target_buffer != NULL) {
			uint32_t_memcpy((uint32_t *)((uint8_t *)ctx->target_buffer->buffer + ctx->cur_frame_pos), 
							(const uint32_t *)buffer, size / 4);
		}
		// update current position:
		ctx->cur_frame_pos += size;
		// check if we are done:
		if (ctx->cur_frame_pos >= ctx->cur_frame_size) {
			// frame done!
			if (ctx->target_buffer_index >= 0) {
				// put back into available buffers (in the front)
				memmove(ctx->avail_bufs + 1, ctx->avail_bufs, ctx->avail_buf_count * sizeof(ctx->avail_bufs[0]));
				ctx->avail_bufs[0] = ctx->target_buffer_index;
				ctx->avail_buf_count += 1;
				// notify camera_img_stream_get_buffers function.
				ctx->new_frame_arrived = true;
			}
			// reset state:
			ctx->target_buffer = NULL;
			ctx->receiving_frame = false;
		}
	} else {
		// no frame currently as the target frame. it must be the beginning of a new frame then!
		// empty the DMA buffer as quickly as possible:
		size_t size_w = size / 4;
		uint32_t buf[size_w];
		uint32_t_memcpy(buf, (const uint32_t *)buffer, size_w);
		// update the sensor: (this might trigger some I2C transfers)
		ctx->sensor->notify_readout_start(ctx->sensor->usr);
		// get current sensor parameter:
		camera_img_param cparam;
		ctx->sensor->get_current_param(ctx->sensor->usr, &cparam);
		// update the receiving variables:
		ctx->cur_frame_index += 1;
		ctx->receiving_frame = true;
		ctx->target_buffer   = NULL;
		ctx->cur_frame_size  = (uint32_t)cparam.size.x * (uint32_t)cparam.size.y;
		ctx->cur_frame_pos   = size;
		ctx->target_buffer_index = -1;
		// check that the size parameters match:
		if (cparam.size.x == ctx->img_stream_param.size.x ||
			cparam.size.y == ctx->img_stream_param.size.y) {
			// get least recently used buffer from the available buffers:
			ctx->target_buffer_index = ctx->avail_bufs[ctx->avail_buf_count - 1];
			ctx->avail_buf_count -= 1;
			ctx->target_buffer   = &ctx->buffers[ctx->target_buffer_index];
			// initialize it:
			ctx->target_buffer->timestamp    = get_boot_time_us();
			ctx->target_buffer->frame_number = ctx->cur_frame_index;
			ctx->target_buffer->param        = cparam;
			// write data to it: (at position 0)
			uint32_t_memcpy((uint32_t *)ctx->target_buffer->buffer, buf, size_w);
		}
	}
}

void camera_transport_frame_done_fn(void *usr) {
}

bool camera_img_stream_schedule_param_change(camera_ctx *ctx, const camera_img_param *img_param) {
}

int camera_img_stream_get_buffers(camera_ctx *ctx, camera_image_buffer **buffers[], size_t count) {
	
}

void camera_img_stream_return_buffers(camera_ctx *ctx, camera_image_buffer **buffers[], size_t count) {
	
}

bool camera_snapshot_schedule(camera_ctx *ctx, const camera_img_param *img_param, camera_image_buffer *dst, camera_snapshot_done_cb cb) {
}


