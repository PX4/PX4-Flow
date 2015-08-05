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

#include "stm32f4xx_conf.h"
#include "core_cmFunc.h"
#include "led.h"

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
	ctx->startup_discard_frame_count = -1;
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
	ctx->buffer_count    = buffer_count;
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
	/* after initialization start the discard count down! */
	ctx->startup_discard_frame_count = 16;
	return true;
}

static void uint32_t_memcpy(uint32_t *dst, const uint32_t *src, size_t count) {
	int i;
	for (i = 0; i < count; ++i) {
		dst[i] = src[i];
	}
}

static void camera_buffer_fifo_remove_front(camera_ctx *ctx, int *out, size_t count) {
	size_t bc = ctx->avail_buf_count;
	size_t i;
	// read out:
	for (i = 0; i < count; ++i) {
		*out++ = ctx->avail_bufs[i];
	}
	// close gap:
	for (i = count; i < bc; ++i) {
		ctx->avail_bufs[i - count] = ctx->avail_bufs[i];
	}
	ctx->avail_buf_count = bc - count;
}

static void camera_buffer_fifo_remove_back(camera_ctx *ctx, int *out, size_t count) {
	size_t bc = ctx->avail_buf_count;
	size_t i;
	// read out:
	for (i = bc - count; i < bc; ++i) {
		*out++ = ctx->avail_bufs[i];
	}
	// reduce count:
	ctx->avail_buf_count = bc - count;
}

static void camera_buffer_fifo_push_at(camera_ctx *ctx, size_t pos, const int *in, size_t count) {
	size_t bc = ctx->avail_buf_count;
	size_t i;
	// move away:
	for (i = bc; i > pos; --i) {
		ctx->avail_bufs[i - 1 + count] = ctx->avail_bufs[i - 1];
	}
	// fill in:
	for (i = pos; i < pos + count; ++i) {
		ctx->avail_bufs[i] = *in++;
	}
	// update count:
	ctx->avail_buf_count = bc + count;
}

void camera_transport_transfer_done_fn(void *usr, const void *buffer, size_t size) {
	camera_ctx *ctx = (camera_ctx *)usr;
	if (ctx->startup_discard_frame_count != 0) {
		return;
	}
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
				camera_buffer_fifo_push_at(ctx, 0, &ctx->target_buffer_index, 1);
				if (ctx->put_back_buf_pos < ctx->avail_buf_count) {
					ctx->put_back_buf_pos += 1;
				}
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
			camera_buffer_fifo_remove_back(ctx, &ctx->target_buffer_index, 1);
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
	camera_ctx *ctx = (camera_ctx *)usr;
	int fdc = ctx->startup_discard_frame_count;
	if (fdc > 0) {
		fdc--;
		ctx->startup_discard_frame_count = fdc;
		/* re-initialize the transport twice */
		if (fdc == 0 || fdc == 1) {
			ctx->transport->reset(ctx->transport->usr);
		}
	}
}

bool camera_img_stream_schedule_param_change(camera_ctx *ctx, const camera_img_param *img_param) {
	if (!ctx->seq_snapshot_active) {
		return ctx->sensor->prepare_update_param(ctx->sensor->usr, img_param);
	}
	return false;
}

static bool camera_img_stream_get_buffers_idx(camera_ctx *ctx, int bidx[], size_t count, bool reset_new_frm) {
	int i;
	__disable_irq();
	camera_buffer_fifo_remove_front(ctx, bidx, count);
	/* check that the buffers are in consecutive order: */
	bool consecutive = true;
	uint32_t exp_frame = ctx->buffers[bidx[0]].frame_number - 1;
	for (i = 1; i < count; ++i, --exp_frame) {
		if (ctx->buffers[bidx[i]].frame_number != exp_frame) {
			consecutive = false;
			break;
		}
	}
	if (consecutive) {
		/* good! */
		ctx->put_back_buf_pos = 0;
		ctx->buffers_are_reserved = true;
	} else {
		/* not good. put back the buffers: */
		camera_buffer_fifo_push_at(ctx, 0, bidx, count);
	}
	if (reset_new_frm) {
		ctx->new_frame_arrived = false;
	}
	__enable_irq();
	return consecutive;
}

int camera_img_stream_get_buffers(camera_ctx *ctx, camera_image_buffer *buffers[], size_t count, bool wait_for_new) {
	if (ctx->buffers_are_reserved) return -1;
	if (count > ctx->buffer_count - 1 || count <= 0) return -1;
	/* buffer management needs to be performed atomically: */
	int bidx[count];
	int i;
	while (1) {
		if (wait_for_new) {
			/* wait until a new frame is available: */
			while(!ctx->new_frame_arrived);
		}
		if (camera_img_stream_get_buffers_idx(ctx, bidx, count, wait_for_new)) {
			/* update the pointers: */
			for (i = 0; i < count; ++i) {
				buffers[i] = &ctx->buffers[bidx[i]];
			}
			return 0;
		} else {
			/* not possible! check if we want to wait for the new frame: */
			if (!wait_for_new) {
				return 1;
			}
		}
	}
}

void camera_img_stream_return_buffers(camera_ctx *ctx, camera_image_buffer *buffers[], size_t count) {
	if (!ctx->buffers_are_reserved) return;
	/* get the buffer indexes: */
	int bidx[count];
	int i;
	for (i = 0; i < count; ++i) {
		int idx = buffers[i] - &ctx->buffers[0];
		if (idx < 0 || idx >= ctx->buffer_count) return;
		bidx[i] = idx;
	}
	/* buffer management needs to be performed atomically: */
	__disable_irq();
	size_t at_pos = ctx->put_back_buf_pos;
	if (at_pos > ctx->avail_buf_count) at_pos = ctx->avail_buf_count;
	camera_buffer_fifo_push_at(ctx, ctx->put_back_buf_pos, bidx, count);
	ctx->buffers_are_reserved = false;
	__enable_irq();
}

bool camera_snapshot_schedule(camera_ctx *ctx, const camera_img_param *img_param, camera_image_buffer *dst, camera_snapshot_done_cb cb) {
	return false;
}


