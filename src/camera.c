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

#include <string.h>

void camera_transport_transfer_done_fn(void *usr, void *buffer, size_t size);
void camera_transport_frame_done_fn(void *usr);

int camera_init(camera_ctx *ctx, camera_sensor_interface *sensor, camera_transport_interface *transport,
				const camera_img_param *img_param,
				camera_image_buffer buffers[], size_t buffer_count) {
	memset(ctx, 0, sizeof(camera_ctx));
	ctx->sensor    = sensor;
	ctx->transport = transport;
	// initialize state:
	
	// initialize hardware:
	ctx->transport->init(ctx->transport->usr,
						 camera_transport_transfer_done_fn,
						 camera_transport_frame_done_fn,
						 ctx);
	ctx->sensor->init(ctx->sensor->usr, img_param);
}

int camera_img_stream_schedule_param_change(camera_ctx *ctx, const camera_img_param *img_param) {
	
}

int camera_img_stream_get_buffers(camera_ctx *ctx, camera_image_buffer **buffers[], size_t count) {
	
}

void camera_img_stream_return_buffers(camera_ctx *ctx, camera_image_buffer **buffers[], size_t count) {
	
}

int camera_snapshot_schedule(camera_ctx *ctx, const camera_img_param *img_param, camera_image_buffer *dst, camera_snapshot_done_cb cb) {
	
}



void camera_transport_transfer_done_fn(void *usr, void *buffer, size_t size) {
	
}

void camera_transport_frame_done_fn(void *usr) {
	
}
