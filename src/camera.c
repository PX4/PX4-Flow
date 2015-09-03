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

#define abs(x) ({			\
	typeof(x) __x = (x);	\
	if (__x < 0) __x = -__x;\
	__x;					\
})


void camera_transport_transfer_done_fn(void *usr, const void *buffer, size_t size);
void camera_transport_frame_done_fn(void *usr, bool probably_infront_dma);

bool camera_init(camera_ctx *ctx, const camera_sensor_interface *sensor, const camera_transport_interface *transport,
				 uint32_t exposure_min_clks, uint32_t exposure_max_clks, float analog_gain_max,
				 const  camera_img_param *img_param,
				 camera_image_buffer buffers[], size_t buffer_count) {
	memset(ctx, 0, sizeof(camera_ctx));
	ctx->sensor    = sensor;
	ctx->transport = transport;
	// check parameter:
	if (buffer_count > CONFIG_CAMERA_MAX_BUFFER_COUNT || buffer_count < 2) {
		return false;
	}
	uint32_t img_size = (uint32_t)img_param->size.x * (uint32_t)img_param->size.y;
	if (img_size % ctx->transport->transfer_size != 0 || img_size / ctx->transport->transfer_size < 2) {
		// invalid size parameter!
		return false;
	}
	// initialize state:
	ctx->resync_discard_frame_count = -1;
	ctx->exposure_min_clks = exposure_min_clks;
	ctx->exposure_max_clks = exposure_max_clks;
	ctx->analog_gain_max   = analog_gain_max;
	ctx->analog_gain       = 1;
	ctx->exposure          = ctx->exposure_min_clks;
	ctx->exposure_smoothing = ctx->exposure_min_clks;
	ctx->exposure_sampling_stride = 0;
	ctx->exposure_skip_frame_cnt = 0;
	memset(ctx->exposure_bins, 0, sizeof(ctx->exposure_bins));
	ctx->exposure_hist_count = 0;
	ctx->img_stream_param.p = *img_param;
	ctx->img_stream_param.analog_gain = ctx->analog_gain;
	ctx->img_stream_param.exposure = ctx->exposure;
	ctx->snapshot_param = ctx->img_stream_param;
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
		ctx->buffers[i].param = ctx->img_stream_param;
		memset(ctx->buffers[i].buffer, 0, img_size);
		// init the avail_bufs array:
		ctx->buf_avail[i] = i;
	}
	ctx->buffer_count    = buffer_count;
	ctx->buf_avail_count = buffer_count;
	ctx->new_frame_arrived = false;
	
	ctx->snapshot_buffer = NULL;
	
	ctx->frame_done_infront_count = 0;
	
	ctx->cur_frame_number = buffer_count + 1;
	ctx->seq_frame_receiving = false;
	ctx->cur_frame_target_buf   = NULL;
	
	ctx->seq_snapshot_active                 = false;
	ctx->seq_pend_img_stream_param        = false;
	// initialize hardware:
	if (!ctx->transport->init(ctx->transport->usr,
							  camera_transport_transfer_done_fn,
							  camera_transport_frame_done_fn,
							  ctx)) {
		return false;
	}
	if (!ctx->sensor->init(ctx->sensor->usr, &ctx->img_stream_param)) {
		return false;
	}
	/* after initialization start the discard count down! */
	ctx->resync_discard_frame_count = 16;
	return true;
}

static void uint32_t_memcpy(uint32_t *dst, const uint32_t *src, size_t count) {
	int i;
	for (i = 0; i < count; ++i) {
		dst[i] = src[i];
	}
}

static bool camera_buffer_fifo_remove_front(camera_ctx *ctx, int *out, size_t count) {
	size_t bc = ctx->buf_avail_count;
	size_t i;
	if (count > bc) return false;
	// read out:
	for (i = 0; i < count; ++i) {
		*out++ = ctx->buf_avail[i];
	}
	// close gap:
	for (i = count; i < bc; ++i) {
		ctx->buf_avail[i - count] = ctx->buf_avail[i];
	}
	ctx->buf_avail_count = bc - count;
	return true;
}

static bool camera_buffer_fifo_remove_back(camera_ctx *ctx, int *out, size_t count) {
	size_t bc = ctx->buf_avail_count;
	size_t i;
	if (count > bc) return false;
	// read out:
	for (i = bc - count; i < bc; ++i) {
		*out++ = ctx->buf_avail[i];
	}
	// reduce count:
	ctx->buf_avail_count = bc - count;
	return true;
}

static void camera_buffer_fifo_push_at(camera_ctx *ctx, size_t pos, const int *in, size_t count) {
	size_t bc = ctx->buf_avail_count;
	size_t i;
	// move away:
	for (i = bc; i > pos; --i) {
		ctx->buf_avail[i - 1 + count] = ctx->buf_avail[i - 1];
	}
	// fill in:
	for (i = pos; i < pos + count; ++i) {
		ctx->buf_avail[i] = *in++;
	}
	// update count:
	ctx->buf_avail_count = bc + count;
}

static void camera_update_exposure_hist(camera_ctx *ctx, const uint8_t *buffer, size_t size) {
	if (ctx->exposure_sampling_stride > 0) {
		int i;
		int c = 0;
		for (i = ctx->exposure_sampling_stride / 2; i < size; i += ctx->exposure_sampling_stride) {
			int idx = (buffer[i] >> (8u - CONFIG_CAMERA_EXPOSURE_BIN_BITS));
			ctx->exposure_bins[idx]++;
			c++;
		}
		ctx->exposure_hist_count += c;
	}
}

static int camera_exposure_hist_extract_brightness_8b(camera_ctx *ctx) {
	/* test for over-exposed image: */
	int value = CONFIG_CAMERA_EXPOSURE_BIN_COUNT - 1;
	if ((uint32_t)ctx->exposure_bins[value] * 100u >
	   ((uint32_t)ctx->exposure_hist_count) * CONFIG_CAMERA_EXTREME_OVEREXPOSE_RATIO) {
		return 256;
	}
	/* find the bin where the outliers have been ignored: */
	uint32_t pix_rem = ((uint32_t)ctx->exposure_hist_count * (uint32_t)CONFIG_CAMERA_OUTLIER_RATIO) / 100u;
	while (ctx->exposure_bins[value] < pix_rem && value > 0) {
		pix_rem -= ctx->exposure_bins[value];
		value--;
	}
	/* expand to 8bits: */
	uint32_t bin_size = 1 << (8u - CONFIG_CAMERA_EXPOSURE_BIN_BITS);
	value  = value * bin_size;
	return value;
}

void camera_transport_transfer_done_fn(void *usr, const void *buffer, size_t size) {
	camera_ctx *ctx = (camera_ctx *)usr;
	if (ctx->resync_discard_frame_count != 0) {
		return;
	}
	if (ctx->seq_frame_receiving) {
		camera_update_exposure_hist(ctx, buffer, size);
		// check if we have a target buffer:
		if (ctx->cur_frame_target_buf != NULL) {
			uint32_t_memcpy((uint32_t *)((uint8_t *)ctx->cur_frame_target_buf->buffer + ctx->cur_frame_pos), 
							(const uint32_t *)buffer, size / 4);
		}
		// update current position:
		ctx->cur_frame_pos += size;
		// check if we are done:
		if (ctx->cur_frame_pos >= ctx->cur_frame_size) {
			// frame done!
			ctx->sensor->notify_readout_end(ctx->sensor->usr);
			if (ctx->cur_frame_target_buf_idx >= 0) {
				// put back into available buffers (in the front)
				camera_buffer_fifo_push_at(ctx, 0, &ctx->cur_frame_target_buf_idx, 1);
				if (ctx->buf_put_back_pos < ctx->buf_avail_count) {
					ctx->buf_put_back_pos += 1;
				}
				// notify camera_img_stream_get_buffers function.
				ctx->new_frame_arrived = true;
			} else if (ctx->seq_snapshot_active && ctx->cur_frame_target_buf != NULL) {
				// snapshot has been taken!
				ctx->snapshot_cb(true);
				ctx->seq_snapshot_active = false;
			}
			/* handle auto-exposure: */
			if (ctx->exposure_sampling_stride > 0) {
				float exposure = (float)ctx->cur_frame_param.exposure * ctx->cur_frame_param.analog_gain;
				// analyze histogram:
				int brightness = camera_exposure_hist_extract_brightness_8b(ctx);
				ctx->last_brightness = brightness;
				if (brightness >= 256) {
					// extremely over-exposed! halve the integration time:
					exposure = exposure * 0.33f;
				} else {
					int desired_bright = CONFIG_CAMERA_DESIRED_EXPOSURE_8B;
					desired_bright &= (0xFF << (8u - CONFIG_CAMERA_EXPOSURE_BIN_BITS));
					if (brightness < desired_bright / 4) {
						// extremely under-exposed! double the integration time:
						exposure =  exposure * 4.f;
					} else {
						if (abs(desired_bright - brightness) > CONFIG_CAMERA_DESIRED_EXPOSURE_TOL_8B) {
							// determine optimal exposure for next frame:
							exposure = (exposure * desired_bright) / brightness;
						}
					}
				}
				/* clip the value within bounds: */
				if (exposure < (float)ctx->exposure_min_clks) {
					exposure = ctx->exposure_min_clks;
				} else if (exposure > (float)ctx->exposure_max_clks * ctx->analog_gain_max) {
					exposure = (float)ctx->exposure_max_clks * ctx->analog_gain_max;
				}
				ctx->exposure_smoothing = CONFIG_CAMERA_EXPOSURE_SMOOTHING_K * ctx->exposure_smoothing + 
								  (1.0f - CONFIG_CAMERA_EXPOSURE_SMOOTHING_K) * exposure;
				/* update the exposure and analog gain values: */
				if (ctx->exposure_smoothing > ctx->exposure_max_clks) {
					ctx->exposure = ctx->exposure_max_clks;
					ctx->analog_gain = ctx->exposure_smoothing / (float)ctx->exposure_max_clks;
				} else {
					ctx->exposure = ctx->exposure_smoothing;
					ctx->analog_gain = 1;
				}
				/* update the sensor! */
				if (!ctx->seq_updating_sensor) {
					/* update it ourself! */
					ctx->sensor->update_exposure_param(ctx->sensor->usr, ctx->exposure, ctx->analog_gain);
				} else {
					ctx->seq_repeat_updating_sensor = true;
				}
			}
			// reset state:
			ctx->cur_frame_target_buf = NULL;
			ctx->seq_frame_receiving = false;
		}
	} else {
		// no frame currently as the target frame. it must be the beginning of a new frame then!
		// update the sensor: (this might trigger some I2C transfers)
		ctx->sensor->notify_readout_start(ctx->sensor->usr);
		// get current sensor parameter:
		bool img_data_valid;
		ctx->sensor->get_current_param(ctx->sensor->usr, &ctx->cur_frame_param, &img_data_valid);
		// update the receiving variables:
		ctx->cur_frame_number += 1;
		ctx->seq_frame_receiving = true;
		ctx->cur_frame_target_buf   = NULL;
		ctx->cur_frame_size  = (uint32_t)ctx->cur_frame_param.p.size.x * (uint32_t)ctx->cur_frame_param.p.size.y;
		ctx->cur_frame_pos   = size;
		ctx->cur_frame_target_buf_idx = -1;
		if (!ctx->seq_snapshot_active) {
			// check that the size parameters match to the img stream:
			if (ctx->cur_frame_param.p.size.x  == ctx->img_stream_param.p.size.x &&
				ctx->cur_frame_param.p.size.y  == ctx->img_stream_param.p.size.y) {
				if (img_data_valid) {
					// get least recently used buffer from the available buffers:
					if (camera_buffer_fifo_remove_back(ctx, &ctx->cur_frame_target_buf_idx, 1)) {
						ctx->cur_frame_target_buf   = &ctx->buffers[ctx->cur_frame_target_buf_idx];
						/* if there are no more buffers in the fifo reset the flag again: */
						if (ctx->buf_avail_count == 0) {
							ctx->new_frame_arrived = false;
						}
					} else {
						ctx->cur_frame_target_buf_idx = -1;
					}
				}
			}
		} else {
			// check that the size parameters match to the snapshot parameters:
			if (ctx->cur_frame_param.p.size.x  == ctx->snapshot_param.p.size.x &&
				ctx->cur_frame_param.p.size.y  == ctx->snapshot_param.p.size.y &&
				ctx->cur_frame_param.p.binning == ctx->snapshot_param.p.binning) {
				if (img_data_valid) {
					// get the buffer:
					ctx->cur_frame_target_buf   = ctx->snapshot_buffer;
					// initiate switching back to img stream mode:
					ctx->sensor->restore_previous_param(ctx->sensor->usr);
				}
			}
		}
		// initialize the target buffer:
		if (ctx->cur_frame_target_buf != NULL) {
			ctx->cur_frame_target_buf->timestamp    = get_boot_time_us();
			ctx->cur_frame_target_buf->frame_number = ctx->cur_frame_number;
			ctx->cur_frame_target_buf->param        = ctx->cur_frame_param;
			// write data to it: (at position 0)
			uint32_t_memcpy((uint32_t *)ctx->cur_frame_target_buf->buffer, (uint32_t *)buffer, size / 4);
		}
		// initialize exposure measuring (do not process snapshot images)
		if (img_data_valid && !ctx->seq_snapshot_active) {
			ctx->exposure_skip_frame_cnt++;
			if (ctx->exposure_skip_frame_cnt >= CONFIG_CAMERA_EXPOSURE_UPDATE_INTERVAL) {
				ctx->exposure_skip_frame_cnt = 0;
				/* make sure no more than 65535 pixels are sampled: */
				uint16_t skip = ctx->cur_frame_size / 65536u + 1;
				if (skip < 6) {
					skip = 6;
				}
				ctx->exposure_sampling_stride = skip;
				// reset histogram:
				ctx->exposure_hist_count = 0;
				memset(ctx->exposure_bins, 0, sizeof(ctx->exposure_bins));
			} else {
				ctx->exposure_sampling_stride = 0;
			}
		} else {
			ctx->exposure_sampling_stride = 0;
		}
		camera_update_exposure_hist(ctx, (const uint8_t *)buffer, size);
	}
}

void camera_transport_frame_done_fn(void *usr, bool probably_infront_dma) {
	camera_ctx *ctx = (camera_ctx *)usr;
	/* we will only pay attention to the probably_infront_dma flag if we are in normal operating mode. */
	int fdc = ctx->resync_discard_frame_count;
	if (fdc > 0) {
		fdc--;
		ctx->resync_discard_frame_count = fdc;
		/* re-initialize the transport twice */
		if (fdc == 0 || fdc == 1) {
			LEDOff(LED_ERR);
			ctx->transport->reset(ctx->transport->usr);
		}
	} else {
		/* we trust the probably infront DMA flag 3 times in a row. */
		if (!probably_infront_dma || ctx->frame_done_infront_count > 3) {
			ctx->frame_done_infront_count = 0;
			/* if we don't trust it anymore we trigger a re-sync */
			if (ctx->seq_frame_receiving || probably_infront_dma) {
				/* we are out of sync! abort current frame: */
				if (ctx->cur_frame_target_buf_idx >= 0) {
					// put back into available buffers (in the back)
					uint16_t ac_b = ctx->buf_avail_count;
					camera_buffer_fifo_push_at(ctx, ac_b, &ctx->cur_frame_target_buf_idx, 1);
					// set frame number something much older than the last frame:
					if (ac_b > 0) {
						int last_b  = ctx->buf_avail[ac_b - 1];
						int this_b = ctx->buf_avail[ac_b];
						ctx->buffers[this_b].frame_number = ctx->buffers[last_b].frame_number - ctx->buffer_count - 1;
					} else {
						ctx->new_frame_arrived = false;
					}
				} else if (ctx->seq_snapshot_active && ctx->cur_frame_target_buf != NULL) {
					// snapshot abort!
					ctx->snapshot_cb(false);
					ctx->seq_snapshot_active = false;
				}
				// reset state:
				ctx->cur_frame_target_buf = NULL;
				ctx->seq_frame_receiving = false;
				/* initiate resynchronization: */
				ctx->resync_discard_frame_count = 2;

				LEDOn(LED_ERR);
			}
		} else {
			ctx->frame_done_infront_count++;
		}
	}
}

static bool camera_update_sensor_param(camera_ctx *ctx, const camera_img_param *img_param, camera_img_param_ex *ptoup) {
	camera_img_param_ex p = *ptoup;
	p.p = *img_param;
	bool result = false;
	do {
		ctx->seq_repeat_updating_sensor = false;
		ctx->seq_updating_sensor = true;
		/* update exposure and analog gain: */
		p.exposure    = ctx->exposure;
		p.analog_gain = ctx->analog_gain;
		/* write: */
		if (ctx->sensor->prepare_update_param(ctx->sensor->usr, &p)) {
			*ptoup = p;
			result = true;
		} else {
			/* restore previous: */
			p = *ptoup;
		}
		ctx->seq_updating_sensor = false;
	} while (ctx->seq_repeat_updating_sensor);
	return result;
}

void camera_reconfigure_general(camera_ctx *ctx) {
	if (!ctx->seq_snapshot_active && !ctx->seq_pend_reconfigure_general) {
		ctx->sensor->reconfigure_general(ctx->sensor->usr);
	} else {
		ctx->seq_pend_reconfigure_general = true;
	}
}

bool camera_img_stream_schedule_param_change(camera_ctx *ctx, const camera_img_param *img_param) {
	uint32_t img_size = (uint32_t)img_param->size.x * (uint32_t)img_param->size.y;
	if (img_size % ctx->transport->transfer_size != 0 || img_size / ctx->transport->transfer_size < 2) {
		// invalid size parameter!
		return false;
	}
	int i;
	for (i = 0; i < ctx->buffer_count; ++i) {
		// check image size against each buffer:
		if (img_size > ctx->buffers[i].buffer_size) {
			return false;
		}
	}
	if (!ctx->seq_snapshot_active && !ctx->seq_pend_img_stream_param) {
		return camera_update_sensor_param(ctx, img_param, &ctx->img_stream_param);
	} else {
		ctx->img_stream_param_pend = *img_param;
		ctx->seq_pend_img_stream_param = true;
		return true;
	}
	return false;
}

bool camera_snapshot_schedule(camera_ctx *ctx, const camera_img_param *img_param, camera_image_buffer *dst, camera_snapshot_done_cb cb) {
	uint32_t img_size = (uint32_t)img_param->size.x * (uint32_t)img_param->size.y;
	if (img_size % ctx->transport->transfer_size != 0 || img_size / ctx->transport->transfer_size < 2) {
		// invalid size parameter!
		return false;
	}
	// check image size against the buffer:
	if (img_size > dst->buffer_size || dst->buffer == NULL) {
		return false;
	}
	if (!ctx->seq_snapshot_active) {
		if (camera_update_sensor_param(ctx, img_param, &ctx->snapshot_param)) {
			ctx->snapshot_buffer = dst;
			ctx->snapshot_cb     = cb;
			ctx->seq_snapshot_active = true;
			return true;
		}
	}
	return false;
}

void camera_snapshot_acknowledge(camera_ctx *ctx) {
	if (!ctx->seq_snapshot_active) {
		if (ctx->seq_pend_reconfigure_general) {
			ctx->seq_pend_reconfigure_general = false;
			ctx->sensor->reconfigure_general(ctx->sensor->usr);
		}
		if (ctx->seq_pend_img_stream_param) {
			ctx->seq_pend_img_stream_param = false;
			/* write the pending changes to the sensor: */
			camera_update_sensor_param(ctx, &ctx->img_stream_param_pend, &ctx->img_stream_param);
		}
	}
}

static bool camera_img_stream_get_buffers_idx(camera_ctx *ctx, int bidx[], size_t count, bool reset_new_frm) {
	int i;
	__disable_irq();
	if (!camera_buffer_fifo_remove_front(ctx, bidx, count)) {
		if (reset_new_frm) {
			ctx->new_frame_arrived = false;
		}
		__enable_irq();
		return false;
	}
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
		ctx->buf_put_back_pos = 0;
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
	size_t at_pos = ctx->buf_put_back_pos;
	if (at_pos > ctx->buf_avail_count) at_pos = ctx->buf_avail_count;
	camera_buffer_fifo_push_at(ctx, ctx->buf_put_back_pos, bidx, count);
	ctx->buffers_are_reserved = false;
	__enable_irq();
}
