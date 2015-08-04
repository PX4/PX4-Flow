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

#ifndef CAMERA_H_
#define CAMERA_H_

#include <stdint.h>

#define CAMERA_MAX_BUFFER_COUNT 5

struct _camera_sensor_interface;
typedef struct _camera_sensor_interface camera_sensor_interface;

struct _camera_transport_interface;
typedef struct _camera_transport_interface camera_transport_interface;

struct _camera_ctx;
typedef struct _camera_ctx camera_ctx;

/**	
 * Struct holding image parameters for the camera.
 */
typedef struct _camera_img_param {
	struct _size {
		uint16_t x;		///< Image size in x direction.
		uint16_t y;		///< Image size in y direction.
	} size;				/**< Image size. 
						 *   The image size is constrained by the transfer size of the camera_transport_interface.
						 *   The image size in bytes must be an exact multiple of the transfer size 
						 *   as well as at least two times the transfer size. */
	uint8_t col_bin;	///< Column binning ratio. (x direction)
	uint8_t row_bin;	///< Row binning ration.   (y direction)
} camera_img_param;

/**
 * Struct holding information about an image and a pointer to the buffer itself.
 */
typedef struct _camera_image_buffer {
	camera_img_param param;		///< The parameters of the image that is stored in the buffer.
	uint32_t frame_number;		///< The frame number. This number increases with every frame that is captured from the camera module.
	uint32_t timestamp;			///< Frame timestamp in microseconds.
	void *buffer;				///< Pointer to the buffer itself.
	size_t buffer_size;			///< Maximum size of the buffer. Note: this is not the actual size of the image in the buffer.
	uint32_t meta;				///< Buffer meta data not used by the camera driver. This can be used to identify buffers.
} camera_image_buffer;

/**
 * Callback which is called when a snapshot capture has finished.
 * @note This callback may be called from an interrupt handler.
 * @param Pointer to buffer which contains the snapshot.
 */
typedef void (*camera_snapshot_done_cb)(camera_image_buffer *buf);

/**
 * Initializes the camera driver.
 * @param ctx		The context to use.
 * @param sensor	The sensor interface to use.
 * @param transport	The sensor data transport interface to use.
 * @param img_param	The initial image parameters to use for img_stream operation mode.
 * @param buffers	Array of initialized buffers to use for buffering the images in img_stream mode.
 *					In each camera_image_buffer the buffer and buffer_size members must be correctly set.
 *					The camera_img_stream_get_buffers function will return pointers to one of these buffers.
 *					The buffer can reside in the CCM of the microcontroller.
 * @param buffer_count Number of buffers that are passed in buffers.
 *					Must be lower than or equal the configured CAMERA_MAX_BUFFER_COUNT. 
 *					There must be at least one more buffer than what is requested in camera_img_stream_get_buffers.
 *					More buffers will reduce the latency when frames are skipped.
 * @return			Zero when successful.
 */
int camera_init(camera_ctx *ctx, camera_sensor_interface *sensor, camera_transport_interface *transport,
				const camera_img_param *img_param,
				camera_image_buffer buffers[], size_t buffer_count);

/**
 * Schedules the new parameters to take effect as soon as possible. This function makes sure that 
 * parameter changes are commited to the sensor in a time window which will guarantee glitch free operation.
 * @param ctx		The context to use.
 * @param img_param	The new image parameters to use. Note that the image must still fit inside the buffers passed 
 *					to camera_init.
 * @return	Zero on success.
 */
int camera_img_stream_schedule_param_change(camera_ctx *ctx, const camera_img_param *img_param);

/**
 * Gets the most recent images. If no more recent image is available this function returns immediatly without doing anything.
 * @param ctx		The context to use.
 * @param buffers	Pointer to an array of buffer pointers which are updated to point to the most recent buffers.
 *					The first buffer pointer will be updated to the most recent image followed by the next
 *					pointer which is updated to the next older image and so on.
 *					It is guaranteed to retrieve consecutive images.
 * @param count		Number of most recent images to retrieve. Must be lower than buffer_count - 1 which was 
 *					passed to camera_init.
 * @return			0 when a new most recent image has been retrieved. 
 *					1 if there was no new most recent image since the last call to this function. (In this case nothing has been updated)
 *					-1 on error.
 * @note			When this function is successful the buffers need to be returned to the camera driver before 
 *					requesting new buffers.
 */
int camera_img_stream_get_buffers(camera_ctx *ctx, camera_image_buffer **buffers[], size_t count);

/**
 * Returns the buffers that have been retrieved by camera_img_stream_get_buffers back to the camera driver.
 * This function must be called when image processing has been finished.
 * @param ctx		The context to use.
 * @param buffers	Pointer to an array of buffer pointers which will be returned to the camera driver.
 * @param count		Number of buffer pointers in the array.
 */
void camera_img_stream_return_buffers(camera_ctx *ctx, camera_image_buffer **buffers[], size_t count);

/**
 * Schedules a snapshot to be taken with the camera. The snapshot will be scheduled as soon as possible.
 * Only one snapshot can be scheduled at a time.
 * @param ctx		The context to use.
 * @param img_param	The image parameters for the snapshot image.
 * @param dst		The destination buffer which should receive the image.
 *					The buffer and buffer_size members must be correctly set.
 * @param cb		Callback function which is called when the snapshot has been taken.
 *					Note that the callback is called from the interrupt context and should only do minimal stuff
 *					like setting a flag to notify the main loop. Calling camera_* functions is not allowed.
 * @return			Zero when snapshot has been successfully scheduled.
 */
int camera_snapshot_schedule(camera_ctx *ctx, const camera_img_param *img_param, camera_image_buffer *dst, camera_snapshot_done_cb cb);

/**
 * The camera driver context struct.
 */
struct _camera_ctx {
	camera_sensor_interface *sensor;
	camera_transport_interface *transport;
};

/** Camera sensor configuration interface.
 */
struct _camera_sensor_interface {
	void *usr;		///< User pointer that should be passed to the following interface functions.
	/**
	 * Initializes the sensor and the hardware of the microcontroller.
	 * @param usr		User pointer from this struct.
	 * @param img_param	The image parameters to use for initialization.
	 * @return 0 on success.
	 */
	int (*init)(void *usr, const camera_img_param *img_param);
	/**
	 * Prepares the sensor to switch to new parameters.
	 * This function should perform most of the work that is needed to update the sensor with new parameters.
	 * @param usr		User pointer from this struct.
	 * @param img_param	The new image parameters.
	 * @return 0 on success.
	 */
	int (*prepare_update_param)(void *usr, const camera_img_param *img_param);
	/**
	 * Switches the sensor to the new parameters that have been previously prepared.
	 * This function is called just after the camera module has started outputting a new frame.
	 * @param usr		User pointer from this struct.
	 * @return >= 0 on success: the number of frames until the changes take effect.
	 *		   < 0 on error.
	 * @note  This function may be called from an interrupt vector and should do as little work as necessary.
	 */
	int (*update_param)(void *usr);
};

/**
 * Callback for notifying the camera driver about a completed transfer.
 * @param usr		The user data pointer that has been specified in the init function. (cb_usr)
 * @param buffer	The buffer that contains the data of the transfer was completed.
 * @param size		The size of the transfer.
 */
typedef void (*camera_transport_transfer_done_cb)(void *usr, void *buffer, size_t size);

/**
 * Callback for notifying the camera driver about a completed frame.
 * @param usr		The user data pointer that has been specified in the init function. (cb_usr)
 */
typedef void (*camera_transport_frame_done_cb)(void *usr);

/** Camera image transport interface.
 */
struct _camera_transport_interface {
	void *usr;		///< User pointer that should be passed to the following interface functions.
	/**
	 * Initializes the sensor and the hardware of the microcontroller.
	 * @param usr		User pointer from this struct.
	 * @param transfer_done_cb Callback which should be called when a transfer has been completed.
	 * @param frame_done_cb Callback which should be called when a frame has been completed.
	 * @param cb_usr    Callback user data pointer which should be passed to the transfer_done_cb
	 *					and frame_done_cb callbacks.
	 * @return 0 on success.
	 */
	int (*init)(void *usr, 
				camera_transport_transfer_done_cb transfer_done_cb, 
				camera_transport_frame_done_cb frame_done_cb,
				void *cb_usr);
};

#endif /* CAMERA_H_ */