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
#include <stdbool.h>
#include <stddef.h>

/**
 * The maximum number of buffers the camera driver supports.
 */
#define CONFIG_CAMERA_MAX_BUFFER_COUNT 5

/**
 * Number of bits to use for the histogram bins.
 * This affects the speed of the algorithm as well as the possible granularity of the other configuration options.
 */
#define CONFIG_CAMERA_EXPOSURE_BIN_BITS (5u)
#define CONFIG_CAMERA_EXPOSURE_BIN_COUNT (1u << CONFIG_CAMERA_EXPOSURE_BIN_BITS)
/**
 * The ratio of pixels (in percent) that need to fall in the highest histogram bin to trigger
 * the extreme overexpose condition. This is intended to quickly react to highly over-exposed images 
 * by decreasing the exposure time quickly.
 */
#define CONFIG_CAMERA_EXTREME_OVEREXPOSE_RATIO (20u)
/**
 * The ratio of pixels (in percent) that are rejected as outliers when evaluating the histogram starting from
 * the brightest pixel.
 */
#define CONFIG_CAMERA_OUTLIER_RATIO (5u)
/**
 * The value that the algorithm uses as a target pixel intensity when adjusting the exposure time.
 * The algorithm compares the extracted value from the histogram (after outlier rejection) to this value
 * and adjusts the exposure time accordingly.
 */
#define CONFIG_CAMERA_DESIRED_EXPOSURE_8B (200u)
/**
 * The size of the exposure tolerance band.
 * If there is a certain amount of change in the image brightness the tolerance band will decide after
 * how much brightness delta the exposure algorithm will start to re-adjust the exposure time.
 */
#define CONFIG_CAMERA_DESIRED_EXPOSURE_TOL_8B (24)
/**
 * The smoothing factor of the exposure time. This is the constant of an exponential filter.
 * 
 * new = old * K + (1 - K) * new
 * 
 * This constant influences the speed with which the algorithm reacts to brightness changes.
 */
#define CONFIG_CAMERA_EXPOSURE_SMOOTHING_K (0.83f)
/**
 * The interval between updating the exposure value in number of frames. Snapshot and invalid frames are skipped.
 */
#define CONFIG_CAMERA_EXPOSURE_UPDATE_INTERVAL (4)

struct _camera_sensor_interface;
typedef struct _camera_sensor_interface camera_sensor_interface;

struct _camera_transport_interface;
typedef struct _camera_transport_interface camera_transport_interface;

struct _camera_ctx;
typedef struct _camera_ctx camera_ctx;

typedef struct _camera_img_param {
	struct _size {
		uint16_t x;		///< Image size in x direction.
		uint16_t y;		///< Image size in y direction.
	} size;				/**< Image size. 
						 *   The image size is constrained by the transfer size of the camera_transport_interface.
						 *   The image size in bytes must be an exact multiple of the transfer size 
						 *   as well as at least two times the transfer size. */
	uint8_t binning;	///< Column and row binning ratio.
} camera_img_param;

/**	
 * Struct holding image parameters for the camera.
 */
typedef struct _camera_img_param_ex {
	camera_img_param p;	///< User editable parameters.
	uint32_t exposure;	///< Exposure time in master clock times.
	float analog_gain;	///< Analog gain value. 1 - 4.
} camera_img_param_ex;

/**
 * Struct holding information about an image and a pointer to the buffer itself.
 */
typedef struct _camera_image_buffer {
	camera_img_param_ex param;	///< The parameters of the image that is stored in the buffer.
	uint32_t frame_number;		///< The frame number. This number increases with every frame that is captured from the camera module.
	uint32_t timestamp;			///< Frame timestamp in microseconds.
	void *buffer;				///< Pointer to the buffer itself.
	size_t buffer_size;			///< Maximum size of the buffer. Note: this is not the actual size of the image in the buffer.
	uint32_t meta;				///< Buffer meta data not used by the camera driver. This can be used to identify buffers.
} camera_image_buffer;

#define BuildCameraImageBuffer(memory_variable) ((const camera_image_buffer){.buffer = memory_variable, .buffer_size = sizeof(memory_variable)})

/**
 * Callback which is called when a snapshot capture has finished.
 * @note This callback may be called from an interrupt handler.
 * @param success True when capture was successful, false if it failed due to desynchronization. 
 *				  You should retry in the mainloop after calling camera_snapshot_acknowledge.
 */
typedef void (*camera_snapshot_done_cb)(bool success);

/**
 * Initializes the camera driver.
 * @param ctx		The context to use.
 * @param sensor	The sensor interface to use.
 * @param transport	The sensor data transport interface to use.
 * @param exposure_min_clks Minimum exposure in clocks.
 * @param exposure_max_clks Maximum exposure in clocks.
 * @param analog_gain_max Maximum analog gain. > 1 depending on what the sensor supports. (mt9v034: 1-4)
 * @param img_param	The initial image parameters to use for img_stream operation mode.
 * @param buffers	Array of initialized buffers to use for buffering the images in img_stream mode.
 *					In each camera_image_buffer the buffer and buffer_size members must be correctly set.
 *					The camera_img_stream_get_buffers function will return pointers to one of these (copied) buffers.
 *					The buffer can reside in the CCM of the microcontroller.
 *					The structs are copied to an internal data-structure.
 * @param buffer_count Number of buffers that are passed in buffers.
 *					Must be lower than or equal the configured CAMERA_MAX_BUFFER_COUNT. 
 *					There must be at least one more buffer than what is requested in camera_img_stream_get_buffers.
 *					More buffers will reduce the latency when frames are skipped.
 * @return			True when successful.
 */
bool camera_init(camera_ctx *ctx, const camera_sensor_interface *sensor, const camera_transport_interface *transport,
				 uint32_t exposure_min_clks, uint32_t exposure_max_clks, float analog_gain_max,
				 const camera_img_param *img_param,
				 camera_image_buffer buffers[], size_t buffer_count);


/**
 * Reconfigures the general camera parameters.
 * Call this after some MAVLINK parameters have changed.
 * @param ctx		The context to use.
 */
void camera_reconfigure_general(camera_ctx *ctx);

/**
 * Schedules the new parameters to take effect as soon as possible. This function makes sure that 
 * parameter changes are commited to the sensor in a time window which will guarantee glitch free operation.
 * @param ctx		The context to use.
 * @param img_param	The new image parameters to use. Note that the image must still fit inside the buffers passed 
 *					to camera_init.
 * @return	True on success.
 */
bool camera_img_stream_schedule_param_change(camera_ctx *ctx, const camera_img_param *img_param);

/**
 * Gets the most recent images. If no more recent image is available this function returns immediately without doing anything.
 * @param ctx		The context to use.
 * @param buffers	Pointer to an array of buffer pointers which are updated to point to the most recent buffers.
 *					The first buffer pointer will be updated to the most recent image followed by the next
 *					pointer which is updated to the next older image and so on.
 *					It is guaranteed to retrieve consecutive images.
 * @param count		Number of most recent images to retrieve. Must be lower than buffer_count - 1 which was 
 *					passed to camera_init.
 * @param check_for_new When true the function will return true only if a new frame is pending. It will then clear the
                        pending flag.
 * @return	true when a set of count consecutive most recent images have been retrieved.
            false when two consecutive images are not ready
 * @note			When this function is successful (return value 0) the buffers need to be returned to the camera driver before 
 *					requesting new buffers. (use camera_img_stream_return_buffers)
 */
bool camera_img_stream_get_buffers(camera_ctx *ctx, camera_image_buffer *buffers[], size_t count, bool wait_for_new);

/**
 * Returns the buffers that have been retrieved by camera_img_stream_get_buffers back to the camera driver.
 * This function must be called when image processing has been finished.
 * @param ctx		The context to use.
 * @param buffers	Pointer to an array of buffer pointers which will be returned to the camera driver.
 * @param count		Number of buffer pointers in the array.
 */
void camera_img_stream_return_buffers(camera_ctx *ctx, camera_image_buffer *buffers[], size_t count);

/**
 * Schedules a snapshot to be taken with the camera. The snapshot will be scheduled as soon as possible.
 * Only one snapshot can be scheduled at a time.
 * @param ctx		The context to use.
 * @param img_param	The image parameters for the snapshot image.
 * @param dst		The destination buffer which should receive the image.
 *					The buffer and buffer_size members must be correctly set.
 *					The pointed variable must remain valid until the snapshot capture is done.
 * @param cb		Callback function which is called when the snapshot has been taken.
 *					Note that the callback is called from the interrupt context and should only do minimal stuff
 *					like setting a flag to notify the main loop. Calling camera_* functions is not allowed.
 *					camera_snapshot_acknowledge must be called as soon as possible afterwards in the mainloop 
 *					regardless of the success status of the snapshot.
 * @return			True when snapshot has been successfully scheduled.
 */
bool camera_snapshot_schedule(camera_ctx *ctx, const camera_img_param *img_param, camera_image_buffer *dst, camera_snapshot_done_cb cb);

/**
 * This function must be called right after the snapshot callback function has been called. No other calls to the camera driver are allowed.
 * @note	This function must not be called directly from the callback!
 * @param ctx		The context to use.
 */
void camera_snapshot_acknowledge(camera_ctx *ctx);

/** 
 * Camera sensor configuration interface.
 * 
 * The functions inside this interface are divided into two sets of function classes:
 * 
 * A)  init, prepare_update_param, reconfigure_general
 * These functions may only be called from the mainloop and have a possible long run-time. They may be interrupted by any of the class B functions.
 * The sensor interface has to make sure that there will be no conflict when a class B function starts to run while a class A function is not finished yet.
 * 
 * B)  restore_previous_param, notify_readout_start, notify_readout_end, update_exposure_param, get_current_param
 * These functions may be called from an interrupt context and may interrupt class A functions. They have to make sure to not access resources in use 
 * by class A functions.
 * 
 * The camera driver will make sure that only one function of each class is entered at the same time and that class B functions are never interrupted by class A functions.
 * 
 * 
 * The sequence of functions that are called is the following one:
 * 
 * === First buffer of a new frame has arrived ===
 * notify_readout_start()
 * get_current_param() -> returned parameters are used to calculate the number of buffers we need to receive for this frame.
 * restore_previous_param() (optional if switching back to the previous parameters is desired)
 * 
 * === ... receiving the buffers between the first and the last ... ===
 * 
 * === Last buffer of the current frame has arrived ===
 * notify_readout_end()
 * update_exposure_param() (optional if the exposure parameters have been updated)
 * 
 *  ==> repeat
 * 
 */
struct _camera_sensor_interface {
	void *usr;		///< User pointer that should be passed to the following interface functions.
	/**
	 * Initializes the sensor and the hardware of the microcontroller.
	 * @param usr		User pointer from this struct.
	 * @param img_param	The image parameters to use for initialization.
	 * @return true on success.
	 */
	bool (*init)(void *usr, const camera_img_param_ex *img_param);
	/**
	 * Prepares the sensor to switch to new parameters.
	 * This function should perform most of the work that is needed to update the sensor with new parameters.
	 * @param usr		User pointer from this struct.
	 * @param img_param	The new image parameters.
	 * @return true on success.
	 */
	bool (*prepare_update_param)(void *usr, const camera_img_param_ex *img_param);
	/**
	 * Reconfigures the general sensor parameters.
	 */
	void (*reconfigure_general)(void *usr);
	/**
	 * Immediately switches back to the previous parameters.
	 * @param usr		User pointer from this struct.
	 */
	void (*restore_previous_param)(void *usr);
	/**
	 * Called every frame just after readout has started (but not completed yet).
	 * This function may be used to switch the sensor to new parameters that have been previously prepared.
	 * @param usr		User pointer from this struct.
	 * @note  This function may be called from an interrupt vector and should do as little work as necessary.
	 */
	void (*notify_readout_start)(void *usr);
	/**
	 * Called every frame just after readout has finished.
	 * @param usr		User pointer from this struct.
	 * @note  This function may be called from an interrupt vector and should do as little work as necessary.
	 */
	void (*notify_readout_end)(void *usr);
	/**
	 * Called at the end of the readout after notify_readout_end to update the exposure parameters of the current context.
	 * @return Return true if it was possible to perform the update.
	 */
	bool (*update_exposure_param)(void *usr, uint32_t exposure, float gain);
	/**
	 * Called to retrieve the image parameters of the current frame that is being output.
	 * This function is called after notify_readout_start to retrieve the
	 * parameters that were in effect at the time the image was taken.
	 * @param  img_data_valid boolean which receives a flag whether the image data is valid or not.
	 */
	void (*get_current_param)(void *usr, camera_img_param_ex *img_param, bool *img_data_valid);
};

/**
 * Callback for notifying the camera driver about a completed transfer.
 * @param usr		The user data pointer that has been specified in the init function. (cb_usr)
 * @param buffer	The buffer that contains the data of the transfer was completed.
 * @param size		The size of the transfer.
 */
typedef void (*camera_transport_transfer_done_cb)(void *usr, const void *buffer, size_t size);

/**
 * Callback for notifying the camera driver about a completed frame.
 * This callback should be called AFTER the last camera_transport_transfer_done_cb call of the frame.
 * @param usr		The user data pointer that has been specified in the init function. (cb_usr)
 * @param probably_infront_dma A hint to the camera driver that the frame done call might be just in front of the transfer done call.
 *					It is up to the camera driver to interpret it.
 */
typedef void (*camera_transport_frame_done_cb)(void *usr, bool probably_infront_dma);

/** Camera image transport interface.
 */
struct _camera_transport_interface {
	void *usr;		///< User pointer that should be passed to the following interface functions.
	size_t transfer_size;	///< Transfer size of this transport.
	/**
	 * Initializes the sensor and the hardware of the microcontroller.
	 * @param usr		User pointer from this struct.
	 * @param transfer_done_cb Callback which should be called when a transfer has been completed.
	 * @param frame_done_cb Callback which should be called when a frame has been completed.
	 * @param cb_usr    Callback user data pointer which should be passed to the transfer_done_cb
	 *					and frame_done_cb callbacks.
	 * @return True on success.
	 */
	bool (*init)(void *usr, 
				 camera_transport_transfer_done_cb transfer_done_cb, 
				 camera_transport_frame_done_cb frame_done_cb,
				 void *cb_usr);
	/**
	 * This function is called to reset the transport layer to align it with the frame boundaries.
	 * It is called in the frame_done_cb after a few frames when we can be sure that the camera module has the correct resolution.
	 */
	void (*reset)(void *usr);
};

/**
 * The camera driver context struct.
 */
struct _camera_ctx {
	/* startup control */
	volatile int resync_discard_frame_count;				///< Number of frames to discard at startup.
	
	/* assets of the camera driver */
	
	const camera_sensor_interface *sensor;					///< Sensor interface.
	const camera_transport_interface *transport;			///< Transport interface.
	
	uint32_t exposure_min_clks;								///< Minimum exposure in clocks.
	uint32_t exposure_max_clks;								///< Maximum exposure in clocks.
	float analog_gain_max;									///< Maximum gain.
	
	/* exposure control */
	
	float analog_gain;										///< Current analog gain.
	uint32_t exposure;										///< Current exposure in clocks.
	
	float exposure_smoothing;								///< Exposure smoothing variable.
	
	int exposure_skip_frame_cnt;							///< Counter to skip frames between exposure calculations.
	
	uint16_t exposure_sampling_stride;						///< Image sampling stride used for calculating exposure.
	uint16_t exposure_bins[CONFIG_CAMERA_EXPOSURE_BIN_COUNT];///< Histogram of the sampled pixels.
	uint16_t exposure_hist_count;							///< Number of sampled pixels in the histogram.
	int last_brightness;									///< The last brightness value that has been extracted from the histogram.
	
	/* image streaming buffer and parameters */
	
	camera_img_param_ex img_stream_param;					///< The parameters of the image streaming mode.
	camera_img_param img_stream_param_pend;					///< The pending image streaming mode parameters.
	camera_image_buffer buffers[CONFIG_CAMERA_MAX_BUFFER_COUNT];///< The image buffers for image stream mode.
	size_t buffer_count;										///< Total number of buffers.
	volatile uint8_t buf_avail[CONFIG_CAMERA_MAX_BUFFER_COUNT];///< Indexes to the buffers that are available. Ordered in the MRU order.
	volatile uint8_t buf_avail_count;						///< Number of buffer indexes in the avail_bufs array.
	volatile uint8_t buf_put_back_pos;						///< Position where to put back the reserved buffers.
	
	volatile bool new_frame_arrived;						///< Flag which is set by the interrupt handler to notify that a new frame has arrived.
	volatile bool buffers_are_reserved;						///< True when buffers have been reserved by the camera_img_stream_get_buffers function and need to be returned.
	
	/* image snapshot buffer and parameters */
	
	camera_img_param_ex snapshot_param;						///< The parameters of the snapshot mode.
	camera_image_buffer *snapshot_buffer;					///< Pointer to buffer which receives the snapshot. NULL when no snapshot is pending.
	camera_snapshot_done_cb snapshot_cb;					///< Callback pointer which is called when the snapshot is complete.
	
	/* frame acquisition */
	
	camera_img_param_ex cur_frame_param;					///< Parameters of the frame we are currently receiving.
	uint32_t cur_frame_number;								///< Number of the frame we are currently receiving.
	camera_image_buffer *cur_frame_target_buf;				///< Target buffer of the frame we are currently receiving. When NULL we are discarding the frame.
	int cur_frame_target_buf_idx;							///< When the target buffer points to one of the image stream buffers this is set to the index of that buffer. -1 otherwise.
	uint32_t cur_frame_size;								///< The number of bytes that need to be received until the frame is complete.
	uint32_t cur_frame_pos;									///< The current byte position inside the frame that is beeing received.
	
	/* sequencing */
	
	int frame_done_infront_count;
	
	volatile bool seq_frame_receiving;						///< True when we are currently receiving a frame.
	volatile bool seq_updating_sensor;						///< True when the mainloop is currently calling sensor functions to update the sensor context.
	volatile bool seq_repeat_updating_sensor;				/**< This variable is set when the interrupt hanlder could not update the exposure settings because seq_updating_sensor was set.
															 *   After the mainloop has finished updating the sensor context, it should repeat it immediatly with new exposure settings. */
	volatile bool seq_snapshot_active;						/**< Flag that is asserted as long as a snapshot is not finished.
															 *   While asserted the camera_img_stream_schedule_param_change function
															 *   should not update the sensor. It should write the new parameters to the img_stream_param_pend variable. */
	volatile bool seq_pend_reconfigure_general;				/**< Flag signalling that a general reconfiguration is pending because seq_snapshot_active was active. */
	volatile bool seq_pend_img_stream_param;				/**< Flag that is set when pending img stream parameter updates are in the img_stream_param_pend variable.
															 *   These updates will be written to the sensor in the camera_snapshot_acknowledge function. */
};

#endif /* CAMERA_H_ */