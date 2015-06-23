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

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "mavlink_bridge_header.h"
#include <mavlink.h>
#include "dcmi.h"
#include "debug.h"

#define __INLINE inline
#define __ASM asm
#include "core_cm4_simd.h"

#define FRAME_SIZE	BOTTOM_FLOW_IMAGE_WIDTH
#define SEARCH_SIZE	BOTTOM_FLOW_SEARCH_WINDOW_SIZE // maximum offset to search: 4 + 1/2 pixels
#define TILE_SIZE	8               						// x & y tile size
#define NUM_BLOCKS	3 // x & y number of tiles to check

//this are the settings for KLT based flow
#define PYR_LVLS 2
#define HALF_PATCH_SIZE 4       //this is half the wanted patch size minus 1
#define PATCH_SIZE (HALF_PATCH_SIZE*2+1)

float Jx[PATCH_SIZE*PATCH_SIZE];
float Jy[PATCH_SIZE*PATCH_SIZE];

#define sign(x) (( x > 0 ) - ( x < 0 ))

// compliments of Adam Williams
#define ABSDIFF(frame1, frame2) \
({ \
 int result = 0; \
 asm volatile( \
  "mov %[result], #0\n"           /* accumulator */ \
 \
  "ldr r4, [%[src], #0]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #0]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #4]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #4]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  "ldr r4, [%[src], #(64 * 1)]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #(64 * 1)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #(64 * 1 + 4)]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #(64 * 1 + 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  "ldr r4, [%[src], #(64 * 2)]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #(64 * 2)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #(64 * 2 + 4)]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #(64 * 2 + 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  "ldr r4, [%[src], #(64 * 3)]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #(64 * 3)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #(64 * 3 + 4)]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #(64 * 3 + 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  "ldr r4, [%[src], #(64 * 4)]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #(64 * 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #(64 * 4 + 4)]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #(64 * 4 + 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  "ldr r4, [%[src], #(64 * 5)]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #(64 * 5)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #(64 * 5 + 4)]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #(64 * 5 + 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  "ldr r4, [%[src], #(64 * 6)]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #(64 * 6)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #(64 * 6 + 4)]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #(64 * 6 + 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  "ldr r4, [%[src], #(64 * 7)]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #(64 * 7)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #(64 * 7 + 4)]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #(64 * 7 + 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  : [result] "+r" (result) \
  : [src] "r" (frame1), [dst] "r" (frame2) \
  : "r4", "r5" \
  ); \
  \
 result; \
})

/**
 * @brief Compute the average pixel gradient of all horizontal and vertical steps
 *
 * TODO compute_diff is not appropriate for low-light mode images
 *
 * @param image ...
 * @param offX x coordinate of upper left corner of 8x8 pattern in image
 * @param offY y coordinate of upper left corner of 8x8 pattern in image
 */
static inline uint32_t compute_diff(uint8_t *image, uint16_t offX, uint16_t offY, uint16_t row_size)
{
	/* calculate position in image buffer */
	uint16_t off = (offY + 2) * row_size + (offX + 2); // we calc only the 4x4 pattern
	uint32_t acc;

	/* calc row diff */
	acc = __USAD8 (*((uint32_t*) &image[off + 0 + 0 * row_size]), *((uint32_t*) &image[off + 0 + 1 * row_size]));
	acc = __USADA8(*((uint32_t*) &image[off + 0 + 1 * row_size]), *((uint32_t*) &image[off + 0 + 2 * row_size]), acc);
	acc = __USADA8(*((uint32_t*) &image[off + 0 + 2 * row_size]), *((uint32_t*) &image[off + 0 + 3 * row_size]), acc);

	/* we need to get columns */
	uint32_t col1 = (image[off + 0 + 0 * row_size] << 24) | image[off + 1 + 0 * row_size] << 16 | image[off + 2 + 0 * row_size] << 8 | image[off + 3 + 0 * row_size];
	uint32_t col2 = (image[off + 0 + 1 * row_size] << 24) | image[off + 1 + 1 * row_size] << 16 | image[off + 2 + 1 * row_size] << 8 | image[off + 3 + 1 * row_size];
	uint32_t col3 = (image[off + 0 + 2 * row_size] << 24) | image[off + 1 + 2 * row_size] << 16 | image[off + 2 + 2 * row_size] << 8 | image[off + 3 + 2 * row_size];
	uint32_t col4 = (image[off + 0 + 3 * row_size] << 24) | image[off + 1 + 3 * row_size] << 16 | image[off + 2 + 3 * row_size] << 8 | image[off + 3 + 3 * row_size];

	/* calc column diff */
	acc = __USADA8(col1, col2, acc);
	acc = __USADA8(col2, col3, acc);
	acc = __USADA8(col3, col4, acc);

	return acc;

}

/**
 * @brief Compute SAD distances of subpixel shift of two 8x8 pixel patterns.
 *
 * @param image1 ...
 * @param image2 ...
 * @param off1X x coordinate of upper left corner of pattern in image1
 * @param off1Y y coordinate of upper left corner of pattern in image1
 * @param off2X x coordinate of upper left corner of pattern in image2
 * @param off2Y y coordinate of upper left corner of pattern in image2
 * @param acc array to store SAD distances for shift in every direction
 */
static inline uint32_t compute_subpixel(uint8_t *image1, uint8_t *image2, uint16_t off1X, uint16_t off1Y, uint16_t off2X, uint16_t off2Y, uint32_t *acc, uint16_t row_size)
{
	/* calculate position in image buffer */
	uint16_t off1 = off1Y * row_size + off1X; // image1
	uint16_t off2 = off2Y * row_size + off2X; // image2

	uint32_t s0, s1, s2, s3, s4, s5, s6, s7, t1, t3, t5, t7;

	for (uint16_t i = 0; i < 8; i++)
	{
		acc[i] = 0;
	}


	/*
	 * calculate for each pixel in the 8x8 field with upper left corner (off1X / off1Y)
	 * every iteration is one line of the 8x8 field.
	 *
	 *  + - + - + - + - + - + - + - + - +
	 *  |   |   |   |   |   |   |   |   |
	 *  + - + - + - + - + - + - + - + - +
	 *
	 *
	 */

	for (uint16_t i = 0; i < 8; i++)
	{
		/*
		 * first column of 4 pixels:
		 *
		 *  + - + - + - + - + - + - + - + - +
		 *  | x | x | x | x |   |   |   |   |
		 *  + - + - + - + - + - + - + - + - +
		 *
		 * the 8 s values are from following positions for each pixel (X):
		 *  + - + - + - +
		 *  +   5   7   +
		 *  + - + 6 + - +
		 *  +   4 X 0   +
		 *  + - + 2 + - +
		 *  +   3   1   +
		 *  + - + - + - +
		 *
		 *  variables (s1, ...) contains all 4 results (32bit -> 4 * 8bit values)
		 *
		 */

		/* compute average of two pixel values */
		s0 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 1 + (i+0) * row_size])));
		s1 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i+1) * row_size]), *((uint32_t*) &image2[off2 + 1 + (i+1) * row_size])));
		s2 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 0 + (i+1) * row_size])));
		s3 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i+1) * row_size]), *((uint32_t*) &image2[off2 - 1 + (i+1) * row_size])));
		s4 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i+0) * row_size]), *((uint32_t*) &image2[off2 - 1 + (i+0) * row_size])));
		s5 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i-1) * row_size]), *((uint32_t*) &image2[off2 - 1 + (i-1) * row_size])));
		s6 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 0 + (i-1) * row_size])));
		s7 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i-1) * row_size]), *((uint32_t*) &image2[off2 + 1 + (i-1) * row_size])));

		/* these 4 t values are from the corners around the center pixel */
		t1 = (__UHADD8(s0, s1));
		t3 = (__UHADD8(s3, s4));
		t5 = (__UHADD8(s4, s5));
		t7 = (__UHADD8(s7, s0));

		/*
		 * finally we got all 8 subpixels (s0, t1, s2, t3, s4, t5, s6, t7):
		 *  + - + - + - +
		 *  |   |   |   |
		 *  + - 5 6 7 - +
		 *  |   4 X 0   |
		 *  + - 3 2 1 - +
		 *  |   |   |   |
		 *  + - + - + - +
		 */

		/* fill accumulation vector */
		acc[0] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), s0, acc[0]);
		acc[1] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), t1, acc[1]);
		acc[2] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), s2, acc[2]);
		acc[3] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), t3, acc[3]);
		acc[4] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), s4, acc[4]);
		acc[5] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), t5, acc[5]);
		acc[6] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), s6, acc[6]);
		acc[7] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), t7, acc[7]);

		/*
		 * same for second column of 4 pixels:
		 *
		 *  + - + - + - + - + - + - + - + - +
		 *  |   |   |   |   | x | x | x | x |
		 *  + - + - + - + - + - + - + - + - +
		 *
		 */

		s0 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 5 + (i+0) * row_size])));
		s1 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i+1) * row_size]), *((uint32_t*) &image2[off2 + 5 + (i+1) * row_size])));
		s2 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 4 + (i+1) * row_size])));
		s3 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i+1) * row_size]), *((uint32_t*) &image2[off2 + 3 + (i+1) * row_size])));
		s4 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 3 + (i+0) * row_size])));
		s5 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i-1) * row_size]), *((uint32_t*) &image2[off2 + 3 + (i-1) * row_size])));
		s6 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 4 + (i-1) * row_size])));
		s7 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i-1) * row_size]), *((uint32_t*) &image2[off2 + 5 + (i-1) * row_size])));

		t1 = (__UHADD8(s0, s1));
		t3 = (__UHADD8(s3, s4));
		t5 = (__UHADD8(s4, s5));
		t7 = (__UHADD8(s7, s0));

		acc[0] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), s0, acc[0]);
		acc[1] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), t1, acc[1]);
		acc[2] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), s2, acc[2]);
		acc[3] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), t3, acc[3]);
		acc[4] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), s4, acc[4]);
		acc[5] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), t5, acc[5]);
		acc[6] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), s6, acc[6]);
		acc[7] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), t7, acc[7]);
	}

	return 0;
}

/**
 * @brief Computes pixel flow from image1 to image2
 *
 * Searches the corresponding position in the new image (image2) of max. 64 pixels from the old image (image1)
 * and calculates the average offset of all.
 *
 * @param image1 previous image buffer
 * @param image2 current image buffer (new)
 * @param x_rate gyro x rate
 * @param y_rate gyro y rate
 * @param z_rate gyro z rate
 *
 * @return quality of flow calculation
 */
uint8_t compute_flow(uint8_t *image1, uint8_t *image2, float x_rate, float y_rate, float z_rate, float *pixel_flow_x, float *pixel_flow_y) {

	/* constants */
	const int16_t winmin = -SEARCH_SIZE;
	const int16_t winmax = SEARCH_SIZE;
	const uint16_t hist_size = 2*(winmax-winmin+1)+1;

	/* variables */
        uint16_t pixLo = SEARCH_SIZE + 1;
        uint16_t pixHi = FRAME_SIZE - (SEARCH_SIZE + 1);
        uint16_t pixStep = (pixHi - pixLo) / (NUM_BLOCKS-1);
	uint16_t i, j;
	uint32_t acc[8]; // subpixels
	uint16_t histx[hist_size]; // counter for x shift
	uint16_t histy[hist_size]; // counter for y shift
	int8_t  dirsx[64]; // shift directions in x
	int8_t  dirsy[64]; // shift directions in y
	uint8_t  subdirs[64]; // shift directions of best subpixels
	float meanflowx = 0.0f;
	float meanflowy = 0.0f;
	uint16_t meancount = 0;
	float histflowx = 0.0f;
	float histflowy = 0.0f;

	int xxx = 0;

	/* initialize with 0 */
	for (j = 0; j < hist_size; j++) { histx[j] = 0; histy[j] = 0; }

	/* iterate over all patterns
	 */
	for (j = pixLo; j < pixHi; j += pixStep)
	{
		for (i = pixLo; i < pixHi; i += pixStep)
		{
		  xxx++;
			/* test pixel if it is suitable for flow tracking */
			uint32_t diff = compute_diff(image1, i, j, (uint16_t) global_data.param[PARAM_IMAGE_WIDTH]);
			if (diff < global_data.param[PARAM_BOTTOM_FLOW_FEATURE_THRESHOLD])
			{
				continue;
			}

			uint32_t dist = 0xFFFFFFFF; // set initial distance to "infinity"
			int8_t sumx = 0;
			int8_t sumy = 0;
			int8_t ii, jj;

			uint8_t *base1 = image1 + j * (uint16_t) global_data.param[PARAM_IMAGE_WIDTH] + i;

			for (jj = winmin; jj <= winmax; jj++)
			{
				uint8_t *base2 = image2 + (j+jj) * (uint16_t) global_data.param[PARAM_IMAGE_WIDTH] + i;

				for (ii = winmin; ii <= winmax; ii++)
				{
//					uint32_t temp_dist = compute_sad_8x8(image1, image2, i, j, i + ii, j + jj, (uint16_t) global_data.param[PARAM_IMAGE_WIDTH]);
					uint32_t temp_dist = ABSDIFF(base1, base2 + ii);
					if (temp_dist < dist)
					{
						sumx = ii;
						sumy = jj;
						dist = temp_dist;
					}
				}
			}

			/* acceptance SAD distance threshhold */
			if (dist < global_data.param[PARAM_BOTTOM_FLOW_VALUE_THRESHOLD])
			{
				meanflowx += (float) sumx;
				meanflowy += (float) sumy;

				compute_subpixel(image1, image2, i, j, i + sumx, j + sumy, acc, (uint16_t) global_data.param[PARAM_IMAGE_WIDTH]);
				uint32_t mindist = dist; // best SAD until now
				uint8_t mindir = 8; // direction 8 for no direction
				for(uint8_t k = 0; k < 8; k++)
				{
					if (acc[k] < mindist)
					{
						// SAD becomes better in direction k
						mindist = acc[k];
						mindir = k;
					}
				}
				dirsx[meancount] = sumx;
				dirsy[meancount] = sumy;
				subdirs[meancount] = mindir;
				meancount++;

				/* feed histogram filter*/
				uint8_t hist_index_x = 2*sumx + (winmax-winmin+1);
				if (subdirs[i] == 0 || subdirs[i] == 1 || subdirs[i] == 7) hist_index_x += 1;
				if (subdirs[i] == 3 || subdirs[i] == 4 || subdirs[i] == 5) hist_index_x += -1;
				uint8_t hist_index_y = 2*sumy + (winmax-winmin+1);
				if (subdirs[i] == 5 || subdirs[i] == 6 || subdirs[i] == 7) hist_index_y += 1;
				if (subdirs[i] == 1 || subdirs[i] == 2 || subdirs[i] == 3) hist_index_y += -1;

				histx[hist_index_x]++;
				histy[hist_index_y]++;

			}
		}
	}

	/* create flow image if needed (image1 is not needed anymore)
	 * -> can be used for debugging purpose
	 */
//	if (global_data.param[PARAM_USB_SEND_VIDEO] )//&& global_data.param[PARAM_VIDEO_USB_MODE] == FLOW_VIDEO)
//	{
//
//		for (j = pixLo; j < pixHi; j += pixStep)
//		{
//			for (i = pixLo; i < pixHi; i += pixStep)
//			{
//
//				uint32_t diff = compute_diff(image1, i, j, (uint16_t) global_data.param[PARAM_IMAGE_WIDTH]);
//				if (diff > global_data.param[PARAM_BOTTOM_FLOW_FEATURE_THRESHOLD])
//				{
//					image1[j * ((uint16_t) global_data.param[PARAM_IMAGE_WIDTH]) + i] = 255;
//				}
//
//			}
//		}
//	}

	/* evaluate flow calculation */
	if (meancount > 10)
	{
		meanflowx /= meancount;
		meanflowy /= meancount;

		int16_t maxpositionx = 0;
		int16_t maxpositiony = 0;
		uint16_t maxvaluex = 0;
		uint16_t maxvaluey = 0;

		/* position of maximal histogram peek */
		for (j = 0; j < hist_size; j++)
		{
			if (histx[j] > maxvaluex)
			{
				maxvaluex = histx[j];
				maxpositionx = j;
			}
			if (histy[j] > maxvaluey)
			{
				maxvaluey = histy[j];
				maxpositiony = j;
			}
		}

		/* check if there is a peak value in histogram */
		if (1) //(histx[maxpositionx] > meancount / 6 && histy[maxpositiony] > meancount / 6)
		{
			if (global_data.param[PARAM_BOTTOM_FLOW_HIST_FILTER])
			{

				/* use histogram filter peek value */
				uint16_t hist_x_min = maxpositionx;
				uint16_t hist_x_max = maxpositionx;
				uint16_t hist_y_min = maxpositiony;
				uint16_t hist_y_max = maxpositiony;

				/* x direction */
				if (maxpositionx > 1 && maxpositionx < hist_size-2)
				{
					hist_x_min = maxpositionx - 2;
					hist_x_max = maxpositionx + 2;
				}
				else if (maxpositionx == 0)
				{
					hist_x_min = maxpositionx;
					hist_x_max = maxpositionx + 2;
				}
				else  if (maxpositionx == hist_size-1)
				{
					hist_x_min = maxpositionx - 2;
					hist_x_max = maxpositionx;
				}
				else if (maxpositionx == 1)
				{
					hist_x_min = maxpositionx - 1;
					hist_x_max = maxpositionx + 2;
				}
				else  if (maxpositionx == hist_size-2)
				{
					hist_x_min = maxpositionx - 2;
					hist_x_max = maxpositionx + 1;
				}

				/* y direction */
				if (maxpositiony > 1 && maxpositiony < hist_size-2)
				{
					hist_y_min = maxpositiony - 2;
					hist_y_max = maxpositiony + 2;
				}
				else if (maxpositiony == 0)
				{
					hist_y_min = maxpositiony;
					hist_y_max = maxpositiony + 2;
				}
				else if (maxpositiony == hist_size-1)
				{
					hist_y_min = maxpositiony - 2;
					hist_y_max = maxpositiony;
				}
				else if (maxpositiony == 1)
				{
					hist_y_min = maxpositiony - 1;
					hist_y_max = maxpositiony + 2;
				}
				else if (maxpositiony == hist_size-2)
				{
					hist_y_min = maxpositiony - 2;
					hist_y_max = maxpositiony + 1;
				}

				float hist_x_value = 0.0f;
				float hist_x_weight = 0.0f;

				float hist_y_value = 0.0f;
				float hist_y_weight = 0.0f;

				for (uint8_t i = hist_x_min; i < hist_x_max+1; i++)
				{
					hist_x_value += (float) (i*histx[i]);
					hist_x_weight += (float) histx[i];
				}

				for (uint8_t i = hist_y_min; i<hist_y_max+1; i++)
				{
					hist_y_value += (float) (i*histy[i]);
					hist_y_weight += (float) histy[i];
				}

				histflowx = (hist_x_value/hist_x_weight - (winmax-winmin+1)) / 2.0f ;
				histflowy = (hist_y_value/hist_y_weight - (winmax-winmin+1)) / 2.0f;

			}
			else
			{

				/* use average of accepted flow values */
				uint32_t meancount_x = 0;
				uint32_t meancount_y = 0;

				for (uint8_t i = 0; i < meancount; i++)
				{
					float subdirx = 0.0f;
					if (subdirs[i] == 0 || subdirs[i] == 1 || subdirs[i] == 7) subdirx = 0.5f;

					if (subdirs[i] == 3 || subdirs[i] == 4 || subdirs[i] == 5) subdirx = -0.5f;
					histflowx += (float)dirsx[i] + subdirx;
					meancount_x++;

					float subdiry = 0.0f;
					if (subdirs[i] == 5 || subdirs[i] == 6 || subdirs[i] == 7) subdiry = 0.5f;
					if (subdirs[i] == 1 || subdirs[i] == 2 || subdirs[i] == 3) subdiry = -0.5f;
					histflowy += (float)dirsy[i] + subdiry;
					meancount_y++;
				}

				histflowx /= meancount_x;
				histflowy /= meancount_y;

			}

      /* write results */
      *pixel_flow_x = histflowx;
      *pixel_flow_y = histflowy;
		}
		else
		{
			*pixel_flow_x = 0.0f;
			*pixel_flow_y = 0.0f;
			return 0;
		}
	}
	else
	{
		*pixel_flow_x = 0.0f;
		*pixel_flow_y = 0.0f;
		return 0;
	}

	/* calc quality */
	uint8_t qual = (uint8_t)(meancount * 255 / (NUM_BLOCKS*NUM_BLOCKS));

	return qual;
}


/**
 * @brief Computes pixel flow from image1 to image2
 *
 * Searches the corresponding position in the new image (image2) of max. 64 pixels from the old image (image1)
 * with the KLT method and outputs the average value of all flow vectors
 *
 * @param image1 previous image buffer
 * @param image2 current image buffer (new)
 * @param x_rate gyro x rate
 * @param y_rate gyro y rate
 * @param z_rate gyro z rate
 *
 * @return quality of flow calculation
 */
uint8_t compute_klt(uint8_t *image1, uint8_t *image2, float x_rate, float y_rate, float z_rate, float *pixel_flow_x, float *pixel_flow_y)
{
  /* variables */
  uint16_t i, j;

  float meanflowx = 0.0f;
  float meanflowy = 0.0f;
  uint16_t meancount = 0;

  /*
   * compute image pyramid for current frame
   * there is 188*120 bytes per buffer, we are only using 64*64 per buffer,
   * so just add the pyramid levels after the image
   */
  //first compute the offsets in the memory for the pyramid levels
  uint16_t lvl_off[PYR_LVLS];
  uint16_t s = FRAME_SIZE;
  uint16_t off = 0;
  for (int l = 0; l < PYR_LVLS; l++)
  {
    lvl_off[l] = off;
    off += s*s;
    s /= 2;
  }

  //then subsample the images consecutively, no blurring is done before the subsampling (if someone volunteers, please go ahead...)
  for (int l = 1; l < PYR_LVLS; l++)
  {
    uint16_t src_size = FRAME_SIZE >> (l-1);
    uint16_t tar_size = FRAME_SIZE >> l;
    uint8_t *source = &image2[lvl_off[l-1]]; //pointer to the beginning of the previous level
    uint8_t *target = &image2[lvl_off[l]];   //pointer to the beginning of the current level
    for (j = 0; j < tar_size; j++)
      for (i = 0; i < tar_size; i+=2)
      {
        //subsample the image by 2, use the halving-add instruction to do so
        uint32_t l1 = (__UHADD8(*((uint32_t*) &source[(j*2+0)*src_size + i*2]), *((uint32_t*) &source[(j*2+0)*src_size + i*2+1])));
        uint32_t l2 = (__UHADD8(*((uint32_t*) &source[(j*2+1)*src_size + i*2]), *((uint32_t*) &source[(j*2+1)*src_size + i*2+1])));
        uint32_t r = __UHADD8(l1, l2);

        //the first and the third byte are the values we want to have
        target[j*tar_size + i+0] = (uint8_t) r;
        target[j*tar_size + i+1] = (uint8_t) (r>>16);
      }
  }

  //need to store the flow values between pyramid level changes
  float us[NUM_BLOCKS*NUM_BLOCKS];
  float vs[NUM_BLOCKS*NUM_BLOCKS];
  uint16_t is[NUM_BLOCKS*NUM_BLOCKS];
  uint16_t js[NUM_BLOCKS*NUM_BLOCKS];


  //initialize flow values with the pixel value of the previous image
  uint16_t pixLo = FRAME_SIZE / (NUM_BLOCKS + 1);
  if (pixLo < PATCH_SIZE) pixLo = PYR_LVLS * PATCH_SIZE;
  uint16_t pixHi = (uint16_t)FRAME_SIZE * NUM_BLOCKS / (NUM_BLOCKS + 1);
  if (pixHi > (FRAME_SIZE - PATCH_SIZE)) pixHi = FRAME_SIZE - (PYR_LVLS * PATCH_SIZE);
  uint16_t pixStep = (pixHi - pixLo) / (NUM_BLOCKS-1);

  j = pixLo;
  for (int y = 0; y < NUM_BLOCKS; y++, j += pixStep)
  {
    i = pixLo;
    for (int x = 0; x < NUM_BLOCKS; x++, i += pixStep)
    {
      //TODO: for proper rotation compensation, insert gyro values here and then substract at the end
      us[y*NUM_BLOCKS+x] = i; //position in new image at level 0
      vs[y*NUM_BLOCKS+x] = j;
      is[y*NUM_BLOCKS+x] = i; //position in previous image  at level 0
      js[y*NUM_BLOCKS+x] = j;
    }
  }

  //for all pyramid levels, start from the smallest level
  for (int l = PYR_LVLS-1; l >= 0; l--)
  {
    //iterate over all patterns
    for (int k = 0; k < NUM_BLOCKS*NUM_BLOCKS; k++)
    {
      uint16_t i = is[k] >> l;  //reference pixel for the current level
      uint16_t j = js[k] >> l;

      uint16_t iwidth = FRAME_SIZE >> l;
      uint8_t *base1 = image1 + lvl_off[l] + j * iwidth + i;

      float JTJ[4];   //the 2x2 Hessian
      JTJ[0] = 0;
      JTJ[1] = 0;
      JTJ[2] = 0;
      JTJ[3] = 0;
      int c = 0;

      //compute jacobians and the hessian for the patch at the current location
      for (int8_t jj = -HALF_PATCH_SIZE; jj <= HALF_PATCH_SIZE; jj++)
      {
        uint8_t *left = base1 + jj*iwidth;
        for (int8_t ii = -HALF_PATCH_SIZE; ii <= HALF_PATCH_SIZE; ii++)
        {
          const float jx = ((uint16_t)left[ii+1] - (uint16_t)left[ii-1]) * 0.5f;
          const float jy = ((uint16_t)left[ii+iwidth] - (uint16_t)left[ii-iwidth]) * 0.5f;
          Jx[c] = jx;
          Jy[c] = jy;
          JTJ[0] += jx*jx;
          JTJ[1] += jx*jy;
          JTJ[2] += jx*jy;
          JTJ[3] += jy*jy;
          c++;
        }
      }

      // us and vs store the sample position in level 0 pixel coordinates
      float u = (us[k] / (1<<l));
      float v = (vs[k] / (1<<l));

      float chi_sq_previous = 0.f;

      //Now do some Gauss-Newton iterations for flow
      for (int iters = 0; iters < 5; iters++)
      {
        float JTe_x = 0;  //accumulators for Jac transposed times error
        float JTe_y = 0;

        uint8_t *base2 = image2 + lvl_off[l] + (uint16_t)v * iwidth + (uint16_t)u;

        //extract bilinearly filtered pixel values for the current location in image2
        float dX = u - floorf(u);
        float dY = v - floorf(v);
        float fMixTL = (1.f - dX) * (1.f - dY);
        float fMixTR = (dX) * (1.f - dY);
        float fMixBL = (1.f - dX) * (dY);
        float fMixBR = (dX) * (dY);

        float chi_sq = 0.f;
        int c = 0;
        for (int8_t jj = -HALF_PATCH_SIZE; jj <= HALF_PATCH_SIZE; jj++)
        {
          uint8_t *left1 = base1 + jj*iwidth;
          uint8_t *left2 = base2 + jj*iwidth;

          for (int8_t ii = -HALF_PATCH_SIZE; ii <= HALF_PATCH_SIZE; ii++)
          {
            float fPixel = fMixTL * left2[ii] + fMixTR * left2[ii+1] + fMixBL * left2[ii+iwidth] + fMixBR * left2[ii+iwidth+1];
            float fDiff = fPixel - left1[ii];
            JTe_x += fDiff * Jx[c];
            JTe_y += fDiff * Jy[c];
            chi_sq += fDiff*fDiff;
            c++;
          }
        }

        //only update if the error got smaller
        if (iters == 0 || chi_sq_previous > chi_sq)
        {
          //compute inverse of hessian
          float det = (JTJ[0]*JTJ[3]-JTJ[1]*JTJ[2]);
          if (det != 0.f)
          {
            float detinv = 1.f / det;
            float JTJinv[4];
            JTJinv[0] = detinv * JTJ[3];
            JTJinv[1] = detinv * -JTJ[1];
            JTJinv[2] = detinv * -JTJ[2];
            JTJinv[3] = detinv * JTJ[0];

            //compute update and shift current position accordingly
            float updx = JTJinv[0]*JTe_x + JTJinv[1]*JTe_y;
            float updy = JTJinv[2]*JTe_x + JTJinv[3]*JTe_y;
            float new_u = u-updx;
            float new_v = v-updy;

            //check if we drifted outside the image
            if (((int16_t)new_u < HALF_PATCH_SIZE) || (int16_t)new_u > (iwidth-HALF_PATCH_SIZE-1) || ((int16_t)new_v < HALF_PATCH_SIZE) || (int16_t)new_v > (iwidth-HALF_PATCH_SIZE-1))
            {
              break;
            }
            else
            {
              u = new_u;
              v = new_v;
            }
          }
          else
            break;
        }
        else
          break;

        chi_sq_previous = chi_sq;
      }

      if (l > 0)
      {
        us[k] = u * (1<<l);
        vs[k] = v * (1<<l);
      }
      else  //for the last, level compute the actual flow in pixels
      {
        float nx = i-u;
        float ny = j-v;
        //TODO: check if patch drifted too far - take number of pyramid levels in to account for that
        {
          meanflowx += nx;
          meanflowy += ny;
          meancount++;
        }
      }
    }
  }

  /* compute mean flow */
  if (meancount > 0)
  {
    meanflowx /= meancount;
    meanflowy /= meancount;

    *pixel_flow_x = meanflowx;
    *pixel_flow_y = meanflowy;
  }
  else
  {
    *pixel_flow_x = 0.0f;
    *pixel_flow_y = 0.0f;
    return 0;
  }

  /* return quality */
  return (uint8_t)(meancount * 255 / (NUM_BLOCKS*NUM_BLOCKS));
}
