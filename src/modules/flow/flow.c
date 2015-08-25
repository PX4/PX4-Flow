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

#include "no_warnings.h"
#include "mavlink_bridge_header.h"
#include "settings.h"
#include <mavlink.h>
#include "flow.h"
#include "dcmi.h"
#include "debug.h"
#include "timer.h"

#define __INLINE inline
#define __ASM asm
#include "core_cm4_simd.h"

#define SEARCH_SIZE	global_data.param[PARAM_FLOW_MAX_PIXEL] // maximum offset to search: 4 + 1/2 pixels
#define TILE_SIZE	8               						// x & y tile size
#define NUM_BLOCKS	5 // x & y number of tiles to check
#define NUM_BLOCK_KLT 4

//this are the settings for KLT based flow
#define PYR_LVLS 2
#define HALF_PATCH_SIZE 3       //this is half the wanted patch size minus 1
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
 * @brief Computes the Hessian at a pixel location
 *
 * The hessian (second order partial derivatives of the image) is
 * a measure of the salience of the image at the appropriate
 * box filter scale. It allows to judge wether a pixel
 * location is suitable for optical flow calculation.
 *
 * @param image the array holding pixel data
 * @param x location of the pixel in x
 * @param y location of the pixel in y
 *
 * @return gradient magnitude
 */
static inline uint32_t compute_hessian_4x6(uint8_t *image, uint16_t x, uint16_t y, uint16_t row_size)
{
	// candidate for hessian calculation:
	uint16_t off1 = y*row_size + x;   	// First row of ones
	uint16_t off2 = (y+1)*row_size + x;   // Second row of ones
	uint16_t off3 = (y+2)*row_size + x;   // Third row of minus twos
	uint16_t off4 = (y+3)*row_size + x;   // Third row of minus twos
	uint16_t off5 = (y+4)*row_size + x;   // Third row of minus twos
	uint16_t off6 = (y+5)*row_size + x;   // Third row of minus twos
	uint32_t magnitude;

	// Uncentered for max. performance:
	// center pixel is in brackets ()

	//  1   1   1   1
	//  1   1   1   1
	// -2 (-2) -2  -2
	// -2  -2  -2  -2
	//  1   1   1   1
	//  1   1   1   1

	magnitude = __UADD8(*((uint32_t*) &image[off1 - 1]), *((uint32_t*) &image[off2 - 1]));
	magnitude -= 2*__UADD8(*((uint32_t*) &image[off3 - 1]), *((uint32_t*) &image[off4 - 1]));
	magnitude += __UADD8(*((uint32_t*) &image[off5 - 1]), *((uint32_t*) &image[off6 - 1]));

	return magnitude;
}


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
	uint32_t col1 = (image[off + 0 + 0 * row_size] << 24) | image[off + 0 + 1 * row_size] << 16 | image[off + 0 + 2 * row_size] << 8 | image[off + 0 + 3 * row_size];
	uint32_t col2 = (image[off + 1 + 0 * row_size] << 24) | image[off + 1 + 1 * row_size] << 16 | image[off + 1 + 2 * row_size] << 8 | image[off + 1 + 3 * row_size];
	uint32_t col3 = (image[off + 2 + 0 * row_size] << 24) | image[off + 2 + 1 * row_size] << 16 | image[off + 2 + 2 * row_size] << 8 | image[off + 2 + 3 * row_size];
	uint32_t col4 = (image[off + 3 + 0 * row_size] << 24) | image[off + 3 + 1 * row_size] << 16 | image[off + 3 + 2 * row_size] << 8 | image[off + 3 + 3 * row_size];

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
 * @brief Compute SAD of two 8x8 pixel windows.
 *
 * @param image1 ...
 * @param image2 ...
 * @param off1X x coordinate of upper left corner of pattern in image1
 * @param off1Y y coordinate of upper left corner of pattern in image1
 * @param off2X x coordinate of upper left corner of pattern in image2
 * @param off2Y y coordinate of upper left corner of pattern in image2
 */
static inline uint32_t compute_sad_8x8(uint8_t *image1, uint8_t *image2, uint16_t off1X, uint16_t off1Y, uint16_t off2X, uint16_t off2Y, uint16_t row_size)
{
	/* calculate position in image buffer */
	uint16_t off1 = off1Y * row_size + off1X; // image1
	uint16_t off2 = off2Y * row_size + off2X; // image2

	uint32_t acc;
	acc = __USAD8 (*((uint32_t*) &image1[off1 + 0 + 0 * row_size]), *((uint32_t*) &image2[off2 + 0 + 0 * row_size]));
	acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 0 * row_size]), *((uint32_t*) &image2[off2 + 4 + 0 * row_size]), acc);

	acc = __USADA8(*((uint32_t*) &image1[off1 + 0 + 1 * row_size]), *((uint32_t*) &image2[off2 + 0 + 1 * row_size]), acc);
	acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 1 * row_size]), *((uint32_t*) &image2[off2 + 4 + 1 * row_size]), acc);

	acc = __USADA8(*((uint32_t*) &image1[off1 + 0 + 2 * row_size]), *((uint32_t*) &image2[off2 + 0 + 2 * row_size]), acc);
	acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 2 * row_size]), *((uint32_t*) &image2[off2 + 4 + 2 * row_size]), acc);

	acc = __USADA8(*((uint32_t*) &image1[off1 + 0 + 3 * row_size]), *((uint32_t*) &image2[off2 + 0 + 3 * row_size]), acc);
	acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 3 * row_size]), *((uint32_t*) &image2[off2 + 4 + 3 * row_size]), acc);

	acc = __USADA8(*((uint32_t*) &image1[off1 + 0 + 4 * row_size]), *((uint32_t*) &image2[off2 + 0 + 4 * row_size]), acc);
	acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 4 * row_size]), *((uint32_t*) &image2[off2 + 4 + 4 * row_size]), acc);

	acc = __USADA8(*((uint32_t*) &image1[off1 + 0 + 5 * row_size]), *((uint32_t*) &image2[off2 + 0 + 5 * row_size]), acc);
	acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 5 * row_size]), *((uint32_t*) &image2[off2 + 4 + 5 * row_size]), acc);

	acc = __USADA8(*((uint32_t*) &image1[off1 + 0 + 6 * row_size]), *((uint32_t*) &image2[off2 + 0 + 6 * row_size]), acc);
	acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 6 * row_size]), *((uint32_t*) &image2[off2 + 4 + 6 * row_size]), acc);

	acc = __USADA8(*((uint32_t*) &image1[off1 + 0 + 7 * row_size]), *((uint32_t*) &image2[off2 + 0 + 7 * row_size]), acc);
	acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 7 * row_size]), *((uint32_t*) &image2[off2 + 4 + 7 * row_size]), acc);

	return acc;
}

uint16_t compute_flow(uint8_t *image1, uint8_t *image2, float x_rate, float y_rate, float z_rate,
					  flow_raw_result *out, uint16_t max_out)
{
	/* constants */
	const uint16_t search_size = SEARCH_SIZE;
	const uint16_t frame_size = FLOW_FRAME_SIZE;
	const int16_t winmin = -search_size;
	const int16_t winmax = search_size;

	/* variables */
	uint16_t pixLo = search_size + 1;
	uint16_t pixHi = frame_size - (search_size + 1) - TILE_SIZE;
	uint16_t pixStep = (pixHi - pixLo - 1) / NUM_BLOCKS;
	pixHi = pixLo + pixStep * NUM_BLOCKS;
	uint16_t i, j;
	uint32_t acc[8]; // subpixels

	uint16_t result_count = 0;

	/* iterate over all patterns
	 */
	for (j = pixLo; j < pixHi; j += pixStep)
	{
		for (i = pixLo; i < pixHi; i += pixStep)
		{
			/* abort if the output buffer is full */
			if (result_count >= max_out) break;
			flow_raw_result *result = &out[result_count];
			/* init the result */
			result->x = 0;
			result->y = 0;
			result->quality = 0;
			result->at_x = i + TILE_SIZE / 2;
			result->at_y = j + TILE_SIZE / 2;

			result_count++;

			/* test pixel if it is suitable for flow tracking */
			uint32_t diff = compute_diff(image1, i, j, frame_size);
			if (diff < global_data.param[PARAM_FLOW_FEATURE_THRESHOLD])
			{
				continue;
			}

			uint32_t dist = 0xFFFFFFFF; // set initial distance to "infinity"
			int8_t sumx = 0;
			int8_t sumy = 0;
			int8_t ii, jj;

			//uint8_t *base1 = image1 + j * frame_size + i;

			for (jj = winmin; jj <= winmax; jj++)
			{
				//uint8_t *base2 = image2 + (j+jj) * frame_size + i;

				for (ii = winmin; ii <= winmax; ii++)
				{
					uint32_t temp_dist = compute_sad_8x8(image1, image2, i, j, i + ii, j + jj, frame_size);
//					uint32_t temp_dist = ABSDIFF(base1, base2 + ii);
					if (temp_dist < dist)
					{
						sumx = ii;
						sumy = jj;
						dist = temp_dist;
					}
				}
			}

			/* acceptance SAD distance threshhold */
			if (dist < global_data.param[PARAM_FLOW_VALUE_THRESHOLD])
			{
				compute_subpixel(image1, image2, i, j, i + sumx, j + sumy, acc, frame_size);
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
				/* store the flow value */
				result->x = sumx;
				result->y = sumy;
				if (mindir == 0 || mindir == 1 || mindir == 7) result->x += 0.5f;
				if (mindir == 3 || mindir == 4 || mindir == 5) result->x -= 0.5f;
				if (mindir == 5 || mindir == 6 || mindir == 7) result->y -= 0.5f;
				if (mindir == 1 || mindir == 2 || mindir == 3) result->y += 0.5f;
				if (FLOAT_AS_BOOL(global_data.param[PARAM_FLOW_GYRO_COMPENSATION])) {
					/* gyro compensation */
					result->x -= x_rate;
					result->y -= y_rate;
				}
				result->quality = 1.0;
			}
		}
	}

	return result_count;
}

void klt_preprocess_image(uint8_t *image, flow_klt_image *klt_image) {
	uint16_t i, j;

	klt_image->image = image;
	
	/*
	 * compute image pyramid for current frame
	 * there is 188*120 bytes per buffer, we are only using 64*64 per buffer,
	 * so just add the pyramid levels after the image
	 */
	//first compute the offsets in the memory for the pyramid levels
	uint8_t *lvl_base[PYR_LVLS];
	uint16_t frame_size = (uint16_t)(FLOW_FRAME_SIZE+0.5);
	uint16_t s = frame_size / 2;
	uint16_t off = 0;
	lvl_base[0] = image;
	for (int l = 1; l < PYR_LVLS; l++)
	{
		lvl_base[l] = klt_image->preprocessed + off;
		off += s*s;
		s /= 2;
	}

	//then subsample the images consecutively, no blurring is done before the subsampling (if someone volunteers, please go ahead...)
	for (int l = 1; l < PYR_LVLS; l++)
	{
		uint16_t src_size = frame_size >> (l-1);
		uint16_t tar_size = frame_size >> l;
		uint8_t *source = lvl_base[l-1]; //pointer to the beginning of the previous level
		uint8_t *target = lvl_base[l];   //pointer to the beginning of the current level
		for (j = 0; j < tar_size; j++) {
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
	}
}

uint16_t compute_klt(flow_klt_image *image1, flow_klt_image *image2, float x_rate, float y_rate, float z_rate,
					 flow_raw_result *out, uint16_t max_out)
{
	/* variables */
	uint16_t i, j;

	float chi_sum = 0.0f;
	uint8_t chicount = 0;

	uint16_t max_iters = global_data.param[PARAM_KLT_MAX_ITERS];

	/*
	 * compute image pyramid for current frame
	 * there is 188*120 bytes per buffer, we are only using 64*64 per buffer,
	 * so just add the pyramid levels after the image
	 */
	//first compute the offsets in the memory for the pyramid levels
	uint8_t *lvl_base1[PYR_LVLS];
	uint8_t *lvl_base2[PYR_LVLS];
	uint16_t frame_size = (uint16_t)(FLOW_FRAME_SIZE+0.5);
	uint16_t s = frame_size / 2;
	uint16_t off = 0;
	lvl_base1[0] = image1->image;
	lvl_base2[0] = image2->image;
	for (int l = 1; l < PYR_LVLS; l++)
	{
		lvl_base1[l] = image1->preprocessed + off;
		lvl_base2[l] = image2->preprocessed + off;
		off += s*s;
		s /= 2;
	}

	//need to store the flow values between pyramid level changes
	float us[NUM_BLOCK_KLT*NUM_BLOCK_KLT];
	float vs[NUM_BLOCK_KLT*NUM_BLOCK_KLT];
	uint16_t is[NUM_BLOCK_KLT*NUM_BLOCK_KLT];
	uint16_t js[NUM_BLOCK_KLT*NUM_BLOCK_KLT];

	//initialize flow values with the pixel value of the previous image
	uint16_t topPyrStep = 1 << (PYR_LVLS - 1);

    /* 
     * if the gyro change (x_rate & y_rate) is more than the maximum pixel
     * difference that can be detected between two frames (depends on PYR_LVLS)
     * we can't calculate the flow. So return empty flow.
     */
    if(fabsf(x_rate) > (float)(2 * topPyrStep) || fabsf(y_rate) > (float)(2 * topPyrStep)){
        return 0;
    }

	uint16_t pixStep = frame_size / (NUM_BLOCK_KLT + 1);
	uint16_t pixLo = pixStep;
	/* align with topPyrStep */
	pixStep = ((uint16_t)(pixStep                 )) & ~((uint16_t)(topPyrStep - 1));	// round down
	pixLo   = ((uint16_t)(pixLo + (topPyrStep - 1))) & ~((uint16_t)(topPyrStep - 1));	// round up
	//uint16_t pixHi = pixLo + pixStep * (NUM_BLOCK_KLT - 1);

	j = pixLo;
	for (int y = 0; y < NUM_BLOCK_KLT; y++, j += pixStep)
	{
		i = pixLo;
		for (int x = 0; x < NUM_BLOCK_KLT; x++, i += pixStep)
		{
			uint16_t idx = y*NUM_BLOCK_KLT+x;
			if (FLOAT_AS_BOOL(global_data.param[PARAM_KLT_GYRO_ASSIST])) {
				/* use the gyro measurement to guess the initial position in the new image */
				us[idx] = i + x_rate; //position in new image at level 0
				vs[idx] = j + y_rate;
				if ((int16_t)us[idx] < HALF_PATCH_SIZE) 				 us[idx] = HALF_PATCH_SIZE;
				if ((int16_t)us[idx] > frame_size - HALF_PATCH_SIZE - 1) us[idx] = frame_size - HALF_PATCH_SIZE - 1;
				if ((int16_t)vs[idx] < HALF_PATCH_SIZE) 				 vs[idx] = HALF_PATCH_SIZE;
				if ((int16_t)vs[idx] > frame_size - HALF_PATCH_SIZE - 1) vs[idx] = frame_size - HALF_PATCH_SIZE - 1;
			} else {
				us[idx] = i; //position in new image at level 0
				vs[idx] = j;
			}
			is[idx] = i;	//position in previous image at level 0
			js[idx] = j;
			/* init output vector */
			if (idx < max_out) {
				out[idx].x = 0;
				out[idx].y = 0;
				out[idx].quality = 0;
				out[idx].at_x = i;
				out[idx].at_y = j;
			}
		}
	}

	//for all pyramid levels, start from the smallest level
	for (int l = PYR_LVLS-1; l >= 0; l--)
	{
		//iterate over all patterns
		for (int k = 0; k < NUM_BLOCK_KLT*NUM_BLOCK_KLT; k++)
		{
			i = is[k] >> l;  //reference pixel for the current level
			j = js[k] >> l;

			uint16_t iwidth = frame_size >> l;
			uint8_t *base1 = lvl_base1[l] + j * iwidth + i;

			float JTJ[4];   //the 2x2 Hessian
			JTJ[0] = 0;
			JTJ[1] = 0;
			JTJ[2] = 0;
			JTJ[3] = 0;
			int c = 0;

			//compute jacobians and the hessian for the patch at the current location
			uint8_t min_val = 255;
			uint8_t max_val = 0;
			for (int8_t jj = -HALF_PATCH_SIZE; jj <= HALF_PATCH_SIZE; jj++)
			{
				uint8_t *left = base1 + jj*iwidth;
				for (int8_t ii = -HALF_PATCH_SIZE; ii <= HALF_PATCH_SIZE; ii++)
				{
					uint8_t val = left[ii];
					if (val > max_val) max_val = val;
					if (val < min_val) min_val = val;
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

			//compute inverse of hessian
			float det = (JTJ[0]*JTJ[3]-JTJ[1]*JTJ[2]);
			float dyn_range = (float)(max_val - min_val) + 1;
			float trace = (JTJ[0] + JTJ[3]);
			float M_c = det - global_data.param[PARAM_ALGORITHM_CORNER_KAPPA] * trace * trace;
			if (fabsf(det) > global_data.param[PARAM_KLT_DET_VALUE_MIN] * dyn_range && M_c > 0.0f)
			{
				float detinv = 1.f / det;
				float JTJinv[4];
				JTJinv[0] = detinv * JTJ[3];
				JTJinv[1] = detinv * -JTJ[1];
				JTJinv[2] = detinv * -JTJ[2];
				JTJinv[3] = detinv * JTJ[0];

				// us and vs store the sample position in level 0 pixel coordinates
				float u = (us[k] / (1<<l));
				float v = (vs[k] / (1<<l));

				float chi_sq_previous = 0.f;

				bool result_good = true;

				//Now do some Gauss-Newton iterations for flow
				for (int iters = 0; iters < max_iters; iters++)
				{
					float JTe_x = 0;  //accumulators for Jac transposed times error
					float JTe_y = 0;

					uint8_t *base2 = lvl_base2[l] + (uint16_t)v * iwidth + (uint16_t)u;

					//extract bilinearly filtered pixel values for the current location in image2
					float dX = u - floorf(u);
					float dY = v - floorf(v);
					float fMixTL = (1.f - dX) * (1.f - dY);
					float fMixTR = (dX) * (1.f - dY);
					float fMixBL = (1.f - dX) * (dY);
					float fMixBR = (dX) * (dY);

					float chi_sq = 0.f;
					c = 0;
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
						//compute update and shift current position accordingly
						float updx = JTJinv[0]*JTe_x + JTJinv[1]*JTe_y;
						float updy = JTJinv[2]*JTe_x + JTJinv[3]*JTe_y;
						float new_u = u-updx;
						float new_v = v-updy;

						//check if we drifted outside the image
						if (((int16_t)new_u < HALF_PATCH_SIZE) || (int16_t)new_u > (iwidth-HALF_PATCH_SIZE-1) || ((int16_t)new_v < HALF_PATCH_SIZE) || (int16_t)new_v > (iwidth-HALF_PATCH_SIZE-1))
						{
							result_good = false;
							break;
						}
						else
						{
							u = new_u;
							v = new_v;
						}
					}
					else
					{
						chi_sum += chi_sq_previous;
						chicount++;
						break;
					}
					chi_sq_previous = chi_sq;
				}
				if (l > 0)
				{
					// TODO: evaluate recording failure at each level to calculate a final quality value
					us[k] = u * (1<<l);
					vs[k] = v * (1<<l);
				}
				else  //for the last level compute the actual flow in pixels
				{
					if (result_good && k < max_out) {
						if (FLOAT_AS_BOOL(global_data.param[PARAM_FLOW_GYRO_COMPENSATION])) {
							/* compute flow and compensate gyro */
							out[k].x = u - i - x_rate;
							out[k].y = v - j - y_rate;
						} else {
							out[k].x = u - i;
							out[k].y = v - j;
						}
						if (fabsf(out[k].x) < (float)(2 * topPyrStep) &&
						    fabsf(out[k].y) < (float)(2 * topPyrStep)) {
							out[k].quality = 1.0f;
						} else {
							/* drifted too far */
							out[k].x = 0;
							out[k].y = 0;
							out[k].quality = 0;
						}
					}
				}
			}
		}
	}
	return NUM_BLOCK_KLT * NUM_BLOCK_KLT < max_out ? NUM_BLOCK_KLT * NUM_BLOCK_KLT : max_out;
}

struct flow_res_dim_value {
	float value;
	uint16_t idx;
};

static int flow_res_dim_value_compare(const void* elem1, const void* elem2)
{
	float v1 = ((const struct flow_res_dim_value *)elem1)->value;
	float v2 = ((const struct flow_res_dim_value *)elem2)->value;
    if(v1 < v2)
        return -1;
    return v1 > v2;
}

uint8_t flow_extract_result(flow_raw_result *in, uint16_t result_count, float *px_flow_x, float *px_flow_y,
				float accuracy_p, float accuracy_px)
{
	/* extract all valid results: */
	struct flow_res_dim_value xvalues[result_count];
	struct flow_res_dim_value yvalues[result_count];
	uint16_t valid_c = 0;
	for (int i = 0; i < result_count; i++) {
		if (in[i].quality > 0) {
			xvalues[valid_c].value = in[i].x;
			xvalues[valid_c].idx   = i;
			yvalues[valid_c].value = in[i].y;
			yvalues[valid_c].idx   = i;
			valid_c++;
		}
	}
	if (valid_c < (result_count + 2) / 3 || valid_c < 3) {
		*px_flow_x = 0;
		*px_flow_y = 0;
		return 0;
	}
	struct flow_res_dim_value *axes[2] = {xvalues, yvalues};
	float *output[2] = {px_flow_x, px_flow_y};
	float max_spread_val[2] = {};
	float spread_val[2] = {};
	uint16_t total_avg_c = 0;
	for (int i = 0; i < 2; ++i) {
		struct flow_res_dim_value *axis = axes[i];
		/* sort them */
		qsort(axis, valid_c, sizeof(struct flow_res_dim_value), flow_res_dim_value_compare);
		spread_val[i] = axis[valid_c * 3 / 4].value - axis[valid_c / 4].value;
		/* start with one element */
		uint16_t s = valid_c / 2;
		uint16_t e = valid_c / 2 + 1;
		uint16_t s_res;
		uint16_t e_res;
		float avg_sum = axis[s].value;
		uint16_t avg_c = 1;
		float avg;
		while (1) {
			s_res = s;
			e_res = e;
			/* calculate average and maximum spread to throw away outliers */
			avg = avg_sum / avg_c;
			float max_spread = fabsf(avg) * accuracy_p;
			max_spread = accuracy_px * accuracy_px / (accuracy_px + max_spread) + max_spread;
			max_spread_val[i] = max_spread;
			/* decide on which side to add new data-point (to remain centered in the sorted set) */
			if (s > valid_c - e && s > 0) {
				s--;
				avg_sum += axis[s].value;
			} else if (e < valid_c) {
				e++;
				avg_sum += axis[e - 1].value;
			} else {
				break;
			}
			avg_c++;
			/* check maximum spread */
			if (axis[e - 1].value - axis[s].value > max_spread) break;
			/* its good. continue .. */
		}
		if (e_res - s_res > 1) {
			/* we have a result */
			*output[i] = avg;
			total_avg_c += e_res - s_res;
		} else {
			*px_flow_x = 0;
			*px_flow_y = 0;
			return 0;
		}
	}
	if (FLOAT_AS_BOOL(global_data.param[PARAM_USB_SEND_FLOW_OUTL])) {
		static int ctr = 0;
		if (ctr++ % 20 == 0) {
			mavlink_msg_debug_vect_send(MAVLINK_COMM_2, "SPREAD_X", get_boot_time_us(), spread_val[0], max_spread_val[0], *output[0]);
			mavlink_msg_debug_vect_send(MAVLINK_COMM_2, "SPREAD_Y", get_boot_time_us(), spread_val[1], max_spread_val[1], *output[1]);
		}
	}
	total_avg_c = total_avg_c / 2;
	return (total_avg_c * 255) / result_count;
}

