
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

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define __INLINE inline
#define __ASM asm
#include "core_cm4_simd.h"

#include "filter.h"

void filter_image(uint8_t *image, uint16_t width) {
	uint16_t ext_width = width + 2;
	/* first we make a copy of the image and extend it beyond the edges by 1 pixel */
	uint8_t *img_ext = image + width * width;
	#define IMG_PX(x, y) (*(image + width * (y) + (x)))
	#define EXT_PX(x, y) (*(img_ext + ext_width * (y) + (x)))
	int y, x;
	/* copy the lines from the middle: */
	for (y = 0; y < width; y++) {
		memcpy(&EXT_PX(1, y + 1), &IMG_PX(0, y), width);
	}
	/* top and bottom: */
	memcpy(&EXT_PX(1, 0),         &IMG_PX(0, 0),         width);
	memcpy(&EXT_PX(1, width + 1), &IMG_PX(0, width - 1), width);
	/* left and right: */
	for (y = 0; y < ext_width; y++) {
		EXT_PX(0, y) = EXT_PX(1, y);
		EXT_PX(ext_width - 1, y) = EXT_PX(ext_width - 2, y);
	}
	/* now compute with the following kernel:
	 * -1 -1 -1
	 * -1  8 -1
	 * -1 -1 -1   */
	for (y = 0; y < width; y++) {
		for (x = 0; x < width; x++) {
			int16_t sum = (uint16_t)EXT_PX(x, y + 0) + (uint16_t)EXT_PX(x + 1, y + 0) + (uint16_t)EXT_PX(x + 2, y + 0) +
			              (uint16_t)EXT_PX(x, y + 1) +                                + (uint16_t)EXT_PX(x + 2, y + 1) +
			              (uint16_t)EXT_PX(x, y + 2) + (uint16_t)EXT_PX(x + 1, y + 2) + (uint16_t)EXT_PX(x + 2, y + 2);
			sum -= (((uint16_t)EXT_PX(x + 1, y + 1)) << 3);
			/* the result will be between -255 * 8 to 255 * 8: */
			/* amplification could be done here. */
			/* clip the result to -128 to 127: */
			if      (sum < -128) sum = -128;
			else if (sum >  127) sum =  127;
			IMG_PX(x, y) = sum + 128;
		}
	}
}
