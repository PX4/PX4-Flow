/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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

#include "sonar_mode_filter.h"
#include <string.h>

/**
 * insert-only ring buffer of sonar data, needs to be of uneven size
 * the initialization to zero will make the filter respond zero for the
 * first three inserted readinds, which is a decent startup-logic.
 */
static float sonar_values[3] = { 0.0f };
static unsigned insert_index = 0;

static void sonar_bubble_sort(float in_out_sonar_values[], unsigned n)
{
	float t;

	for (unsigned i = 0; i < (n - 1); i++) {
		for (unsigned j = 0; j < (n - i - 1); j++) {
			if (in_out_sonar_values[j] > in_out_sonar_values[j+1]) {
				/* swap two values */
				t = in_out_sonar_values[j];
				in_out_sonar_values[j] = in_out_sonar_values[j + 1];
				in_out_sonar_values[j + 1] = t;
			}
		}
	}
}

float insert_sonar_value_and_get_mode_value(float insert)
{
	const unsigned sonar_count = sizeof(sonar_values) / sizeof(sonar_values[0]);

	sonar_values[insert_index] = insert;
	insert_index++;
	if (insert_index == sonar_count) {
		insert_index = 0;
	}

	/* sort and return mode */

	/* copy ring buffer */
	float sonar_temp[sonar_count];
	memcpy(sonar_temp, sonar_values, sizeof(sonar_values));

	sonar_bubble_sort(sonar_temp, sonar_count);

	/* the center element represents the mode after sorting */
	return sonar_temp[sonar_count / 2];
}
