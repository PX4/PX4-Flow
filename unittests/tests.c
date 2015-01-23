/**
 * UNIT TESTS for flow code
 */


#include <stdio.h>
#include "../inc/sonar_mode_filter.h"

int main(int argc, char *argv[]) {

	for (unsigned i = 0; i < 25; i++) {

		unsigned in = i;

		if (in % 5 == 0) {
			in*= 10;
		}

		float inf = in / 10.0f;

		float out = insert_sonar_value_and_get_mode_value(inf);

		printf("in: %f\tout: %f\n", (double)inf, (double)out);
	}
	
}
