/**
 * @file feature_orientation.c
 *
 * @copyright Copyright 2010-2012 Stuart Bennett <sb476@cam.ac.uk>
 *
 * Functions related to assignment and use of ChESS orientations
 */
#include "feature_orientation.h"

// #include "stdutil.h"

#include <stdlib.h>	// abs
#include <sys/types.h>	// off_t

/**
 * Analyzes the area around a given point to determine the feature's orientation
 *
 * @param	w	image width
 * @param	image	the camera image
 * @param	val_off	the offset of the feature within the image array
 * @param	r	the radius multiplier for the sampling ring
 * @return		the detected feature orientation
 */
int assign_orientation(size_t w, uint8_t *image, off_t val_off, int r)
{
	// val contains the four "sampling cross" rotations previously
	// combined in the sum response
	int val[4] = {
		image[val_off - r * 2 - r * 5 * w] -
		image[val_off + r * 5 - r * 2 * w] +
		image[val_off + r * 2 + r * 5 * w] -
		image[val_off - r * 5 + r * 2 * w],

		image[val_off - r * 5 * w] -
		image[val_off + r * 5] +
		image[val_off + r * 5 * w] -
		image[val_off - r * 5],

		image[val_off + r * 2 - r * 5 * w] -
		image[val_off + r * 5 + r * 2 * w] +
		image[val_off - r * 2 + r * 5 * w] -
		image[val_off - r * 5 - r * 2 * w],

		image[val_off + r * 4 - r * 4 * w] -
		image[val_off + r * 4 + r * 4 * w] +
		image[val_off - r * 4 + r * 4 * w] -
		image[val_off - r * 4 - r * 4 * w]
	};

	// uninitialized_var() is just a macro to quieten the (stupid) compiler
	int max_response = 0, response;
	// all we basically do is find which of the four rotations has the
	// biggest magnitude, and then determine if it's +ve or -ve
	for (int s = 0; s < 4; s++) {
		int aval;

		// but there's some tricks with adding three adjacent rotations
		// to get some averaging -- the 0th and 3rd rotations have to
		// have their signs flipped before addition
		if (s == 0)
			aval = abs(-val[3] + val[0] + val[1]);
		else if (s == 3)
			aval = abs(val[2] + val[3] - val[0]);
		else
			aval = abs(val[s - 1] + val[s] + val[s + 1]);

		if (aval > max_response) {
			max_response = aval;
			// once the strongest combination is found, consider
			// the middle rotation response's sign to increase the
			// number of possible orientation classifications to 8
			response = val[s] < 0 ? s - 4 : s;
		}
	}

	return response;
}
