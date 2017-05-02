/* Copyright 2010-2012 Stuart Bennett <sb476@cam.ac.uk> */

#ifndef FEATURE_ORIENTATION_H
#define FEATURE_ORIENTATION_H

// #include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>	// off_t
#include <iostream>

/**
 * True if ori_a and ori_b are /different/ -- /opposite/ +/- 1 bin
 */
static inline bool different_orientation(int ori_a, int ori_b)
{
	return ((ori_a - ori_b + 5) & 7) < 3;
}

/**
 * True if ori_a and ori_b are /similar/ -- equal +/- 1 bin
 */
static inline bool similar_orientation(int ori_a, int ori_b)
{
	return ((ori_a - ori_b + 6) & 7) > 4;
}

/**
 * Returns a signed value dependent on ori_a - ori_b:
 * 0 for equality, -4 for /opposite/, 1-3 and -1--3 for intermediate differences
 */
static inline int orientation_diff(int ori_a, int ori_b)
{
	return ((ori_a - ori_b + 4) & 7) - 4;
}

int assign_orientation(size_t w, uint8_t *image, off_t val_off, int r);

#endif /* FEATURE_ORIENTATION_H */
