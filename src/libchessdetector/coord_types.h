/* Copyright 2010-2012 Stuart Bennett <sb476@cam.ac.uk> */

#ifndef COORD_TYPES_H
#define COORD_TYPES_H

#include <stdint.h>

struct fcoord {
	float x, y;
};

struct icoord {
	int x, y;
};

struct scoord {
	int16_t x, y;
};

#endif /* COORD_TYPES_H */
