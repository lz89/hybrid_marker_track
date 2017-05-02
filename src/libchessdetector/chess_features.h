/**
 * @file chess_features.h
 *
 * @copyright Copyright 2010-2012 Stuart Bennett <sb476@cam.ac.uk>
 *
 * Data structures and algorithms for storing and searching ChESS feature points
 */

#ifndef CHESS_FEATURES_H
#define CHESS_FEATURES_H

#include "coord_types.h"


/**
 * Structure containing key ChESS feature-point information: position and orientation
 */
struct point {
	// FIXME: now that we undistort, should we keep integer px coords of image feature location?
	struct fcoord pos;
	int ori;
	int idx;
	void *userdata;
};

/**
 * Structure bundling up a number of struct points
 */
struct sized_point_list {
	int capacity;
	int occupancy;
	struct point point[];
};

void *new_point_list(int n);
bool append_pl_point(void *plv, struct fcoord *pos);

#endif /* CHESS_FEATURES_H */
