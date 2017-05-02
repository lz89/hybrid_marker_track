/**
 * @file chess_features.c
 *
 * @copyright Copyright 2012 Stuart Bennett <sb476@cam.ac.uk>
 *
 * Functions for creating/altering lists of ChESS feature points
 */
#include "chess_features.h"

#include <stdlib.h>

/**
 * Creates a new struct sized_point_list -- a list of struct point
 *
 * @param	n	number of points the list is to store
 * @return		the list, as an opaque pointer type
 */
void *new_point_list(int n)
{
	struct sized_point_list *spl;

	spl = static_cast<sized_point_list*>(malloc(sizeof(struct sized_point_list) + n * sizeof(struct point)));
	if (!(spl))
		return NULL;

	spl->capacity = n;
	spl->occupancy = 0;
	for (int p = 0; p < n; p++)
		spl->point[p].idx = p;

	return spl;
}

/**
 * Adds a co-ordinate pair to a pre-existing point list
 *
 * @param	splv	the point list, as an opaque pointer
 * @param	pos	the new co-ordinates
 * @return		boolean success of addition (fails if list full)
 */
bool append_pl_point(void *splv, struct fcoord *pos)
{
	struct sized_point_list *spl = static_cast<sized_point_list*>(splv);
	if (spl->occupancy >= spl->capacity)
		return false;

	spl->point[spl->occupancy++].pos = *pos;

	return true;
}
