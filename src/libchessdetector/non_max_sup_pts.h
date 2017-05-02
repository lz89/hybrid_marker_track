/* Copyright 2010-2012 Stuart Bennett <sb476@cam.ac.uk> */

#ifndef NON_MAX_SUP_PTS_H
#define NON_MAX_SUP_PTS_H

#include "coord_types.h"

// #include <stdbool.h>
#include <stddef.h>

// int non_max_sup_pts(size_t w, size_t h, int16_t image[w * h], int border,
// 		    unsigned radius, int thresh, bool use_com, char compare_halfwidth,
// 		    void *(*new_pt_output)(int), bool (*append_pt)(void *, struct fcoord *), void **pt_output);
int non_max_sup_pts(size_t w, size_t h, int16_t image[], int border,
					unsigned radius, int thresh, bool use_com, char compare_halfwidth,
					void *(*new_pt_output)(int), bool (*append_pt)(void *, struct fcoord *), void **pt_output);

#endif /* NON_MAX_SUP_PTS_H */
