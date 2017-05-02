/**
 * @file non_max_sup_pts.c
 *
 * @copyright Copyright 2010-2012 Stuart Bennett <sb476@cam.ac.uk>
 *
 * This programme is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Performs suppression of non-maxima in a certain radius, suppression of weak
 * local maxima over a large radius, and sub-pixel localization of maxima
 */

#include "non_max_sup_pts.h"

// #include "stdutil.h"

#include <stdio.h>
#include <stdlib.h>	// abs
#include <string.h>	// memset
#include <sys/types.h>	// off_t
#include <vector>
#include <algorithm>

// limit number of points considered to practical upper bound
// (radius being the basic non-maximal suppression radius)
// #define MAX_POINTS_TO_CONSIDER ((w / radius) * (h / radius))

/**
 * The attribute structure for potentially significant maxima
 */
struct consider_point {
	bool valid;
	struct icoord coord;
	uint16_t max;	// the strength of the response, as given by the sum of the two most intense pixels
	struct fcoord com;
	int mass;	// the strength of the response, as given by the sum of connected pixels
};

#define COM_RADIUS 7	// half-width of area sampled by com() below

/**
 * Centre of mass finding function, for a given response cluster
 *
 * @param	r	radius over which to include responses
 * @param	w	response image width
 * @param	ri	response image
 * @param	cp	a consideration point, providing maximal point
 * 			co-ordinates, getting CoM co-ordinates in return
 */
static void com(int r, size_t w, int16_t *ri, struct consider_point *cp)
{
	char sz = 2 * r + 1;
// 	bool **_visited = new bool*[sz];
// 	for(char i = 0; i < sz; ++i)
// 		_visited[i] = new bool[sz];

	bool **visited = new bool*[sz];
	for(char i = 0; i < sz; ++i)
		visited[i] = new bool[sz];

	std::vector<icoord> to_visit(sz * sz);
	
	int tvl = 1;
	off_t off = cp->coord.x + cp->coord.y * w;
	int xsum = 0, ysum = 0, m = 0;

	// rather than take the CoM of the whole area, this only includes
	// above threshold responses connected in one of lrud to the maximal
	// point
// 	memset(_visited, 0, sizeof(_visited));
	memset(visited, 0, sizeof(visited));
// 	visited = (bool (*)[sz])&_visited[r][r];
	visited[0][0] = true;
	to_visit[0].x = 0; to_visit[0].y = 0;

	while (tvl--) {
		struct icoord o = to_visit[tvl], new_;

		for (new_.y = std::max(o.y - 1, -r); new_.y <= std::min(o.y + 1, r); new_.y++)
			for (new_.x = std::max(o.x - 1, -r); new_.x <= std::min(o.x + 1, r); new_.x++)
				// threshold at 0.  hmm.  FIXME?
				if (!visited[new_.x][new_.y] && ri[off + new_.x + new_.y * w] > 0) {
					to_visit[tvl++] = new_;
					visited[new_.x][new_.y] = true;
				}

		xsum += o.x * ri[off + o.x + o.y * w];
		ysum += o.y * ri[off + o.x + o.y * w];
		m += ri[off + o.x + o.y * w];
	}

	cp->com.x = cp->coord.x + (float)xsum / m;
	cp->com.y = cp->coord.y + (float)ysum / m;
	cp->mass = m;
}

/**
 * Tests found maxima for connectivity (spurious hit reduction), records metric
 * of response cluster strength (combined intensity of two strongest pixels)
 * and co-ordinates of maximum as a potentially significant point
 *
 * @param	w	response image width
 * @param	image	response image
 * @param	x	x co-ord of maximum
 * @param	y	y co-ord of maximum
 * @param	use_com	localize centre of mass of response?
 * @param	cp	intermediate list of found points
 * @param	max_cps	limit on number of potential points to locate
 * @param	num_found	current index into cp
 * @return		number of successfully processed points
 */
static int process_maximum(size_t w, int16_t *image, int x, int y,
		           bool use_com, struct consider_point *cp, int max_cps, int *num_found)
{
	off_t val_off = x + (y * w);
	uint16_t second_max = 0;
	off_t second_max_off;

	// now find the next most intense response in the
	// 8 pixels surrounding the maximum
	for (off_t dy = -1; dy <= 1; ++dy)
		for (off_t dx = -1; dx <= 1; ++dx) {
			off_t offset = val_off + dx + (dy * w);

			if (image[offset] > second_max && (dx || dy)) {
				second_max = image[offset];
				second_max_off = offset;
			}
		}
	if (!second_max)	// things we want are always(?) bigger than one pixel
		return 0;

	// guard against adjacent pixels of equal intensity both being added:
	// coord in adjacent direction will be fixed by later interpolation,
	// other coord will just have to be a bit iffy
	if (second_max == image[val_off])
		if (second_max_off < val_off)
			return 0;

	cp[*num_found].valid = true;
	cp[*num_found].max = image[val_off] + second_max;
	cp[*num_found].coord.x = x;
	cp[*num_found].coord.y = y;
	if (use_com)	// find CoM of connected response cluster
		com(COM_RADIUS, w, image, &cp[(*num_found)++]);
	else
		cp[(*num_found)++].mass = 0;

	if (*num_found == max_cps) {
		fprintf(stderr, "Hit MAX_POINTS_TO_CONSIDER limit on corners, only processing %d of them\n", max_cps);
		return -1;
	}

	return 1;
}

/**
 * Searches for maxima with > radius separation in response image, calls
 * process_maximum to process/record position
 *
 * @param	w	response image width
 * @param	h	response image height
 * @param	image	response image
 * @param	border	width of image border without valid points
 * @param	radius	non-maximal suppression radius
 * @param	thresh	threshold response intensity
 * @param	use_com	localize centre of mass of response?
 * @param	cp	intermediate list of found points
 * @param	max_cps	limit on number of potential points to locate
 * @return		number of located potential points
 */
static int search(size_t w, size_t h, int16_t image[], int border,
		  int radius, int thresh, bool use_com, struct consider_point *cp, int max_cps)
{
	int num_found = 0;
	size_t min_xy = std::max(border, radius), max_x = w - std::max(border, radius), max_y = h - std::max(border, radius);

	// limits to avoid reading off the response image edge
	for (size_t y = min_xy; y < max_y; ++y)
	{
		for (size_t x = min_xy; x < max_x; ++x) {
			off_t val_off = x + (y * w);

			if (image[val_off] <= thresh)
				continue;

			// if another point forwards in the row within the radius is better, skip to it
			for (off_t dx = 1; dx <= radius; dx++)
				if (image[val_off + dx] > image[val_off]) {
					x += dx - 1;	// -1 because of ++x in for loop
					goto next;
				}
			// discount this maximum if there's a stronger maxima within a (radius x radius) square
			for (off_t dy = -radius; dy <= radius; ++dy)
				for (off_t dx = -radius; dx <= radius; ++dx)
					if (image[val_off + dx + (dy * w)] > image[val_off]) {
						// skip the amount to shift the stronger point out of the
						// window WITHOUT going past the limit of known weaker points
						// found in the previous test
						x += std::min((int)dx, 0) + radius;
						goto next;
					}
			// if the maximum is just inside the border, there's a good chance that part of its
			// response (possibly even *true* maximum) lies inside the border, so the maximum will
			// be incorrectly localized
			// since a 5x5 CoM is used later, we discount any maximum <= two px inside the border
			if (x < (min_xy + 2) || x > (max_x - 3) || y < (min_xy + 2) || y > (max_y - 3))
				continue;

			// must be a local maximum
			if (process_maximum(w, image, x, y, use_com, cp, max_cps, &num_found) < 0)
				return num_found;
			// it was a maximum, no stronger points exist within the radius
			x += radius;
next:
			;
		}
	}

	return num_found;
}

/**
 * Tunable knobs for cull_neighbourhood().  Unlikely to need changing
 */
// parameterize? FIXME
// make smaller to cull more features
#define DEGRADE_SHIFT 4	// "max" attribute must be > ~6% of greatest neighbour
#define MASS_DEGRADE_SHIFT 5 // "mass" attribute must be > ~3% of greatest neighbour

/**
 * Suppression of intermediate stage points which have significantly less
 * intense response strengths than their neighbours (effectively local
 * lighting/contrast compensation)
 *
 * @param	num_found	number of points under consideration
 * @param	cp		list of points to consider
 * @param	search_size	radius of comparison neighbourhood
 * @return			number of points suppressed
 */
static int cull_neighbourhood(int num_found, struct consider_point cp[], char search_size)
{
	int invalidated = 0;

	// iterate through all consider_points
	for (int i = 0; i < num_found; i++) {
// 		struct consider_point *local[num_found];
		std::vector<consider_point *> local(num_found);
		int locals = 1;
		int max = cp[i].max, maxm = cp[i].mass;

		if (!cp[i].valid)
			continue;

	   	local[0] = &cp[i];

		// iterate through remaining cps
		for (int t = i + 1; t < num_found; t++) {
			if (!cp[t].valid)
				continue;
			// are they within the neighbourhood area?
			if (abs(cp[i].coord.x - cp[t].coord.x) < search_size &&
			    abs(cp[i].coord.y - cp[t].coord.y) < search_size) {
				// if yes, add them to "local" list
				local[locals++] = &cp[t];
				// and keep track of greatest response
				// strengths in neighbourhood
				if (cp[t].max > max)
					max = cp[t].max;
				if (cp[t].mass > maxm)
					maxm = cp[t].mass;
			}
		}

		// determine absolute threshold levels
		int sig = max >> DEGRADE_SHIFT;
		int msig = maxm >> MASS_DEGRADE_SHIFT;
		for (int l = 0; l < locals; l++)
			// invalidate any locals less than thresholds
			if (local[l]->max < sig || local[l]->mass < msig) {
				local[l]->valid = false;
				invalidated++;
			}
	}

	return invalidated;
}

/**
 * Find the centre of mass of a 5x5 area of response image pixels
 * Used for sub-pixel localization of accepted points
 *
 * @param	w	response image width
 * @param	h	response image height
 * @param	resp	response image
 * @param	o	offset of maximal point in response image
 * @param	thresh	threshold response intensity
 * @param	outx	x distance of CoM from maxima
 * @param	outy	y distance of CoM from maxima
 */
static void com_interp5(const size_t w, const int16_t *resp, const off_t o, const int thresh,
			float *outx, float *outy)
{
	int x_sum = 0, y_sum = 0, mass = 0;

	for (int y = -2; y <= 2; y++)
		for (int x = -2; x <= 2; x++)
			if (resp[o + y * w + x] > thresh) {
				x_sum += resp[o + y * w + x] * x;
				y_sum += resp[o + y * w + x] * y;
				mass += resp[o + y * w + x];
			}

	*outx = (float)x_sum / mass;
	*outy = (float)y_sum / mass;
}

/**
 * Entry point to non-maximal suppression routines
 *
 * @param	w	response image width
 * @param	h	response image height
 * @param	image	response image
 * @param	border	width of image border without valid points
 * @param	radius	non-maximal suppression radius
 * @param	thresh	threshold response intensity
 * @param	use_com	localize centre of mass of response?
 * @param	cn_halfwidth	radius of comparison neighbourhood
 * @param	new_pt_output	call-back to allocate/initialize storage of
 * 				accepted points
 * @param	append_pt	call-back to store an accepted localized point
 */
int non_max_sup_pts(size_t w, size_t h, int16_t image[], int border,
		    unsigned radius, int thresh, bool use_com, char cn_halfwidth,
		    void *(*new_pt_output)(int), bool (*append_pt)(void *, struct fcoord *), void **pt_output)
{ 
	int num_found = 0;

	int MAX_POINTS_TO_CONSIDER = ((w / radius) * (h / radius));
	struct consider_point *cp = new consider_point[MAX_POINTS_TO_CONSIDER];

	if ((h <= 2 * std::max(border, (int)radius)) || (w <= 2 * std::max(border, (int)radius))) {
		fprintf(stderr, "Radius too large for input image\n");
		return -1;
	}

	// add half-width of centre-of-mass sampling area to the exclusion
	// border to avoid skewed results.  !use_com path uses 5x5 area:
	// +/- 2 of maxima
	border += use_com ? COM_RADIUS : 2;

	// find the candidate points
	num_found = search(w, h, image, border, radius, thresh, use_com,
			   cp, sizeof(cp) / sizeof(struct consider_point));

	if (!num_found)
		return num_found;

	// prune the neighbourhood weaklings
	int culled = cull_neighbourhood(num_found, cp, cn_halfwidth);

	// allocate storage for the accepted points
	if (!(*pt_output = new_pt_output(num_found - culled))) {
		fprintf(stderr, "Point-coord struct init failed\n");
		return -2;
	}

	for (int idx = 0; idx < num_found; idx++) {
		if (!cp[idx].valid)
			continue;
		
		if (use_com) {
			// use pre-generated CoM positions
			// pixel centres at 0.5px in, either way
			fcoord fc;
			fc.x = cp[idx].com.x + 0.5;
			fc.y = cp[idx].com.y + 0.5;
			append_pt(*pt_output, &fc);
		} else {
			off_t int_centre = cp[idx].coord.x + cp[idx].coord.y * w;
			float dx, dy;

			// get sub-pixel location with 5x5 CoM patch
			com_interp5(w, image, int_centre, thresh, &dx, &dy);

			// pixel centres at 0.5px in, either way
			fcoord fc;
			fc.x = cp[idx].coord.x + 0.5f + dx;
			fc.y = cp[idx].coord.y + 0.5f + dy;
			append_pt(*pt_output, &fc);
		}
	}

	return num_found - culled;
}
