/**
 * @file corner_detect.c
 *
 * @copyright Copyright 2010-2012 Stuart Bennett <sb476@cam.ac.uk>
 *
 * This programme is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Applies the ChESS corner detection algorithm
 */

#include "corner_detect.h"

#include <stdlib.h>	// abs
#include <sys/types.h>	// off_t

/**
 * Our vector types
 */
//typedef char v16qi __attribute__ ((vector_size(16)));
//typedef int16_t v8hi __attribute__ ((vector_size(16)));
//typedef int v4si __attribute__ ((vector_size(16)));
//typedef long long v2di __attribute__ ((vector_size(16)));

/**
 * Unions for inserting/reading data into/from vector types
 */
//union v16b {
//	v16qi v;
//	uint8_t b[16];
//	uint32_t dw[4];
//};
//
//union v8w {
//	v8hi v;
//	int16_t w[8];
//};
//
//union v2qw {
//	v2di v;
//	uint64_t uq[2];
//};

///**
// * Summation and differencing function.  Outputs 4 response pixels per iteration
// *
// * @param	a	"a" samples
// * @param	b	"b" samples
// * @param	c	"c" samples
// * @param	d	"d" samples
// * @param	local_mean	local_mean samples
// * @param	response	output vector
// */
//static inline void sd(v16qi a, v16qi b, v16qi c, v16qi d, uint16_t local_mean[4], int16_t *response)
//{
//	const union v16b zero = { .b = { 0 }};
//	// vector of indices of how samples must be permuted to group samples by output pixel
//	const union v16b shuf = { .b = { 0, 4, 8, 12, 1, 5, 9, 13, 2, 6, 10, 14, 3, 7, 11, 15 }};

//	/*
//	uint16_t sum_response = 0;
//	uint16_t diff_response = 0;
//	uint16_t mean = 0;

//	for (size_t sub_idx = 0; sub_idx < 4; ++sub_idx) {
//		uint8_t a = circular_sample[sub_idx];
//		uint8_t b = circular_sample[sub_idx + 4];
//		uint8_t c = circular_sample[sub_idx + 8];
//		uint8_t d = circular_sample[sub_idx + 12];

//		sum_response += abs(a - b + c - d);	// high magnitude for 2nd FIXME doc
//		diff_response += abs(a - c) + abs(b - d);
//		mean += a + b + c + d;
//	}
//	*/

//	v16qi ab, cd;
//	v8hi sum_response, diff_response, a16, b16, c16, d16, temp;
//	union v8w final, mean[2];

//	// apply sample permutation such that pixel 0's samples are adjacent, pixel 1's sample are adjacent, etc.
//	a = __builtin_ia32_pshufb128(a, shuf.v);
//	b = __builtin_ia32_pshufb128(b, shuf.v);
//	c = __builtin_ia32_pshufb128(c, shuf.v);
//	d = __builtin_ia32_pshufb128(d, shuf.v);

//	// expand output pixel 0 and 1's as and bs to 16 bits
//	a16 = (v8hi)__builtin_ia32_punpcklbw128(a, zero.v);
//	b16 = (v8hi)__builtin_ia32_punpcklbw128(b, zero.v);
//	// arrange output pixel 0 and 1's a and b horizontally along a register
//	ab = (v16qi)__builtin_ia32_punpckldq128((v4si)a, (v4si)b);
//	// expand output pixel 0 and 1's cs and ds to 16 bits
//	c16 = (v8hi)__builtin_ia32_punpcklbw128(c, zero.v);
//	d16 = (v8hi)__builtin_ia32_punpcklbw128(d, zero.v);
//	// arrange output pixel 0 and 1's c and d horizontally along a register
//	cd = (v16qi)__builtin_ia32_punpckldq128((v4si)c, (v4si)d);

//	// abs((a+c) - (b+d))
//	sum_response = __builtin_ia32_pabsw128(__builtin_ia32_psubw128(__builtin_ia32_paddw128(a16, c16), __builtin_ia32_paddw128(b16, d16)));
//	// sum(abs(a_n-c_n) + abs(b_n-d_n))
//	diff_response = (v8hi)__builtin_ia32_psadbw128(ab, cd);
//	// sum(abs((a_n + b_n) - 0)) + sum(abs((c_n + d_n) - 0))
//	mean[0].v = __builtin_ia32_paddw128((v8hi)__builtin_ia32_psadbw128(ab, zero.v), (v8hi)__builtin_ia32_psadbw128(cd, zero.v));
//	// s_r_n_0 - d_r_n
//	temp = __builtin_ia32_psubw128(sum_response, diff_response);

//	// likewise for output pixel 2 and 3
//	a16 = (v8hi)__builtin_ia32_punpckhbw128(a, zero.v);
//	b16 = (v8hi)__builtin_ia32_punpckhbw128(b, zero.v);
//	ab = (v16qi)__builtin_ia32_punpckhdq128((v4si)a, (v4si)b);
//	c16 = (v8hi)__builtin_ia32_punpckhbw128(c, zero.v);
//	d16 = (v8hi)__builtin_ia32_punpckhbw128(d, zero.v);
//	cd = (v16qi)__builtin_ia32_punpckhdq128((v4si)c, (v4si)d);

//	sum_response = __builtin_ia32_pabsw128(__builtin_ia32_psubw128(__builtin_ia32_paddw128(a16, c16), __builtin_ia32_paddw128(b16, d16)));
//	diff_response = (v8hi)__builtin_ia32_psadbw128(ab, cd);
//	mean[1].v = __builtin_ia32_paddw128((v8hi)__builtin_ia32_psadbw128(ab, zero.v), (v8hi)__builtin_ia32_psadbw128(cd, zero.v));
//	final.v = __builtin_ia32_psubw128(sum_response, diff_response);

//	// horizontal add of two regs into one
//	final.v = (v8hi)__builtin_ia32_phaddw128(temp, final.v);
//	// and again, filling top half with zero
//	final.v = __builtin_ia32_phaddw128(final.v, (v8hi)zero.v);

//	// reject stripe case by removing difference in means
//	response[0] = final.w[0] - abs(mean[0].w[0] - local_mean[0]);
//	response[1] = final.w[1] - abs(mean[0].w[4] - local_mean[1]);
//	response[2] = final.w[2] - abs(mean[1].w[0] - local_mean[2]);
//	response[3] = final.w[3] - abs(mean[1].w[4] - local_mean[3]);
//}

///**
// * Perform the ChESS corner detection algorithm with a 5 px sampling radius
// *
// * @param	w	image width
// * @param	h	image height
// * @param	image	input image
// * @param	response	output response image
// */
//void corner_detect5(const size_t w, const size_t h, const uint8_t image[w * h], int16_t response[w * h])
//{
//	// funny bounds due to sampling ring radius (5) and border of previously applied blur (2)
//	for (size_t y = 7; y < h - 7; ++y)
//		// we output 4 pixels at a time
//		for (size_t x = 7; x < w - 7; x += 4) {
//			const off_t val_off = x + y * w;
//			union v16b a, b, c, d;

//			/* confusingly, having w known at compile time (causing offsets to
//			 * be coded as immediates) appears to be slower
//			 */
//			a.dw[2] = *(uint32_t *)&image[val_off - 2 - 5 * w];
//			a.dw[1] = *(uint32_t *)&image[val_off - 5 * w];
//			a.dw[0] = *(uint32_t *)&image[val_off + 2 - 5 * w];
//			c.dw[0] = *(uint32_t *)&image[val_off - 2 + 5 * w];
//			c.dw[1] = *(uint32_t *)&image[val_off + 5 * w];
//			c.dw[2] = *(uint32_t *)&image[val_off + 2 + 5 * w];
//			a.dw[3] = *(uint32_t *)&image[val_off - 4 - 4 * w];	// 5 is faster, but arguably unfair
//			d.dw[3] = *(uint32_t *)&image[val_off + 4 - 4 * w];
//			b.dw[3] = *(uint32_t *)&image[val_off - 4 + 4 * w];
//			c.dw[3] = *(uint32_t *)&image[val_off + 4 + 4 * w];
//			b.dw[0] = *(uint32_t *)&image[val_off - 5 - 2 * w];
//			d.dw[2] = *(uint32_t *)&image[val_off + 5 - 2 * w];
//			b.dw[2] = *(uint32_t *)&image[val_off - 5 + 2 * w];
//			d.dw[0] = *(uint32_t *)&image[val_off + 5 + 2 * w];
//			b.dw[1] = *(uint32_t *)&image[val_off - 5];
//			d.dw[1] = *(uint32_t *)&image[val_off + 5];

//			uint16_t local_mean[4];

//			// purely horizontal local_mean samples
//			local_mean[0] = (image[val_off - 1] + image[val_off] + image[val_off + 1]) * 16 / 3;
//			local_mean[1] = (image[val_off] + image[val_off + 1] + image[val_off + 2]) * 16 / 3;
//			local_mean[2] = (image[val_off + 1] + image[val_off + 2] + image[val_off + 3]) * 16 / 3;
//			local_mean[3] = (image[val_off + 2] + image[val_off + 3] + image[val_off + 4]) * 16 / 3;

//			sd(a.v, b.v, c.v, d.v, local_mean, &response[val_off]);
//		}
//}

///**
// * Perform the ChESS corner detection algorithm with a 10 px sampling radius
// * Uses mean of 2 samples per ChESS input pixel
// *
// * @param	w	image width
// * @param	h	image height
// * @param	image	input image
// * @param	response	output response image
// */
//void corner_detect10(const size_t w, const size_t h, const uint8_t image[w * h], int16_t response[w * h])
//{
//	// funny bounds due to sampling ring radius (10) and border of previously applied blur (2)
//	for (size_t y = 12; y < h - 12; ++y)
//		// we output 4 pixels at a time
//		for (size_t x = 12; x < w - 12; x += 4) {
//			const off_t val_off = x + y * w;
//			union v16b a, b, c, d;

//			/* confusingly, having w known at compile time (causing offsets to
//			 * be coded as immediates) appears to be slower
//			 */
//			// samples get rounded down and divided by 2
//			a.dw[2] = (*(uint32_t *)&image[val_off - 2 - 10 * w] & 0xfefefefe) >> 1;
//			a.dw[1] = (*(uint32_t *)&image[val_off + 2 - 10 * w] & 0xfefefefe) >> 1;
//			c.dw[1] = (*(uint32_t *)&image[val_off - 2 + 10 * w] & 0xfefefefe) >> 1;
//			c.dw[2] = (*(uint32_t *)&image[val_off + 2 + 10 * w] & 0xfefefefe) >> 1;
//			a.dw[3] = (*(uint32_t *)&image[val_off - 6 - 9 * w] & 0xfefefefe) >> 1;
//			a.dw[0] = (*(uint32_t *)&image[val_off + 6 - 9 * w] & 0xfefefefe) >> 1;
//			c.dw[0] = (*(uint32_t *)&image[val_off - 6 + 9 * w] & 0xfefefefe) >> 1;
//			c.dw[3] = (*(uint32_t *)&image[val_off + 6 + 9 * w] & 0xfefefefe) >> 1;
//			b.dw[0] = (*(uint32_t *)&image[val_off - 9 - 6 * w] & 0xfefefefe) >> 1;
//			d.dw[3] = (*(uint32_t *)&image[val_off + 9 - 6 * w] & 0xfefefefe) >> 1;
//			b.dw[3] = (*(uint32_t *)&image[val_off - 9 + 6 * w] & 0xfefefefe) >> 1;
//			d.dw[0] = (*(uint32_t *)&image[val_off + 9 + 6 * w] & 0xfefefefe) >> 1;
//			b.dw[1] = (*(uint32_t *)&image[val_off - 10 - 2 * w] & 0xfefefefe) >> 1;
//			d.dw[2] = (*(uint32_t *)&image[val_off + 10 - 2 * w] & 0xfefefefe) >> 1;
//			b.dw[2] = (*(uint32_t *)&image[val_off - 10 + 2 * w] & 0xfefefefe) >> 1;
//			d.dw[1] = (*(uint32_t *)&image[val_off + 10 + 2 * w] & 0xfefefefe) >> 1;

//			// then added to their partner (similarly /2) to form a mean
//			a.dw[2] += (*(uint32_t *)&image[val_off - 4 - 10 * w] & 0xfefefefe) >> 1;
//			a.dw[1] += (*(uint32_t *)&image[val_off - 10 * w] & 0xfefefefe) >> 1;
//			a.dw[0] += (*(uint32_t *)&image[val_off + 4 - 10 * w] & 0xfefefefe) >> 1;
//			c.dw[0] += (*(uint32_t *)&image[val_off - 4 + 10 * w] & 0xfefefefe) >> 1;
//			c.dw[1] += (*(uint32_t *)&image[val_off + 10 * w] & 0xfefefefe) >> 1;
//			c.dw[2] += (*(uint32_t *)&image[val_off + 4 + 10 * w] & 0xfefefefe) >> 1;
//			a.dw[3] += (*(uint32_t *)&image[val_off - 7 - 7 * w] & 0xfefefefe) >> 1;
//			d.dw[3] += (*(uint32_t *)&image[val_off + 7 - 7 * w] & 0xfefefefe) >> 1;
//			b.dw[3] += (*(uint32_t *)&image[val_off - 7 + 7 * w] & 0xfefefefe) >> 1;
//			c.dw[3] += (*(uint32_t *)&image[val_off + 7 + 7 * w] & 0xfefefefe) >> 1;
//			b.dw[0] += (*(uint32_t *)&image[val_off - 10 - 4 * w] & 0xfefefefe) >> 1;
//			d.dw[2] += (*(uint32_t *)&image[val_off + 10 - 4 * w] & 0xfefefefe) >> 1;
//			b.dw[2] += (*(uint32_t *)&image[val_off - 10 + 4 * w] & 0xfefefefe) >> 1;
//			d.dw[0] += (*(uint32_t *)&image[val_off + 10 + 4 * w] & 0xfefefefe) >> 1;
//			b.dw[1] += (*(uint32_t *)&image[val_off - 10] & 0xfefefefe) >> 1;
//			d.dw[1] += (*(uint32_t *)&image[val_off + 10] & 0xfefefefe) >> 1;

//			uint16_t local_mean[4];

//			// sample 7 pixels for local mean, apply the middle one twice to allow easy scaling to 16
//			local_mean[0] = (image[val_off - 1 - 2 * w] + image[val_off + 1 - 2 * w] +
//					 image[val_off - 2] + 2 * image[val_off] + image[val_off + 2] +
//					 image[val_off - 1 + 2 * w] + image[val_off + 1 + 2 * w]) * 2;	// * 16 / 8
//			local_mean[1] = (image[val_off - 2 * w] + image[val_off + 2 - 2 * w] +
//					 image[val_off - 1] + 2 * image[val_off + 1] + image[val_off + 3] +
//					 image[val_off + 2 * w] + image[val_off + 2 + 2 * w]) * 2;
//			local_mean[2] = (image[val_off + 1 - 2 * w] + image[val_off + 3 - 2 * w] +
//					 image[val_off] + 2 * image[val_off + 2] + image[val_off + 4] +
//					 image[val_off + 1 + 2 * w] + image[val_off + 3 + 2 * w]) * 2;
//			local_mean[3] = (image[val_off + 2 - 2 * w] + image[val_off + 4 - 2 * w] +
//					 image[val_off + 1] + 2 * image[val_off + 3] + image[val_off + 5] +
//					 image[val_off + 2 + 2 * w] + image[val_off + 4 + 2 * w]) * 2;

//			sd(a.v, b.v, c.v, d.v, local_mean, &response[val_off]);
//		}
//}

/**
 * Perform the ChESS corner detection algorithm with a 5 px sampling radius
 *
 * @param	w	image width
 * @param	h	image height
 * @param	image	input image
 * @param	response	output response image
 */
void corner_detect5(const size_t w, const size_t h, const uint8_t image[], int16_t response[])
{
    int x, y;
    // funny bounds due to sampling ring radius (5) and border of previously applied blur (2)
    for (y = 7; y < h - 7; y++)
        for (x = 7; x < w - 7; x++) {
            const unsigned offset = x + y * w;
            uint8_t circular_sample[16];

            circular_sample[2] = image[offset - 2 - 5 * w];
            circular_sample[1] = image[offset - 5 * w];
            circular_sample[0] = image[offset + 2 - 5 * w];
            circular_sample[8] = image[offset - 2 + 5 * w];
            circular_sample[9] = image[offset + 5 * w];
            circular_sample[10] = image[offset + 2 + 5 * w];
            circular_sample[3] = image[offset - 4 - 4 * w];
            circular_sample[15] = image[offset + 4 - 4 * w];
            circular_sample[7] = image[offset - 4 + 4 * w];
            circular_sample[11] = image[offset + 4 + 4 * w];
            circular_sample[4] = image[offset - 5 - 2 * w];
            circular_sample[14] = image[offset + 5 - 2 * w];
            circular_sample[6] = image[offset - 5 + 2 * w];
            circular_sample[12] = image[offset + 5 + 2 * w];
            circular_sample[5] = image[offset - 5];
            circular_sample[13] = image[offset + 5];

            // purely horizontal local_mean samples
            uint16_t local_mean = (image[offset - 1] + image[offset] + image[offset + 1]) * 16 / 3;

            uint16_t sum_response = 0;
            uint16_t diff_response = 0;
            uint16_t mean = 0;

            int sub_idx;
            for (sub_idx = 0; sub_idx < 4; ++sub_idx) {
                uint8_t a = circular_sample[sub_idx];
                uint8_t b = circular_sample[sub_idx + 4];
                uint8_t c = circular_sample[sub_idx + 8];
                uint8_t d = circular_sample[sub_idx + 12];

                sum_response += abs(a - b + c - d);
                diff_response += abs(a - c) + abs(b - d);
                mean += a + b + c + d;
            }

            response[offset] = sum_response - diff_response - abs(mean - local_mean);
        }
}
