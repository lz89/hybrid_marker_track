/* Copyright 2010-2012 Stuart Bennett <sb476@cam.ac.uk> */

#ifndef CORNER_DETECT_H
#define CORNER_DETECT_H

#include <stddef.h>
#include <stdint.h>

// void corner_detect5(const size_t w, const size_t h, const uint8_t image[w * h], int16_t response[w * h]);
void corner_detect5(const size_t w, const size_t h, const uint8_t image[], int16_t response[]);
//void corner_detect10(const size_t w, const size_t h, const uint8_t image[w * h], int16_t response[w * h]);

#endif /* CORNER_DETECT_H */
