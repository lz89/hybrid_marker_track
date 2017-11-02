/*
    ChessDetector class
	Wrapper for ChESS feature detector

    2016-12-10 Lin Zhang
    The Hamlyn Centre for Robotic Surgery,
    Imperial College, London
*/
#ifndef CHESS_DETECTOR_H
#define CHESS_DETECTOR_H


#include <opencv2/core.hpp>
#include "chess_features.h"
#include "corner_detect.h"
#include "non_max_sup_pts.h"
#include "feature_orientation.h"


class ChessDetector
{
public:
	struct Params
	{
		Params();
		unsigned int radius;
		unsigned int neighbourhood;
		
		bool estimateOrientation;

		// Flag to filter points has minority orientation
		bool filterMinorOrientation;
	};

	ChessDetector(const ChessDetector::Params &parameters = ChessDetector::Params());

	~ChessDetector();

	// Input an 'image' and output detected 'points'.
	// If 'thresh_outlier' is specified (!=0), 
	// points NOT stay within 'thresh_outlier' are filtered.
	// 'all_chess_points' are chess features without filter
	bool detect(cv::InputArray image, 
		std::vector<cv::Point2f> &points, 
		std::vector<cv::Point2f> &all_chess_points,
		const int &thresh_outlier = 0);

	// Return true if orientation is valid, false otherwise
	inline bool Orientation(int &ori) {
		ori = m_orient;
		return (ori > 3 || ori < -4) ? false : true; 
	}

private:

	// pointer to current image
	uint8_t *m_img;

	// pointer to current response map
	int16_t *m_resp;

	// list of point
	sized_point_list *m_points;

	// is the response map is prelocated
	bool m_resp_init;

	int m_img_width;
	int m_img_height;

	// Orientation of major detection
	// Valid values: [-4, 3]
	// Invalid values: 1000
	int m_orient;

	Params params;
};
#endif		// CHESS_DETECTOR_H