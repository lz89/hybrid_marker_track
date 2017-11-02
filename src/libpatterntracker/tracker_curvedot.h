/*
	TrackerCurvedot class derived from TrackerKeydot

	Tracking circle dot curve pattern 

	2017-05-02 Lin Zhang
	The Hamlyn Centre for Robotic Surgery,
	Imperial College, London
	Copyright (c) 2017. All rights reserved.
	Use of this source code is governed by a BSD-style license that can be
	found in the LICENCE file.
*/


#ifndef TRACKER_CURVEDOT_H
#define TRACKER_CURVEDOT_H

#include "tracker_keydot.h"
#include "chess_detector.h"

class TrackerCurvedot : public TrackerKeydot
{
public:
	enum DetectState{ UNKNOWN = 0x00000000, 
		TOP_CIR = 0x00000001, 
		MID_CIR = 0x00000002, 
		BOT_CIR = 0x00000004, 
		TOP_CHESS = 0x00000008, 
		MID_CHESS = 0x00000010,
		BOT_CHESS = 0x00000020};

	TrackerCurvedot(cv::Size _pattern_size = cv::Size(2, 5), 
		cv::Size _roi_size = cv::Size(100, 100),
		cv::SimpleBlobDetector::Params params = cv::SimpleBlobDetector::Params(),
		cv::SimpleBlobDetector::Params params_roi = cv::SimpleBlobDetector::Params());

    virtual bool track(const cv::Mat &cur_image);

    // --- Detection part ---
    bool DetectPattern(const cv::Mat& _img_gray, 
		std::vector<cv::Point2f>& _symm_dots,
		std::vector<cv::Point2f>& _asymm_dots,
		std::vector<cv::Point2f>& _chess_pts);

    bool FindDots( cv::InputArray _image, cv::Size sym_patternSize, cv::Size asym_patternSize,
        cv::OutputArray _sym_centers, cv::OutputArray _asym_centers, 
		const cv::Ptr<cv::FeatureDetector> &blobDetector,
		const std::vector<cv::Point2f> &chess_pts = std::vector<cv::Point2f>());

	// --- Tracking part ---
	void initSymTrack(cv::Mat& _pre_gray, std::vector<cv::Point2f> _prev_dots);
	void initAsymTrack(cv::Mat& _pre_gray, std::vector<cv::Point2f> _prev_dots);

	bool TrackPattern(const cv::Mat& _cur_gray, cv::Mat& _sym_H, cv::Mat& _asym_H);

	// --- Draw results ---
	void drawKeydots(cv::InputOutputArray _image);

	inline int CurrDetectState() {return curr_state;}

	inline bool ChessFound() { return m_chess_found; }

	// get points in image coordinate
	std::vector<cv::Point2f> getP_img();

	// get fit chess line
	cv::Vec3f get_chess_line();

	// get chess points
	std::vector<cv::Point2f> get_chess_pts();

	

protected:

	CirclesGridClusterFinder SymmCirclesGridClusterFinder;
	CirclesGridClusterFinder AsymmCirclesGridClusterFinder;

    // --- Dection part ---
	cv::Size asym_pattern_size;
	cv::Size sym_pattern_size;
	float asym_square_size;
	float sym_square_size;


	// Asymmetric dots (TOP or BOTTOM)
	std::vector<cv::Point2f> curr_asym_dots;
	std::vector<cv::Point2f> curr_asym_corners;

	// Symmetric dots (MIDDLE)
	std::vector<cv::Point2f> curr_sym_dots;
	std::vector<cv::Point2f> curr_sym_corners;

	// Chess dots
	std::vector<cv::Point2f> curr_chess_dots;

	// Indicate which part(s) are tracked (TOP, TOP_MID, MID, BOT_MID, BOT)
	int curr_state;

	// Chess feature detector
	ChessDetector m_chess_detector;

	// True if chess feature found
	bool m_chess_found;

	// Threshold to filter false-positive circular dots
	// close to chess feature.
	// Adaptive to current marker size, default value is 10 (pixel)
	int m_thresh_dot_chess;

	// Threshold to filter outliers after chess detection
	// Outliers are considered more than 'thresh' 
	// away from the majority cluster.
	// Adaptive to current marker size
	int m_thresh_chess;

	// Chess line in general form (A, B, C) (AX+BY+C=0)
	cv::Vec3f m_chess_line;

    // --- Tracking part ---
	bool binitSymTracker;
	bool binitAsymTracker;
	bool isSymTracking;
	bool isAsymTracking;
	cv::Mat pre_sym_gray, pre_asym_gray;
	std::vector<unsigned char> pre_sym_status, pre_asym_status;

	std::vector<cv::Point2f> prev_sym_dots, prev_asym_dots, pre_tri_dots;

	std::vector<cv::Point2f> sym_model_dots, asym_model_dots;

	std::vector<cv::Point2f> sym_corner_pts, asym_corner_pts;

    // --- Draw ---
	cv::Mat sym_homography;
	cv::Mat asym_homography;

	// colors for identity
	std::vector<cv::Scalar> sym_dot_colors;
	std::vector<cv::Scalar> asym_dot_colors;

private:

	// Input slope of line, return orientation label (-4 ~ 3)
	void calc_chess_orient (const float &slope, int &label_mid, int &label_out);


};

#endif	//TRACKER_CURVEDOT_H
