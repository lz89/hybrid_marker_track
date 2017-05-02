/*
	TrackerKeydot class derived from Tracker

	Tracking keydot pattern

	2017-05-02 Lin Zhang, Menglong Ye
	The Hamlyn Centre for Robotic Surgery,
	Imperial College, London
	Copyright (c) 2017. All rights reserved.
	Use of this source code is governed by a BSD-style license that can be
	found in the LICENCE file.
*/


#ifndef TRACKER_KEYDOT_H
#define TRACKER_KEYDOT_H

#include <opencv2/opencv.hpp>
#include "circlesgrid.hpp"
#include "tracker.h"

class TrackerKeydot : public Tracker
{
public:
	TrackerKeydot(cv::Size _pattern_size = cv::Size(3, 7), cv::Size _roi_size = cv::Size(100, 100),
		cv::SimpleBlobDetector::Params params = cv::SimpleBlobDetector::Params(),
		cv::SimpleBlobDetector::Params params_roi = cv::SimpleBlobDetector::Params());

	TrackerKeydot(cv::Size _pattern_size = cv::Size(3, 7), 
		int flag = cv::CALIB_CB_ASYMMETRIC_GRID,
		cv::Size _roi_size = cv::Size(100, 100),
		cv::SimpleBlobDetector::Params params = cv::SimpleBlobDetector::Params(),
		cv::SimpleBlobDetector::Params params_roi = cv::SimpleBlobDetector::Params());



	virtual bool track(const cv::Mat &cur_image);

	// --- Detection part ---
	bool DetectPattern(const cv::Mat& _img_gray, std::vector<cv::Point2f>& _dots);

	bool FindDots( cv::InputArray _image, cv::Size patternSize,
		cv::OutputArray _centers, int flags,
		const cv::Ptr<cv::FeatureDetector> &blobDetector);

	void UpdateLastLocation(const std::vector<cv::Point2f>& _dots);


	// --- Tracking part ---
	void initTrack(std::vector<cv::Point2f> _model_dots, cv::Mat& _pre_gray,
		std::vector<cv::Point2f> _prev_dots);

	bool TrackPattern(const cv::Mat& _cur_gray, std::vector<cv::Point2f>& _dots, cv::Mat& _H);

	void DoSpaseOpticalFlow(const cv::Mat& _prev_img, const cv::Mat& _cur_img,
		const std::vector<cv::Point2f>& _prev_pts, std::vector<cv::Point2f>& _cur_pts,
		std::vector<unsigned char>& _status);

	void UpdateLastDots(cv::Mat& _cur_gray, std::vector<cv::Point2f> _prev_dots);

	void UpdateStatus();

	inline bool isInit() const {return binitTracker;}

	// --- Draw results ---
	void drawKeydots(cv::InputOutputArray _image);


	// get points in image coordinate
	inline std::vector<cv::Point2f> getP_img() { return curr_dots; }
	// Get corners coordinate, index start from origin clockwise
	std::vector<cv::Point2f> get_corners();

protected:

	// --- Dection part ---
	cv::Point2f last_valid_location; // random value for initialisation
	int roi_hw;
	int roi_hh;
	cv::Size pattern_size;
	float square_size;
	cv::Ptr<cv::FeatureDetector> blob_detector;
	cv::Ptr<cv::FeatureDetector> roi_blob_detector;
	int pattern_type;

	// --- Tracking part ---
	bool binitTracker;
	cv::Mat pre_gray;
	std::vector<unsigned char> pre_status;
	std::vector<cv::Point2f> prev_dots;
	std::vector<cv::Point2f> model_dots;
	std::vector<cv::Point2f> corner_pts;

	std::vector<cv::Point2f> curr_dots;
	std::vector<cv::Point2f> curr_corners;


	// --- Draw ---
	cv::Mat homography;
	bool bisTracking;
	// colors for identity
	std::vector<cv::Scalar> dot_colors;
};

#endif	//TRACKER_KEYDOT_H
