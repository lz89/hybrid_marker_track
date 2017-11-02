#include "tracker_curvedot.h"

TrackerCurvedot::TrackerCurvedot(cv::Size _pattern_size,
							 cv::Size _roi_size,
                             cv::SimpleBlobDetector::Params params,
                             cv::SimpleBlobDetector::Params params_roi) :
	TrackerKeydot (_pattern_size, _roi_size, params, params_roi),
	binitSymTracker (false), binitAsymTracker (false),
	isSymTracking (false), isAsymTracking (false),
	SymmCirclesGridClusterFinder(false, false),
	AsymmCirclesGridClusterFinder(true, true),
	curr_state (UNKNOWN),
	m_chess_found (false),
	m_thresh_dot_chess(10),
	m_thresh_chess (100)
{
	asym_pattern_size = cv::Size(1, pattern_size.height + (pattern_size.height-1));
	sym_pattern_size = pattern_size;

	// record model points on the grid
	for( int i = 0; i < asym_pattern_size.height; i++ )
	{
		for( int j = 0; j < asym_pattern_size.width; j++ )
		{
			cv::Point2f idealPt;
			idealPt = cv::Point2f((2*j + i % 2)*square_size, i*square_size);
			asym_model_dots.push_back(idealPt);
		}
	}
	for( int i = 0; i < sym_pattern_size.height; i++ )
	{
		for( int j = 0; j < sym_pattern_size.width; j++ )
		{
			cv::Point2f idealPt;
			// TODO: square_size can be different for x-y direction
			idealPt = cv::Point2f(j*square_size, i*square_size);
			sym_model_dots.push_back(idealPt);
		}
	}

	//				3------0
	// Asymetric	 \****/
	//				  2--1
	asym_corner_pts.push_back(cv::Point2f(0, 0));
	asym_corner_pts.push_back(cv::Point2f((asym_pattern_size.width*2-1)*square_size, square_size));
	asym_corner_pts.push_back(cv::Point2f((asym_pattern_size.width*2-1)*square_size, (asym_pattern_size.height-2)*square_size));
	asym_corner_pts.push_back(cv::Point2f(0, (asym_pattern_size.height-1)*square_size));
	//				3-----0
	// Symetric		|*****|
	//				2-----1
	sym_corner_pts.push_back(cv::Point2f(0, 0));
	sym_corner_pts.push_back(cv::Point2f((sym_pattern_size.width-1)*square_size, 0));
	sym_corner_pts.push_back(cv::Point2f((sym_pattern_size.width-1)*square_size, (sym_pattern_size.height-1)*square_size));
	sym_corner_pts.push_back(cv::Point2f(0, (sym_pattern_size.height-1)*square_size));

	// generate colors for identity
	//////////////////////////////////////////////////////////////////////////
	cv::RNG rng(12345);
	for (int i = 0; i < asym_pattern_size.height; i++)
	{
		cv::Scalar color = cv::Scalar(rng.uniform(100, 255), rng.uniform(100, 255),rng.uniform(100, 255)); //unknown color
		for (int j = 0; j < asym_pattern_size.width; j++)
			asym_dot_colors.push_back(color);
	}
	for (int i = 0; i < sym_pattern_size.height; i++)
	{
		cv::Scalar color = cv::Scalar(rng.uniform(100, 255), rng.uniform(100, 255),rng.uniform(100, 255)); //unknown color
		for (int j = 0; j < sym_pattern_size.width; j++)
			sym_dot_colors.push_back(color);
	}
}

bool TrackerCurvedot::track(const cv::Mat &cur_image)
{
	cv::Mat cur_gray;
	cv::cvtColor(cur_image, cur_gray, cv::COLOR_BGR2GRAY);

	bool found = DetectPattern(cur_gray, curr_sym_dots, curr_asym_dots, curr_chess_dots);
	asym_homography = sym_homography = cv::Mat();

	if (found)
	{
		if (!curr_asym_dots.empty())
		{
			// Asymmetric
			asym_homography = cv::findHomography(asym_model_dots, curr_asym_dots, CV_RANSAC, square_size*2);
			isAsymTracking = false;
			if (!binitAsymTracker)
			{
				initAsymTrack(cur_gray, curr_asym_dots);
			}
			std::fill(pre_asym_status.begin(), pre_asym_status.end(), 1);
		}
		if (!curr_sym_dots.empty())
		{
			// Symmetric
			sym_homography = cv::findHomography(sym_model_dots, curr_sym_dots, CV_RANSAC, square_size*2);
			isSymTracking = false;
			
			if (!binitSymTracker)
			{
				initSymTrack(cur_gray, curr_sym_dots);
			}
			std::fill(pre_sym_status.begin(), pre_sym_status.end(), 1);
		}
	}
	else // If detection is not good -> perform tracking using optical flow
	{
		bool homography_valid = TrackPattern(cur_gray, sym_homography, asym_homography);

		if (homography_valid)
		{
			std::vector<cv::Point2f> h_pts;
			if (!sym_homography.empty())
			{
				cv::perspectiveTransform(sym_model_dots, h_pts, sym_homography);
				curr_sym_dots = h_pts;
				isSymTracking = true;
			}
			if (!asym_homography.empty())
			{
				cv::perspectiveTransform(asym_model_dots, h_pts, asym_homography);
				curr_asym_dots = h_pts;
				isAsymTracking = true;
			}
		}
		else
		{
			isSymTracking = false;
			isAsymTracking = false;
			return false;
		}
	}

	// Determine which part(s) are tracked
	if (!curr_asym_dots.empty())
	{
		// A vector from point 0 (origin) to last point is used to determine orientation of the asym grid
		const bool isTop = (curr_asym_dots.back().x - curr_asym_dots[0].x) > 0;
		if (!curr_sym_dots.empty())
		{
			curr_state = isTop ? MID_CIR | TOP_CIR : MID_CIR | BOT_CIR;
			if (!isTop)
			{
				// Bottom, if symmetric pattern has wrong orientation
				if(curr_sym_dots[0] == curr_asym_dots.back())
					std::reverse(std::begin(curr_sym_dots), std::end(curr_sym_dots));
			}
			else
			{
				// Top
				if(curr_sym_dots.back() == curr_asym_dots.back())
					std::reverse(std::begin(curr_sym_dots), std::end(curr_sym_dots));
			}
		}
		else
			curr_state = isTop ? TOP_CIR : BOT_CIR;
	}
	else
	{
		// Check correct orientation
		// Assume symm marker starting from right
		if (curr_sym_dots[0].x < curr_sym_dots.back().x)
			std::reverse(std::begin(curr_sym_dots), std::end(curr_sym_dots));
		curr_state = MID_CIR;
	}

	// Determine which chess line are detected
	
	auto slope = 0.0f;
	auto orient_inside = 0, orient_outside = 0, orient_now = 0;
	
	cv::Point2f diff_pt;
	if (!curr_chess_dots.empty() && m_chess_detector.Orientation(orient_now))
	{
		if (curr_state & TOP_CIR)
		{
			diff_pt = curr_asym_dots.back() - curr_asym_dots.front();
			slope = diff_pt.y / diff_pt.x;
			calc_chess_orient(slope, orient_inside, orient_outside);
			if (similar_orientation(orient_inside, orient_now))
				curr_state = curr_state | MID_CHESS;
			else if (similar_orientation(orient_outside, orient_now))
				curr_state = curr_state | TOP_CHESS;
		}
		else if (curr_state & BOT_CIR)
		{
			diff_pt = curr_asym_dots.front() - curr_asym_dots.back();
			slope = diff_pt.y / diff_pt.x;
			calc_chess_orient(slope, orient_inside, orient_outside);
			if (similar_orientation(orient_inside, orient_now))
				curr_state = curr_state | MID_CHESS;
			else if (similar_orientation(orient_outside, orient_now))
				curr_state = curr_state | BOT_CHESS;
		}
		else if (curr_state & MID_CIR)
		{
			diff_pt = curr_sym_dots.front() - curr_sym_dots.back();
			slope = diff_pt.y / diff_pt.x;
			calc_chess_orient(slope, orient_inside, orient_outside);
			if (similar_orientation(orient_inside, orient_now))
				curr_state = curr_state | MID_CHESS;
		}
	}

	// Update tracking and detection for next image
	if (binitSymTracker)
	{
		pre_sym_gray = cur_gray.clone();
		prev_sym_dots = curr_sym_dots;
	}
	if (binitAsymTracker)
	{
		pre_asym_gray = cur_gray.clone();
		prev_asym_dots = curr_asym_dots;
	}

	if (!curr_sym_dots.empty())
		UpdateLastLocation(curr_sym_dots);
	else if (!curr_asym_dots.empty())
		UpdateLastLocation(curr_asym_dots);

	// Update adaptive threshold for marker detection
	if (isSymTracking && isAsymTracking)
	{
		m_thresh_dot_chess = 10;
		m_thresh_chess = cur_image.cols / 4;
	}
	else
	{
		// If Detection, set threshold as a portion of end dots
		auto end_dots_dist = 0.0f;
		
		if (!curr_asym_dots.empty())
			end_dots_dist = cv::norm(curr_asym_dots.front() - curr_asym_dots.back());
		else if (!curr_sym_dots.empty())
			end_dots_dist = cv::norm(curr_sym_dots.front() - curr_sym_dots.back());


		m_thresh_dot_chess = end_dots_dist / 16;
		m_thresh_chess = end_dots_dist / 2;
	}


	return true;
}

bool TrackerCurvedot::DetectPattern(const cv::Mat& _img_gray,
									std::vector<cv::Point2f>& _symm_dots,
									std::vector<cv::Point2f>& _asymm_dots,
									std::vector<cv::Point2f>& _chess_pts)
{
	bool found;

	// Detect chess vertice first which should be excluded in FindDots
	std::vector<cv::Point2f> chess_pts, all_chess_pts;
	cv::Mat img_burr;
	cv::blur(_img_gray, img_burr, cv::Size(5, 5));

	m_chess_found = m_chess_detector.detect(img_burr, chess_pts, all_chess_pts, m_thresh_chess);//m_thresh_chess
	_chess_pts = chess_pts;

	if (!chess_pts.empty())
	{
		found = FindDots(_img_gray, sym_pattern_size, asym_pattern_size,
			_symm_dots, _asymm_dots,
			blob_detector, chess_pts);
	} 
	else
	{
		found = FindDots(_img_gray, sym_pattern_size, asym_pattern_size,
			_symm_dots, _asymm_dots,
			blob_detector);
	}
	
	// Two attempts

// 	if (!found)
// 	{
// 		// crop a roi for 2nd attempts
// 		cv::Rect rect = cv::Rect((int)last_valid_location.x, (int)last_valid_location.y, 
// 			2*roi_hw, 2*roi_hh);
// 		rect.x = rect.x < 0? 0 : rect.x;
// 		rect.y = rect.y < 0? 0 : rect.y;
// 		rect.width = (rect.x+rect.width) > (_img_gray.cols - 1) 
// 			? (_img_gray.cols-rect.x-1) : rect.width;
// 		rect.height = (rect.y+rect.height) > (_img_gray.rows - 1)
// 			? (_img_gray.rows-rect.y-1) : rect.height;
// 
// 		//cv::rectangle(cur_image, rect, CV_RGB(250,250, 0));
// 		cv::Mat roi = _img_gray(rect);
// 		found = FindDots(roi, sym_pattern_size, asym_pattern_size, _symm_dots, _asymm_dots,
// 			roi_blob_detector);
// 		if (found)
// 		{
// 			for (unsigned int i = 0; i < _symm_dots.size(); i++)
// 			{
// 				_symm_dots[i].x = _symm_dots[i].x + last_valid_location.x;
// 				_symm_dots[i].y = _symm_dots[i].y + last_valid_location.y;
// 				std::cout << _symm_dots[i] << ", ";
// 			}
// 			for (unsigned int i = 0; i < _asymm_dots.size(); i++)
// 			{
// 				_asymm_dots[i].x = _asymm_dots[i].x + last_valid_location.x;
// 				_asymm_dots[i].y = _asymm_dots[i].y + last_valid_location.y;
// 			}
// 			std::cout << std::endl;
// 			std::cout << "2nd Try found: " << _symm_dots.size() << ", " << _asymm_dots.size() << std::endl;
// 		}
// 	}
	return found;
}

bool TrackerCurvedot::FindDots(cv::InputArray _image, cv::Size sym_patternSize, cv::Size asym_patternSize,
							   cv::OutputArray _sym_centers, cv::OutputArray _asym_centers, 
							   const cv::Ptr<cv::FeatureDetector> &blobDetector,
							   const std::vector<cv::Point2f> &chess_pts)
{
	cv::Mat image = _image.getMat();
	std::vector<cv::Point2f> sym_centers, asym_centers;

	std::vector<cv::KeyPoint> keypoints;

	blobDetector->detect(image, keypoints);

	// Filter detection too close to chess points
	std::vector<cv::Point2f> points;
	for (size_t i = 0; i < keypoints.size(); i++)
	{
		bool close_to_chess = false;

		for (auto j = 0; j < chess_pts.size(); j++)
		{
			close_to_chess = cv::norm(keypoints[i].pt - chess_pts[j]) < m_thresh_dot_chess ? true : false;
			if (close_to_chess) break;
		}
		if (!close_to_chess)
		{
			points.push_back(keypoints[i].pt);
		}
	}

// 	if (points.size() >1)
// 	{
// 		cv::Mat out;
// 		
// // 		cv::drawKeypoints(image, keypoints, out, cv::Scalar(0,0,255));
// 		cvtColor(image, out, CV_GRAY2BGR);
// 		for(size_t i=0; i<chess_pts.size(); i++)
// 		{
// 			circle(out, chess_pts[i], 2, cv::Scalar(0,255,255), 2, -1);
// 		}
// 
// 		cv::imwrite("result.png", out);
// 	}


	AsymmCirclesGridClusterFinder.findGrid(points, asym_patternSize, asym_centers);
	if (asym_centers.empty())
		SymmCirclesGridClusterFinder.findGrid(points, sym_patternSize, sym_centers);
	else
	{
		std::vector<uchar> ex_mask = AsymmCirclesGridClusterFinder.getAsmSegMask();
		// If asymmetric grid detected, need to exclude short seg
		SymmCirclesGridClusterFinder.findGridwithExMask(points, sym_patternSize, ex_mask, sym_centers);
	}

	//////////////////////////////////////////////////////////////////////////
	// HERE ///
	
	cv::Mat(sym_centers).copyTo(_sym_centers);
	cv::Mat(asym_centers).copyTo(_asym_centers);
	return !(_sym_centers.empty() && _asym_centers.empty());


// 	CirclesGridFinderParameters parameters;
// 	parameters.vertexPenalty = -0.6f;
// 	parameters.vertexGain = 1;
// 	parameters.existingVertexGain = 10000;
// 	parameters.edgeGain = 1;
// 	parameters.edgePenalty = -0.6f;
// 
// 	if(flags & cv::CALIB_CB_ASYMMETRIC_GRID)
// 		parameters.gridType = CirclesGridFinderParameters::ASYMMETRIC_GRID;
// 	if(flags & cv::CALIB_CB_SYMMETRIC_GRID)
// 		parameters.gridType = CirclesGridFinderParameters::SYMMETRIC_GRID;
// 
// 	const int attempts = 2;
// 	const size_t minHomographyPoints = 4;
// 	cv::Mat H;
// 	for (int i = 0; i < attempts; i++)
// 	{
// 		centers.clear();
// 		// Two boxFinder: one for symmetric, one for asymmetric
// 		CirclesGridFinder boxFinder(patternSize, points, parameters);
// 		bool isFound = false;
// 
// 		try
// 		{
// 			isFound = boxFinder.findHoles();
// 		}
// 		catch (cv::Exception)
// 		{
// 
// 		}
// 
// 		if (isFound)
// 		{
// 			switch(parameters.gridType)
// 			{
// 			case CirclesGridFinderParameters::SYMMETRIC_GRID:
// 				boxFinder.getHoles(centers);
// 				break;
// 			case CirclesGridFinderParameters::ASYMMETRIC_GRID:
// 				boxFinder.getAsymmetricHoles(centers);
// 				break;
// 			default:
// 				CV_Error(CV_StsBadArg, "Unkown pattern type");
// 			}
// 
// 			if (i != 0)
// 			{
// 				cv::Mat orgPointsMat;
// 				cv::transform(centers, orgPointsMat, H.inv());
// 				cv::convertPointsFromHomogeneous(orgPointsMat, centers);
// 			}
// 			cv::Mat(centers).copyTo(_centers);
// 			return true;
// 		}
// 
// 		boxFinder.getHoles(centers);
// 		if (i != attempts - 1)
// 		{
// 			if (centers.size() < minHomographyPoints)
// 				break;
// 			H = CirclesGridFinder::rectifyGrid(boxFinder.getDetectedGridSize(), centers, points, points);
// 		}
// 	}
// 	cv::Mat(centers).copyTo(_centers);
// 	return false;
}

/************************************************************************/
/* Tracking part                                                        */
/************************************************************************/

void TrackerCurvedot::initSymTrack(cv::Mat& _pre_gray,
							  std::vector<cv::Point2f> _prev_dots)
{
	pre_sym_gray = _pre_gray.clone();
	prev_sym_dots = _prev_dots;
	pre_sym_status.resize(sym_model_dots.size());
	std::fill(pre_sym_status.begin(), pre_sym_status.end(), 1);
	binitSymTracker = true;
}

void TrackerCurvedot::initAsymTrack(cv::Mat& _pre_gray,
								   std::vector<cv::Point2f> _prev_dots)
{
	pre_asym_gray = _pre_gray.clone();
	prev_asym_dots = _prev_dots;
	pre_asym_status.resize(asym_model_dots.size());
	std::fill(pre_asym_status.begin(), pre_asym_status.end(), 1);
	binitAsymTracker = true;
}

bool TrackerCurvedot::TrackPattern(const cv::Mat& _cur_gray, cv::Mat& _sym_H, cv::Mat& _asym_H)
{
	bool valid_sym_homography = false, valid_asym_homography = false;
	std::vector<cv::Point2f> _dots;
	std::vector<unsigned char> status;
	cv::Mat sym_H, asym_H;

	// --------- Symmetric ---------
	if (binitSymTracker && prev_sym_dots.size() > 0)
	{
		DoSpaseOpticalFlow(pre_sym_gray, _cur_gray, prev_sym_dots, _dots, status);
		if (std::accumulate(status.begin(), status.end(), 0) > status.size() *0.9)
		{
			std::vector<cv::Point2f> mod_pts;
			std::vector<cv::Point2f> dsc_pts;
			for (unsigned int i = 0; i < pre_sym_status.size(); i++)
			{
				status[i] = !pre_sym_status[i] && status[i] ? 0 : 1;
				if (pre_sym_status[i] && status[i])
				{
					mod_pts.push_back(sym_model_dots[i]);
					dsc_pts.push_back(_dots[i]);
				}
			}
			pre_sym_status = status;

			std::vector<unsigned char> inliers;
			if(mod_pts.size()>=4 && dsc_pts.size()>=4)
				sym_H = cv::findHomography(mod_pts, dsc_pts, inliers, CV_RANSAC, square_size * 2.0);


			double count = 0;
			for (unsigned int i = 0; i < inliers.size(); i++)
			{
				if (inliers[i])
					count = count + 1.;
			}
			if (count/status.size() > 0.8)
			{
				valid_sym_homography = true;
				sym_H.copyTo(_sym_H);
			}
			else
			{
				_sym_H = cv::Mat();
				binitSymTracker = false;
			}
		}
		else
		{
			_sym_H = cv::Mat();
			binitSymTracker = false;
		}
		
	}
	// --------- Asymmetric ---------
	if (binitAsymTracker && prev_asym_dots.size() > 0)
	{
		
		DoSpaseOpticalFlow(pre_asym_gray, _cur_gray, prev_asym_dots, _dots, status);
		std::vector<cv::Point2f> mod_pts;
		std::vector<cv::Point2f> dsc_pts;
		if (std::accumulate(status.begin(), status.end(), 0) > status.size() / 2)
		{
			for (unsigned int i = 0; i < pre_asym_status.size(); i++)
			{
				status[i] = !pre_asym_status[i] && status[i] ? 0 : 1;
				if (pre_asym_status[i] && status[i])
				{
					mod_pts.push_back(asym_model_dots[i]);
					dsc_pts.push_back(_dots[i]);
				}
			}
			pre_asym_status = status;

			std::vector<unsigned char> inliers;
			if(mod_pts.size()>=4 && dsc_pts.size()>=4)
				asym_H = cv::findHomography(mod_pts, dsc_pts, inliers, CV_RANSAC, square_size * 2.0);


			double count = 0;
			for (unsigned int i = 0; i < inliers.size(); i++)
			{
				if (inliers[i])
					count = count + 1.;
			}
			if (count/status.size() > 0.5)
			{
				valid_asym_homography = true;
				asym_H.copyTo(_asym_H);
			}
			else
			{
				_asym_H = cv::Mat();
				binitAsymTracker = false;
			}
		}
		else
		{
			_asym_H = cv::Mat();
			binitAsymTracker = false;
		}
		
	}

	return valid_sym_homography || valid_asym_homography;
}


void TrackerCurvedot::drawKeydots(cv::InputOutputArray _image)
{
	cv::Mat image = _image.getMat();
	cv::Scalar sym_bgr, asym_bgr;
	sym_bgr = isSymTracking ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 255, 0);
	asym_bgr = isAsymTracking ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 255, 0);

	if ((curr_state & MID_CIR) || (curr_state & (MID_CIR & TOP_CIR)) || (curr_state & (MID_CIR & BOT_CIR)))
	{
		cv::perspectiveTransform(sym_corner_pts, curr_sym_corners, sym_homography);


		for (unsigned int i = 0; i < curr_sym_dots.size(); i++)
		{
			cv::circle(image, curr_sym_dots[i], 4, sym_dot_colors[i], 1, CV_AA);
			cv::line(image, cv::Point2f(curr_sym_dots[i].x-3, curr_sym_dots[i].y), 
				cv::Point2f(curr_sym_dots[i].x+3, curr_sym_dots[i].y), sym_dot_colors[i]);
			cv::line(image,cv::Point2f( curr_sym_dots[i].x, curr_sym_dots[i].y-3),
				cv::Point2f(curr_sym_dots[i].x, curr_sym_dots[i].y+3), sym_dot_colors[i] );
		}
// 		cv::circle(image, curr_sym_dots[0], 8, cv::Scalar(255, 10, 10), 2, CV_AA);

		cv::line(image, curr_sym_corners[0], curr_sym_corners[1], sym_bgr, 2, CV_AA);
		cv::line(image, curr_sym_corners[1], curr_sym_corners[2], sym_bgr, 2, CV_AA);
		cv::line(image, curr_sym_corners[2], curr_sym_corners[3], sym_bgr, 2, CV_AA);
		cv::line(image, curr_sym_corners[0], curr_sym_corners[3], sym_bgr, 2, CV_AA);
	}

	if ((curr_state & (MID_CIR & TOP_CIR)) || (curr_state & (MID_CIR & BOT_CIR))
		|| (curr_state & TOP_CIR) || (curr_state & BOT_CIR))
	{
		cv::perspectiveTransform(asym_corner_pts, curr_asym_corners, asym_homography);


		for (unsigned int i = 0; i < curr_asym_dots.size(); i++)
		{
			cv::circle(image, curr_asym_dots[i], 4, asym_dot_colors[i], 1, CV_AA);
			cv::line(image, cv::Point2f(curr_asym_dots[i].x-3, curr_asym_dots[i].y), 
				cv::Point2f(curr_asym_dots[i].x+3, curr_asym_dots[i].y), asym_dot_colors[i]);
			cv::line(image,cv::Point2f( curr_asym_dots[i].x, curr_asym_dots[i].y-3),
				cv::Point2f(curr_asym_dots[i].x, curr_asym_dots[i].y+3), asym_dot_colors[i] );
			if (i < curr_asym_dots.size() - 2)
			{
				cv::line(image,cv::Point2f( curr_asym_dots[i].x, curr_asym_dots[i].y),
					cv::Point2f(curr_asym_dots[i+1].x, curr_asym_dots[i+1].y), asym_dot_colors[i] );
			}
		}

		cv::line(image, curr_asym_corners[0], curr_asym_corners[1], asym_bgr, 2, CV_AA);
		cv::line(image, curr_asym_corners[1], curr_asym_corners[2], asym_bgr, 2, CV_AA);
		cv::line(image, curr_asym_corners[2], curr_asym_corners[3], asym_bgr, 2, CV_AA);
		cv::line(image, curr_asym_corners[0], curr_asym_corners[3], asym_bgr, 2, CV_AA);

// 		cv::circle(image, curr_asym_dots[0], 8, cv::Scalar(255, 0, 0), 2, CV_AA);
	}
	
	if (!curr_chess_dots.empty())
	{
		for (auto i = 0; i < curr_chess_dots.size(); i++)
			cv::circle(image, curr_chess_dots[i], 2, cv::Scalar(0, 255, 255), 2, CV_AA);	

// 		cv::Point pt1, pt2;
// 		pt1.x = 10; pt1.y = (-m_chess_line[0] * pt1.x - m_chess_line[2])/m_chess_line[1];
// 		pt2.x = 900; pt2.y = (-m_chess_line[0] * pt2.x - m_chess_line[2])/m_chess_line[1];
// 		cv::line(image, pt1, pt2, cv::Scalar(0, 255, 255));
	}

	std::string disp_str;
	if (curr_state & (MID_CIR & TOP_CIR))
		disp_str = "MID_TOP";
	else if (curr_state & (MID_CIR & BOT_CIR))
		disp_str = "MID_BOT";
	else if (curr_state & TOP_CIR)
		disp_str = "TOP";
	else if (curr_state & BOT_CIR)
		disp_str = "BOT";
	else if (curr_state & MID_CIR)
		disp_str = "MID";

// 	if (!asym_corner_pts.empty())
// 		cv::putText(image, disp_str, curr_asym_corners[0], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 3,8);
// 	else
// 		cv::putText(image, disp_str, curr_sym_corners[0], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 3,8);


}

std::vector<cv::Point2f> TrackerCurvedot::getP_img()
{
	std::vector<cv::Point2f> out_pts;
	if (((curr_state & TOP_CIR) || (curr_state & BOT_CIR)) && !(curr_state & MID_CIR))
	{
		out_pts = curr_asym_dots;
	}
	else if (((curr_state & TOP_CIR) || (curr_state & BOT_CIR)) && (curr_state & MID_CIR))
	{
		int num_sym_dot = curr_sym_dots.size();
		int num_asym_dot = curr_asym_dots.size();
		out_pts.reserve(num_sym_dot/2 + num_asym_dot);
		out_pts.insert(out_pts.end(), curr_asym_dots.begin(), curr_asym_dots.end());
		if (curr_state & TOP_CIR)
		{
			for (int i = 0; i < num_sym_dot; i++)
			{
				if (i%2 != 0)
					out_pts.push_back(curr_sym_dots[i]);
			}
		}
		else
		{
			for (int i = 0; i < num_sym_dot; i++)
			{
				if (i%2 == 0)
					out_pts.push_back(curr_sym_dots[i]);
			}
		}
	}
	else if ((curr_state & MID_CIR))
		out_pts = curr_sym_dots;
		
	return out_pts;
}

cv::Vec3f TrackerCurvedot::get_chess_line()
{
	cv::Vec4f line_ps;
	cv::fitLine(curr_chess_dots, line_ps, CV_DIST_L1, 0, 0.01, 0.01);

	// Convert point-slope form to general form
	auto m = line_ps[1] / line_ps[0];
	m_chess_line[0] = m;	// A = m
	m_chess_line[1] = -1;	// B
	m_chess_line[2] = -m * line_ps[2] + line_ps[3];	// C = -m * x0 + y_0

	return m_chess_line;
}

std::vector<cv::Point2f> TrackerCurvedot::get_chess_pts()
{
	return curr_chess_dots;
}

void TrackerCurvedot::calc_chess_orient(const float &slope, int &label_mid, int &label_out)
{
	if (slope < -5)
	{
		label_mid = 3;
		label_out = -1;
	}
	else if (slope >= -5 && slope < -1.5)
	{
		label_mid = -4;
		label_out = 0;
	}
	else if (slope >= -1.5 && slope < -0.67)
	{
		label_mid = -3;
		label_out = 1;
	}
	else if (slope >= -0.67 && slope < -0.2)
	{
		label_mid = -2;
		label_out = 2;
	}
	else if (slope >= -0.2 && slope < 0.2)
	{
		// slope = 0
		label_mid = -1;
		label_out = 3;
	}
	else if (slope >= 0.2 && slope < 0.67)
	{
		label_mid = 0;
		label_out = -4;
	}
	else if (slope >= 0.67 && slope < 1.5)
	{
		label_mid = 1;
		label_out = -3;
	}
	else if (slope >= 1.5 && slope < 5)
	{
		label_mid = 2;
		label_out = -2;
	}
	else
	{
		// slope >= 5
		label_mid = 3;
		label_out = -1;
	}
}