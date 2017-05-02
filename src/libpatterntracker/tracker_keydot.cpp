#include "tracker_keydot.h"

TrackerKeydot::TrackerKeydot(cv::Size _pattern_size, cv::Size _roi_size,
							 cv::SimpleBlobDetector::Params params,
							 cv::SimpleBlobDetector::Params params_roi) :
	last_valid_location(cv::Point2f(200, 200)),
	pattern_size(_pattern_size), square_size (1.0f),
	roi_hw(_roi_size.width/2), roi_hh(_roi_size.height/2),
	binitTracker(false), bisTracking(false)
{
	blob_detector = cv::SimpleBlobDetector::create(params);
	roi_blob_detector = cv::SimpleBlobDetector::create(params_roi);
}

TrackerKeydot::TrackerKeydot(cv::Size _pattern_size, int flag,
							 cv::Size _roi_size,
                             cv::SimpleBlobDetector::Params params,
                             cv::SimpleBlobDetector::Params params_roi) :
    last_valid_location(cv::Point2f(200, 200)),
    pattern_size(_pattern_size), square_size (1.0f),
    roi_hw(_roi_size.width/2), roi_hh(_roi_size.height/2),
    binitTracker(false), bisTracking(false),
	pattern_type(flag)
{
    blob_detector = cv::SimpleBlobDetector::create(params);
    roi_blob_detector = cv::SimpleBlobDetector::create(params_roi);

    // record model points on the grid
    for( int i = 0; i < pattern_size.height; i++ )
    {
        for( int j = 0; j < pattern_size.width; j++ )
        {
            cv::Point2f idealPt;
			if(pattern_type == cv::CALIB_CB_ASYMMETRIC_GRID)
				idealPt = cv::Point2f((2*j + i % 2)*square_size, i*square_size);
			else
				idealPt = cv::Point2f(j*square_size, i*square_size);
			model_dots.push_back(idealPt);
        }
    }

    // 3-----0
    // |*****|
    // 2-----1
	if (pattern_type == cv::CALIB_CB_ASYMMETRIC_GRID)
	{
		corner_pts.push_back(cv::Point2f(0, 0));
		corner_pts.push_back(cv::Point2f((pattern_size.width*2-1)*square_size, square_size));
		corner_pts.push_back(cv::Point2f((pattern_size.width*2-1)*square_size, (pattern_size.height-2)*square_size));
		corner_pts.push_back(cv::Point2f(0, (pattern_size.height-1)*square_size));
	}
	else
	{
		corner_pts.push_back(cv::Point2f(0, 0));
		corner_pts.push_back(cv::Point2f((pattern_size.width-1)*square_size, 0));
		corner_pts.push_back(cv::Point2f((pattern_size.width-1)*square_size, (pattern_size.height-1)*square_size));
		corner_pts.push_back(cv::Point2f(0, (pattern_size.height-1)*square_size));
	}
    

    // generate colors for identity
    //////////////////////////////////////////////////////////////////////////
    cv::RNG rng(12345);
    for (int i = 0; i < pattern_size.height; i++)
    {
        cv::Scalar color = cv::Scalar(rng.uniform(100, 255), rng.uniform(100, 255),rng.uniform(100, 255)); //unknow color
        for (int j = 0; j < pattern_size.width; j++)
            dot_colors.push_back(color);
    }

}

bool TrackerKeydot::track(const cv::Mat &cur_image)
{
	cv::Mat cur_gray;
	cv::cvtColor(cur_image, cur_gray, cv::COLOR_BGR2GRAY);

	bool found = DetectPattern(cur_gray, curr_dots);

	if (found)
	{
		std::vector<unsigned char> inls;
		//homography = cv::findHomography(model_dots, curr_dots, inls, CV_RANSAC, square_size*0.5); 
		homography = cv::findHomography(model_dots, curr_dots, CV_RANSAC, square_size*3, inls);

		bisTracking = false;

		if (!binitTracker)
		{
			initTrack(model_dots, cur_gray, curr_dots);
		}

		// Once pattern is detected, update tracking for next image
		UpdateStatus();
	}
	else // If detection is not good -> perform tracking using optical flow
	{
 		std::vector<cv::Point2f> cur_pts;
 		bool homography_valid = TrackPattern(cur_gray, cur_pts, homography);
 
 		if (homography_valid)
 		{
 			std::vector<cv::Point2f> h_pts;
 			cv::perspectiveTransform(model_dots, h_pts, homography);
 			curr_dots = h_pts;
 			bisTracking = true;
 
 		}
 		else
			return false;
	}

	// Update tracking and detection for next image
	if (binitTracker)
		UpdateLastDots(cur_gray, curr_dots);
	UpdateLastLocation(curr_dots);


	return true;
}

bool TrackerKeydot::DetectPattern(const cv::Mat& _img_gray, std::vector<cv::Point2f>& _dots)
{
	// Two attempts
	bool found = FindDots(_img_gray, pattern_size, _dots, 
		pattern_type | cv::CALIB_CB_CLUSTERING, blob_detector);

	if (!found)
	{
		// crop a roi for detection
		cv::Rect rect = cv::Rect((int)last_valid_location.x, (int)last_valid_location.y, 
			2*roi_hw, 2*roi_hh);
		rect.x = rect.x < 0? 0 : rect.x;
		rect.y = rect.y < 0? 0 : rect.y;
		rect.width = (rect.x+rect.width) 
		> (_img_gray.cols - 1)? (_img_gray.cols-rect.x-1) : rect.width;
		rect.height = (rect.y+rect.height) 
		> (_img_gray.rows - 1)? (_img_gray.rows-rect.y-1) : rect.height;

		cv::Mat roi = _img_gray(rect);
		found = FindDots(roi, pattern_size, _dots, 
			pattern_type | cv::CALIB_CB_CLUSTERING, roi_blob_detector);
		if (found)
		{
			for (unsigned int i = 0; i < _dots.size(); i++)
			{
				_dots[i].x = _dots[i].x + last_valid_location.x;
				_dots[i].y = _dots[i].y + last_valid_location.y;
			}
		}
	}
	return found;
}

bool TrackerKeydot::FindDots(cv::InputArray _image, cv::Size patternSize, cv::OutputArray _centers,
								int flags, const cv::Ptr<cv::FeatureDetector> &blobDetector)
{
	bool isAsymmetricGrid = (flags & cv::CALIB_CB_ASYMMETRIC_GRID) ? true : false;
	bool isSymmetricGrid  = (flags & cv::CALIB_CB_SYMMETRIC_GRID ) ? true : false;
	CV_Assert(isAsymmetricGrid ^ isSymmetricGrid);

	cv::Mat image = _image.getMat();
	std::vector<cv::Point2f> centers;

	std::vector<cv::KeyPoint> keypoints;
	blobDetector->detect(image, keypoints);
	std::vector<cv::Point2f> points;
	for (size_t i = 0; i < keypoints.size(); i++)
	{
		points.push_back(keypoints[i].pt);
	}

	if(flags & cv::CALIB_CB_CLUSTERING)
	{
		CirclesGridClusterFinder circlesGridClusterFinder(isAsymmetricGrid, patternSize.width == 1);
		circlesGridClusterFinder.findGrid(points, patternSize, centers);
		cv::Mat(centers).copyTo(_centers);
		return !centers.empty();
	}

	CirclesGridFinderParameters parameters;
	parameters.vertexPenalty = -0.6f;
	parameters.vertexGain = 1;
	parameters.existingVertexGain = 10000;
	parameters.edgeGain = 1;
	parameters.edgePenalty = -0.6f;

	if(flags & cv::CALIB_CB_ASYMMETRIC_GRID)
		parameters.gridType = CirclesGridFinderParameters::ASYMMETRIC_GRID;
	if(flags & cv::CALIB_CB_SYMMETRIC_GRID)
		parameters.gridType = CirclesGridFinderParameters::SYMMETRIC_GRID;

	const int attempts = 2;
	const size_t minHomographyPoints = 4;
	cv::Mat H;
	for (int i = 0; i < attempts; i++)
	{
		centers.clear();
		CirclesGridFinder boxFinder(patternSize, points, parameters);
		bool isFound = false;

		try
		{
			isFound = boxFinder.findHoles();
		}
		catch (cv::Exception)
		{

		}

		if (isFound)
		{
			switch(parameters.gridType)
			{
			case CirclesGridFinderParameters::SYMMETRIC_GRID:
				boxFinder.getHoles(centers);
				break;
			case CirclesGridFinderParameters::ASYMMETRIC_GRID:
				boxFinder.getAsymmetricHoles(centers);
				break;
			default:
				CV_Error(CV_StsBadArg, "Unkown pattern type");
			}

			if (i != 0)
			{
				cv::Mat orgPointsMat;
				cv::transform(centers, orgPointsMat, H.inv());
				cv::convertPointsFromHomogeneous(orgPointsMat, centers);
			}
			cv::Mat(centers).copyTo(_centers);
			return true;
		}

		boxFinder.getHoles(centers);
		if (i != attempts - 1)
		{
			if (centers.size() < minHomographyPoints)
				break;
			H = CirclesGridFinder::rectifyGrid(boxFinder.getDetectedGridSize(), centers, points, points);
		}
	}
	cv::Mat(centers).copyTo(_centers);
	return false;
}

void TrackerKeydot::UpdateLastLocation(const std::vector<cv::Point2f>& _dots)
{
	if (_dots.size()>0)
	{
		cv::Point2f pt(0,0);
		for (unsigned int i = 0; i < _dots.size(); i++)
		{
			pt += _dots[i];
		}
		last_valid_location.x = static_cast<float>(cvRound(pt.x/_dots.size() - roi_hw));
		last_valid_location.y = static_cast<float>(cvRound(pt.y/_dots.size() - roi_hh));
	}
}


/************************************************************************/
/* Tracking part                                                        */
/************************************************************************/

void TrackerKeydot::initTrack(std::vector<cv::Point2f> _model_dots, cv::Mat& _pre_gray,
	std::vector<cv::Point2f> _prev_dots)
{
	pre_gray = _pre_gray.clone();
	model_dots = _model_dots;
	prev_dots = _prev_dots;
	pre_status.resize(model_dots.size());
	std::fill(pre_status.begin(), pre_status.end(), 1);
	binitTracker = true;
}

bool TrackerKeydot::TrackPattern(const cv::Mat& _cur_gray, std::vector<cv::Point2f>& _dots, cv::Mat& _H)
{
	bool homography_valid = false;
	if (binitTracker && prev_dots.size() > 0)
	{
		_dots.clear();
		std::vector<unsigned char> status;
		DoSpaseOpticalFlow(pre_gray, _cur_gray, prev_dots, _dots, status);
		std::vector<cv::Point2f> mod_pts;
		std::vector<cv::Point2f> dsc_pts;
		for (unsigned int i = 0; i < pre_status.size(); i++)
		{
			status[i] = !pre_status[i] && status[i] ? 0 : 1;
			if (pre_status[i] && status[i])
			{
				mod_pts.push_back(model_dots[i]);
				dsc_pts.push_back(_dots[i]);
			}
		}

		pre_status = status;

		std::vector<unsigned char> inliers;
		
		if(mod_pts.size()>=4 && dsc_pts.size()>=4){
			//_H = cv::findHomography(model_dots, _dots, inliers, CV_RANSAC, 0.8);
			_H = cv::findHomography(mod_pts, dsc_pts, inliers, CV_RANSAC, 1.0);
		}
		else
			return false;
		

		double count = 0;
		for (unsigned int i = 0; i < inliers.size(); i++)
		{
			if (inliers[i])
				count = count + 1.;
		}
		if (count/status.size() > 0.5)
			homography_valid = true;
	}

	return homography_valid;
}

void TrackerKeydot::DoSpaseOpticalFlow(const cv::Mat& _prev_img, const cv::Mat& _cur_img,
	const std::vector<cv::Point2f>& _prev_pts, std::vector<cv::Point2f>& _cur_pts, 
	std::vector<unsigned char>& _status)
{
	std::vector<float> errs;
	cv::calcOpticalFlowPyrLK(_prev_img, _cur_img, _prev_pts, _cur_pts, _status, errs);
}

void TrackerKeydot::UpdateLastDots(cv::Mat& _cur_gray, std::vector<cv::Point2f> _prev_dots)
{
	pre_gray = _cur_gray.clone();
	prev_dots = _prev_dots;
}

void TrackerKeydot::UpdateStatus()
{
	std::fill(pre_status.begin(), pre_status.end(), 1);
}

void TrackerKeydot::drawKeydots(cv::InputOutputArray _image)
{
	cv::Mat image = _image.getMat();
	cv::Scalar bgr;
	bgr = bisTracking ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 255, 0);

	cv::perspectiveTransform(corner_pts, curr_corners, homography);

	
	for (unsigned int i = 0; i < curr_dots.size(); i++)
	{
		cv::circle(image, curr_dots[i], 4, dot_colors[i], 1, CV_AA);
		cv::line(image, cv::Point2f(curr_dots[i].x-3, curr_dots[i].y), 
			cv::Point2f(curr_dots[i].x+3, curr_dots[i].y), dot_colors[i]);
		cv::line(image,cv::Point2f( curr_dots[i].x, curr_dots[i].y-3),
			cv::Point2f(curr_dots[i].x, curr_dots[i].y+3), dot_colors[i] );
	}

	cv::line(image, curr_corners[0], curr_corners[1], bgr, 2, CV_AA);
	cv::line(image, curr_corners[1], curr_corners[2], bgr, 2, CV_AA);
	cv::line(image, curr_corners[2], curr_corners[3], bgr, 2, CV_AA);
	cv::line(image, curr_corners[0], curr_corners[3], bgr, 2, CV_AA);

}

std::vector<cv::Point2f> TrackerKeydot::get_corners() {
	cv::perspectiveTransform(corner_pts, curr_corners, homography);
	return curr_corners;
}
