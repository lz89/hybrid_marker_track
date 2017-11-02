#include "chess_detector.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


ChessDetector::Params::Params()
{
	radius = 10;
	neighbourhood = 20;
	estimateOrientation = true;
	filterMinorOrientation = true;
}

ChessDetector::ChessDetector(const ChessDetector::Params &parameters) :
	m_points (NULL),
	m_img (NULL),
	m_resp (NULL),
	m_resp_init (false),
	m_img_width (0),
	m_img_height (0),
	m_orient (1000),
	params(parameters)
{
	
}

ChessDetector::~ChessDetector()
{
	if (m_resp_init)
	{
		delete [] m_resp;
		m_resp = NULL;
		m_resp_init = false;
	}

}

bool ChessDetector::detect(
	cv::InputArray in_image, 
	std::vector<cv::Point2f> &out_points,
	std::vector<cv::Point2f> &all_chess_points,
	const int &thresh_outlier)
{
	out_points.clear();	// Make sure out_point.size() == 0
	std::vector<cv::Point2f> temp_out_points;

	cv::Mat image = in_image.getMat();
	cv::Mat grayscaleImage;
	if (image.channels() == 3)
		cv::cvtColor(image, grayscaleImage, cv::COLOR_BGR2GRAY);
	else
		grayscaleImage = image;

	if (grayscaleImage.type() != CV_8UC1) {
		std::cerr << "Blob detector only supports 8-bit images!" << std::endl;;
	}

	m_img = (uint8_t*) grayscaleImage.data;

	if (!m_resp_init)
	{
		m_img_width = image.cols;
		m_img_height = image.rows;
		m_resp = new int16_t[m_img_width * m_img_height];
		m_resp_init = true;
	}

	memset(m_resp, 0, m_img_width * m_img_height * 2);
	corner_detect5(m_img_width, m_img_height, m_img, m_resp);

	int16_t max_resp = 0;
	for (unsigned px = 0; px < m_img_width * m_img_height; px++)
		if (m_resp[px] > max_resp)
			max_resp = m_resp[px];

	// TODO: DELETE
// 	cv::Mat resp_norm;
// 	cv::Mat resp_mat (m_img_height, m_img_width, CV_16UC1);
// 	for (auto i = 0; i < m_img_height; i++)
// 	{
// 		for (auto j = 0; j < m_img_width; j++)
// 		{
// 			if (m_resp[i*m_img_width+j] < 10)
// 			{
// 				resp_mat.at<int16_t>(i, j) = 0;
// 			} 
// 			else
// 			{
// 				resp_mat.at<int16_t>(i, j) = m_resp[i*m_img_width+j];
// 			}
// 			
// 		}
// 	}
// 	cv::normalize(resp_mat, resp_norm, 10, 255, cv::NORM_MINMAX, CV_8UC1);
// 	cv::Mat resp_norm_inv =  cv::Scalar::all(255) - resp_norm;
// 	cv::imwrite("resp_map.png",resp_norm_inv);

	unsigned thresh = max_resp >> 1;

	if (max_resp > 250)
	{
		if (non_max_sup_pts(m_img_width, m_img_height,
			m_resp, 7, params.radius, thresh, 0, params.neighbourhood,
			&new_point_list, &append_pl_point,
			(void **)&m_points) < 1)
		{
			m_orient = 1000;	// invalid value
			return false;
		}

		// Orientation
		if (params.estimateOrientation)
		{
			for (int i = 0; i < m_points->occupancy; i++)
				m_points->point[i].ori = 
				assign_orientation(m_img_width, m_img, 
				cvRound(m_points->point[i].pos.x - .5f) 
				+ cvRound(m_points->point[i].pos.y - .5f) * m_img_width, 1);

			if (params.filterMinorOrientation)
			{
				auto sz = m_points->occupancy;
				auto count = 0, max_count = 0;
				int majorOrient;
				int *ori = new int[sz];
				for (int i=0;i<sz;i++)
					ori[i] = m_points->point[i].ori;

				for (auto i = 0; i < sz; i++)
				{
					int count=1;
					for (auto j = i+1; j < sz; j++)
						if (ori[i]==ori[j])
							count++;
					if (count>max_count)
					{
						max_count = count;
						majorOrient = ori[i];
					}
				}

				delete [] ori;

				for (auto i = 0; i <sz; i++)
				{
					if (similar_orientation(m_points->point[i].ori, majorOrient))
						temp_out_points.push_back(cv::Point2f(m_points->point[i].pos.x,
						m_points->point[i].pos.y));
				}

				m_orient = majorOrient;
			}
		}

		// If output points is not filled yet, output all detection
		if (temp_out_points.size() == 0)
		{
			temp_out_points.reserve(m_points->occupancy);
			for (int i = 0; i < m_points->occupancy; i++)
			{
				cv::Point pt (m_points->point[i].pos.x, m_points->point[i].pos.y);
				temp_out_points.push_back(pt);
			}
		}

		// If not much valid detection, assume failed
		if (temp_out_points.size() < 3)
		{
			m_orient = 1000;	// invalid value
			return false;
		}

		all_chess_points.clear(); 
		all_chess_points.resize(m_points->occupancy);
		for (int i = 0; i < m_points->occupancy; i++)
		{
			all_chess_points[i].x = m_points->point[i].pos.x;
			all_chess_points[i].y = m_points->point[i].pos.y;
		}
	}
	else
	{
		m_orient = 1000;	// invalid value
		return false;
	}

	// Filter outliers based on specified threshold
	if (thresh_outlier > 0)
	{
		// Apply partition 
		// All pixels within the radius tolerance distance 
		// will belong to the same class (same label)
		std::vector<int> labels;

		// With lambda function (require C++11)
		int th2 = thresh_outlier * thresh_outlier;
		int n_labels = cv::partition(temp_out_points, labels, [th2](const cv::Point2f& lhs, const cv::Point2f& rhs) {
			return ((lhs.x - rhs.x)*(lhs.x - rhs.x) + (lhs.y - rhs.y)*(lhs.y - rhs.y)) < th2;
		});

		// Find majority (largest) cluster
		if (n_labels == 1)
			out_points = temp_out_points;
		else
		{
			// Moore's voting algorithm for finding majority (more than half) elements
			auto sz = temp_out_points.size();
			auto count = 0;
			int majorityLabel;
			for (auto i = 0; i < sz; i++) {
				if (count == 0)
					majorityLabel = labels[i];
				if (labels[i] == majorityLabel)
					count++;
				else
					count--;
			}
			// Check indeed is majority
			count = 0;
			for (auto i = 0; i < sz; i++)
				if (labels[i] == majorityLabel)
					count++;
			
			// May not be necessary to check
			if (count >= sz/2)
			{
				out_points.reserve(count);
				for (auto i = 0; i < sz; i++)
				{
					if (labels[i] == majorityLabel)
						out_points.push_back(temp_out_points[i]);
				}
			}
		}
	}
	else
		out_points = temp_out_points;
	return true;
}