#include <opencv2/opencv.hpp>
#include <string>
#include "tracker_curvedot.h"
#include "track_helper.h"

using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
	bool need_undistort = true;


	string video_filename = "circular_test_video.mp4";	//hybrid_test_video
    VideoCapture vid_cap (video_filename);

	if(!vid_cap.isOpened())
	{
		std::cout << "Cannot open: " << video_filename << std::endl;
	}

	TrackHelper track_helper("..\\config\\Settings.xml");

	cv::namedWindow("marker tracking");
	while (true)
	{
		Mat img, img_track;
		if (!vid_cap.read(img))
			break;

		//resize(img, img, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);	for re-scale video
		track_helper.process(img, img_track);

		imshow("marker tracking", img_track);
		char key = waitKey(5);
		if (key == 27)
			break;
	}

	vid_cap.release();
    return 0;
}
