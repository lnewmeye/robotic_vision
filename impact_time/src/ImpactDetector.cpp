// EcEn 672 - Robotic Vision
// Assigment 7 - ImpactDetector Class
// Luke Newmeyer

#include "ImpactDetector.hpp"

#define DEBUG
#ifdef DEBUG
#include <iostream>
#endif

using std::vector;
using cv::Mat;
using cv::Point2f;
using cv::Scalar;

void ImpactDetector::setInitial(cv::Mat image)
{
	// Find features on image
	vector<Point2f> initial_corners;
	cv::goodFeaturesToTrack(image, initial_corners, IMPACT_CORNER_MAX,
			IMPACT_CORNER_QUALITY, IMPACT_CORNER_DISTANCE);

	// Push corners found and image to interior data structures
	previous_images.push_back(image.clone());
	previous_corners.push_back(initial_corners);
}

void ImpactDetector::detectImpact(cv::Mat image, cv::Mat& display)
{
	// Find matched features on image
	Mat previous_image = previous_images.back();
	vector<Point2f> last_corners = previous_corners.back();
	vector<Point2f> new_corners;
	vector<unsigned char> status;
	vector<float> error;
	cv::calcOpticalFlowPyrLK(previous_image, image, last_corners,
			new_corners, status, error);

	// Remove bad corners
	for (unsigned i = status.size(); i > 0; i--) {
		if (status[i] == 0) {
			std::cout << "Remove point " << new_corners[i] << std::endl;
			//for (unsigned j = 0; j < previous_images.size(); j++) {
				//previous_corners[j].erase(previous_corners[j].begin()+j-1);
			//}
		}
	}

	// Draw new corners on image
	Scalar color(0, 0, 255);
	cv::cvtColor(image, display, CV_GRAY2BGR);
	drawCorners(display, new_corners, color);

	// Push back corners found and image
	previous_images.push_back(image);
	previous_corners.push_back(new_corners);

	// Iterate through previous corners and remove bad corners
}

void ImpactDetector::drawCorners(cv::Mat& image, 
		std::vector<cv::Point2f> corners, cv::Scalar color)
{
	// Iterate through corners and draw on image
	for (Point2f corner : corners) {
		cv::circle(image, corner, IMPACT_CORNER_DRAW_SIZE, color, -1);
	}
}
