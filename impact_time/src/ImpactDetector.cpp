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
using cv::Point2d;
using cv::Scalar;

void ImpactDetector::setInitial(cv::Mat image)
{
	std::cout << "Made it here" << std::endl;
	// Find features on image
	vector<Point2d> initial_corners;
	cv::goodFeaturesToTrack(image, initial_corners, IMPACT_CORNER_MAX,
			IMPACT_CORNER_QUALITY, IMPACT_CORNER_DISTANCE);

	std::cout << "Made it here" << std::endl;
	// Push corners found and image to interior data structures
	previous_images.push_back(image.clone());
	previous_corners.push_back(initial_corners);
}

void ImpactDetector::detectImpact(cv::Mat image, cv::Mat& display)
{
	// Find matched features on image
	Mat previous_image = previous_images.back();
	vector<Point2d> last_corners = previous_corners.back();
	vector<Point2d> new_corners;
	vector<uchar> status;
	vector<double> error;
	cv::calcOpticalFlowPyrLK(previous_image, image, last_corners,
			new_corners, status, error);

	// Remove bad corners

	std::cout << "Made it here" << std::endl;

	// Draw corners on image
	Scalar color(0, 0, 255);
	cv::cvtColor(image, display, CV_GRAY2BGR);
	drawCorners(display, new_corners, color);

	// Push back corners found and image
	previous_images.push_back(image);
	previous_corners.push_back(new_corners);

	// Iterate through previous corners and remove bad corners
}

void ImpactDetector::drawCorners(cv::Mat& image, 
		std::vector<cv::Point2d> corners, cv::Scalar color)
{
	// Iterate through corners and draw on image
	for (Point2d corner : corners) {
		cv::circle(image, corner, IMPACT_CORNER_DRAW_SIZE, color, -1);
	}
}
