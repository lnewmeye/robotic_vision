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
using cv::Rect;
using cv::Size;
using cv::Point2f;
using cv::Scalar;

void ImpactDetector::setInitial(cv::Mat image, cv::Mat& display)
{
	// Select region of interest
	Rect region(IMPACT_REGION_X, IMPACT_REGION_Y,
			IMPACT_REGION_WIDTH, IMPACT_REGION_HEIGHT);
	Mat selection(image, region);

	// Find features on image
	vector<Point2f> initial_corners;
	cv::goodFeaturesToTrack(selection, initial_corners, IMPACT_CORNER_MAX,
			IMPACT_CORNER_QUALITY, IMPACT_CORNER_DISTANCE);
	cv::cornerSubPix(selection, initial_corners, Size(5,5), Size(-1,-1),
			cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001));

	// Offset corners
	for (unsigned i = 0; i < initial_corners.size(); i++) {
		initial_corners[i].x += IMPACT_REGION_X;
		initial_corners[i].y += IMPACT_REGION_Y;
	}

	// Compute distances for initial image
	vector<double> distances = findDistances(initial_corners);

	// Push corners found and image to interior data structures
	previous_images.push_back(image.clone());
	previous_corners.push_back(initial_corners);
	previous_distances.push_back(distances);

	// Draw corners and region of interest on image
	cv::cvtColor(image, display, CV_GRAY2BGR);
	Scalar color(0, 0, 255);
	drawCorners(display, initial_corners, color);
	color = Scalar(255, 0, 0);
	drawRegion(display, region, color);
}

double ImpactDetector::detectImpact(cv::Mat image, cv::Mat& display)
{
	// Find matched features on image
	Mat previous_image = previous_images.back();
	vector<Point2f> last_corners = previous_corners.back();
	vector<Point2f> new_corners;
	vector<unsigned char> status;
	vector<float> error;
	cv::calcOpticalFlowPyrLK(previous_image, image, last_corners,
			new_corners, status, error);

	// Compute size ratio from new corners
	vector<double> new_distances = findDistances(new_corners);
	double ratio = computeRatio(previous_distances.back(), 
			new_distances);
	previous_distances.push_back(new_distances);
	previous_ratios.push_back(ratio);

	// Estimate time to impact
	double frames = estimateFrames(previous_ratios);

	// Push back corners found and image
	previous_images.push_back(image);
	previous_corners.push_back(new_corners);

	// Draw new corners on image
	Scalar color(0, 0, 255);
	cv::cvtColor(image, display, CV_GRAY2BGR);
	drawCorners(display, new_corners, color);

	// Draw display image 

	return frames;
}

vector<double> ImpactDetector::findDistances(vector<Point2f> points)
{
	// Create vector of distances
	vector<double> distances;
	double distance;

	// Itereate through points and find distances
	for (Point2f point : points) {
		distance = sqrt(pow(point.x - IMPACT_CAMERA_CENTER_X, 2) +
		                pow(point.y - IMPACT_CAMERA_CENTER_Y, 2));
		distances.push_back(distance);
	}

	return distances;
}

double ImpactDetector::computeRatio(vector<double> last_distances,
		vector<double> new_distances)
{
	// Initialize cumulative value to 0.0
	double ratio = 0.0;

	// For each distance take ratio and cumulate
	for (unsigned i = 0; i < new_distances.size(); i++) {
		ratio = ratio + (new_distances[i] / last_distances[i]);
	}

	// Normalize ratio
	ratio = ratio / (double)new_distances.size();

	return ratio;
}

double ImpactDetector::estimateFrames(vector<double> ratios)
{
	// Create variable for solution
	double frames, a;

	// If nnot enough values for least squares
	//if (ratios.size() <= 5) {
	if (ratios.size() <= 50) {
		a = ratios.back();
		frames = a / (a - 1);
	}
	else { // Compute linear least squares solution

		// Create data structures for lest squres
		Mat A = cv::Mat_<double>(ratios.size(), 2);
		Mat b = cv::Mat_<double>(ratios.size(), 1);
		Mat x;

		// Form matricies
		for (unsigned i = 0; i < ratios.size(); i++) {

			// Form projection space A
			A.at<double>(i,0) = i;
			A.at<double>(i,1) = 1;

			// Form solution space b
			a = ratios[i];
			b.at<double>(i,0) = a / (a - 1);
		}

		// Solve for x
		cv::solve(A, b, x, cv::DECOMP_QR);

		//std::cout << "A =" << std::endl << A << std::endl;
		//std::cout << "b =" << std::endl << b << std::endl;
		//std::cout << "x =" << std::endl << x << std::endl;

		// Compute expected impact in frames
		frames = -x.at<double>(1,0) / x.at<double>(0,0);
	}

	return frames;
}

void ImpactDetector::drawCorners(cv::Mat& image, 
		std::vector<cv::Point2f> corners, cv::Scalar color)
{
	// Iterate through corners and draw on image
	for (Point2f corner : corners) {
		cv::circle(image, corner, IMPACT_CORNER_DRAW_SIZE, color, -1);
	}
}

void ImpactDetector::drawRegion(cv::Mat& image, cv::Rect region, 
		cv::Scalar color)
{
	cv::rectangle(image, region, color, 3);
}
