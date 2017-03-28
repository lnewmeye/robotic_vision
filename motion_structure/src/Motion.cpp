// EcEn 631 - Robotic Vision
// Structure from Motion - Motion Class
// Luke Newmeyer

#include "Motion.h"

// DEBUG define (comment out when not needed)
#define DEBUG
#ifdef DEBUG
#include <iostream>
#include <opencv2/highgui.hpp>
#define DEBUG_WINDOW_NAME "Debug Window"
#endif

// Standard namespaces used
using std::vector;

// OpenCV namespaces used
using cv::Mat;
using cv::Size;
using cv::Rect;
using cv::Point;
using cv::Scalar;
using cv::Point2f;

Motion::Motion()
{
	image_size = Size(IMAGE_WIDTH, IMAGE_HEIGHT);

#ifdef DEBUG
	cv::namedWindow(DEBUG_WINDOW_NAME, cv::WINDOW_KEEPRATIO);
#endif
}

void Motion::reset()
{
	// Empty vectors
	tracked_corners.clear();
	previous_images.clear();
	F_measurement.clear();

	// Reset matricies
	F.release();
	E.release();
	M.release();
	H1.release();
	H2.release();
	R1.release();
	R2.release();
	D.release();
	R.release();
	T.release();
}

void Motion::setInitial(Mat image)
{
	// Stup constants used for corner identification
	Size SUBPIX_SIZE(5,5);
	Size ZERO_ZONE(-1,-1);
	cv::TermCriteria SUBPIX_CRITERIA(CV_TERMCRIT_EPS + 
			CV_TERMCRIT_ITER, 40, 0.001);

	// Find features to track in image
	vector<Point2f> corners_found;
	cv::goodFeaturesToTrack(image, corners_found, MOTION_CORNER_QUANTITY,
			MOTION_CORNER_QUALITY, MOTION_CORNER_DISTANCE); 
	cv::cornerSubPix(image, corners_found, SUBPIX_SIZE, ZERO_ZONE,
			SUBPIX_CRITERIA);

	// Push corners_found to corners_tracked and save previous image
	tracked_corners.push_back(corners_found);
	previous_images.push_back(image.clone());
}

void Motion::addFrame(Mat image)
{
	// Get previous image and corners to compare with, push new image
	Mat previous_image = previous_images.back();
	previous_images.push_back(image.clone());
	vector<Point2f> corners = tracked_corners.back();
	vector<Point2f> flow_corners;
	vector<uchar> mask;
	uchar match;

	// Match points between images
	flow_corners = opticalMatchingFine(previous_image, image, corners);
	tracked_corners.push_back(flow_corners);
	F = cv::findFundamentalMat(corners, flow_corners, cv::FM_RANSAC,
			3, 0.99, mask);

	// Add fundamental matrix to vector of measurements
	F_measurement.push_back(F.clone());
	//std::cout << "F =" << std::endl << F << std::endl;

	// Remove corners not matching between imges
	for (int j = mask.size(); j > 0; j--) {

		// Pop off from mask
		match = mask.back();
		mask.pop_back();

		// Remove item off of not a match
		if (!match) {
			for (unsigned k = 0; k < tracked_corners.size(); k++) {
				tracked_corners[k].erase(tracked_corners[k].begin()+j-1);
			}
		}
	}
}

vector<Point2f> Motion::opticalMatchingFine(Mat& initial, Mat& image,
		vector<Point2f> corners)
{
	// Create variables for loop
	int x1, x2, y1, y2;
	Mat selection, block, result;
	Rect selection_rect, block_rect;
	int image_width = initial.cols;
	int image_height = initial.rows;
	vector<Point2f> matches;
	Point max_location;
	Point2f match;

	// Iterate through corners and find best fit flow
	for (Point2f corner : corners) {

		// Replace corner if it comes back negative
		if (corner.x < 1 || corner.y < 1)
			corner = Point(1,1);

		// Find positions for defining selection
		x1 = corner.x - MATCH_SELECTION_WIDTH/2;
		y1 = corner.y - MATCH_SELECTION_HEIGHT/2;
		x2 = x1 + MATCH_SELECTION_WIDTH;
		y2 = y1 + MATCH_SELECTION_HEIGHT;

		// Correct positions of wrong
		if (x1 <= 0)
			x1 = 1;
		if (y1 <= 0)
			y1 = 1;
		if (x2 >= image_width)
			x2 = image_width-1;
		if (y2 >= image_height)
			y2 = image_height-1;

		// Create rects defining regions for matching
		selection_rect = Rect(Point(x1,y1), Point(x2,y2));

		// Find positions for defining block
		x1 = corner.x - MATCH_BLOCK_WIDTH/2;
		y1 = corner.y - MATCH_BLOCK_HEIGHT/2;
		x2 = x1 + MATCH_BLOCK_WIDTH;
		y2 = y1 + MATCH_BLOCK_HEIGHT;

		// Correct positions of wrong
		if (x1 <= 0)
			x1 = 1;
		if (y1 <= 0)
			y1 = 1;
		if (x2 >= image_width)
			x2 = image_width-1;
		if (y2 >= image_height)
			y2 = image_height-1;

		// Set rect defining block
		block_rect = Rect(Point(x1,y1), Point(x2,y2));

		// Select regions of initial and match images
		selection = image(selection_rect);
		block = initial(block_rect);

		// Match block in selection
		cv::matchTemplate(selection, block, result, cv::TM_CCOEFF);
		normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, Mat());
		cv::minMaxLoc(result, NULL, NULL, NULL, &max_location);
		match.x = max_location.x + corner.x - MATCH_BLOCK_WIDTH/2;
		match.y = max_location.y + corner.y - MATCH_BLOCK_HEIGHT/2;
		matches.push_back(match);
	}

	// Resolve corner find grain location
	Size subpix_size(12,12);
	Size zero_zone(-1,-1);
	cv::TermCriteria subpix_criteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 
			40, 0.001);
	//cv::cornerSubPix(image, matches, subpix_size, zero_zone, subpix_criteria);

	return matches;
}

vector<Mat> Motion::getMotionImages()
{
	// Create vector of images to be returned
	vector<Point2f> corners, flow_corners;
	vector<Mat> display;
	Mat image;

	// Draw on images
	for (unsigned i = 1; i < previous_images.size(); i++) {

		// Get corners and image to be displayed
		corners = tracked_corners[i-1];
		flow_corners = tracked_corners[i];
		cv::cvtColor(previous_images[i-1], image, CV_GRAY2BGR);

		// Draw image and push to vector
		drawMotion(image, flow_corners, corners);
		display.push_back(image.clone());
	}

	return display;
}

void Motion::drawMotion(Mat& image, vector<Point2f> corners, 
	vector<Point2f> flow)
{
	// Create variables for loop
	Point2f corner, tip;

	// Loop through vectors and plot if status shows flow to be found
	for (unsigned i = 0; i < corners.size(); i++) {

		// Get corner and flow values
		corner = corners[i];
		tip = flow[i];

		// Draw circle and flow vector
		cv::circle(image, corner, 4, Scalar(0,255,0), -1, 8, 0);
		cv::arrowedLine(image, corner, tip, Scalar(0,0,255));
	}
}

Mat Motion::averageIntrinsic()
{
	// Reset matrix F with zeros
	F = cv::Mat_<double>::zeros(3, 3);

	// Iterate through measured parameters and sum
	for (Mat measurement : F_measurement) {
		F = F + measurement;
	}

	// Average measurements out
	F = F / (double)F_measurement.size();

	return F.clone();
}

void Motion::rectifyImage(Mat& display1, Mat& display2)
{
	// Note: this function is designed specifically for task 1 of
	// assignment 6 robotic vision. It estimates both intrinsic and
	// extrinsic parameters

	// Average intrinsic parameters to get best fundamental matrix
	averageIntrinsic();

	// Form camera matrix M
	M = cv::Mat_<double>::zeros(3, 3);
	M.at<double>(0,0) = IMAGE_FOCAL_GUESS;
	M.at<double>(1,1) = IMAGE_FOCAL_GUESS;
	M.at<double>(0,2) = IMAGE_CENTER_GUESS_X;
	M.at<double>(1,2) = IMAGE_CENTER_GUESS_Y;
	M.at<double>(2,2) = 1;

	// Form distortion matrix D
	D = (cv::Mat_<double>(5,1) << IMAGE_DISTORTION_GUESS);

	// Stereo rectify points in first and last images
	vector<Point2f> first_points = tracked_corners.front();
	vector<Point2f> last_points = tracked_corners.back();
	//vector<Point2f> last_points = tracked_corners[1];
	cv::stereoRectifyUncalibrated(first_points, last_points, F,
			image_size, H1, H2);

	// Transform homogrophies to rectifications
	cv::solve(M, H1*M, R1, cv::DECOMP_LU);
	cv::solve(M, H2*M, R2, cv::DECOMP_LU);
	//Mat M_inv;
	//cv::invert(M, M_inv);
	//R1 = M_inv * H1 * M;
	//R2 = M_inv * H2 * M;

	// Rectify images
	Mat map1[2], map2[2];
	cv::initUndistortRectifyMap(M, D, R1, M,  image_size, CV_16SC2,
			map1[0], map1[1]);
	cv::initUndistortRectifyMap(M, D, R2, M,  image_size, CV_16SC2,
			map2[0], map2[1]);

	// Remap image
	Mat image1 = previous_images.front();
	Mat image2 = previous_images.back();
	//Mat image2 = previous_images[1];
	cv::remap(image1, display1, map1[0], map1[1], cv::INTER_LINEAR); 
	cv::remap(image2, display2, map2[0], map2[1], cv::INTER_LINEAR);	

	// Convert to BGR and draw horizontal lines
	cv::cvtColor(display1, display1, cv::COLOR_GRAY2BGR);
	cv::cvtColor(display2, display2, cv::COLOR_GRAY2BGR);
	drawHorizontalLines(display1);
	drawHorizontalLines(display2);
}

void Motion::findEssential()
{
	// Load intrinsic and distortion parameters
	D = (cv::Mat_<double>(5,1) << IMAGE_DISTORTION_KNOWN);
	M = (cv::Mat_<double>(3,3) << IMAGE_INTRINSIC_KNOWN);

	// Compute essential matrix
	Mat Mt;
	cv::transpose(M, Mt);
	E = Mt * F * M;

	// Perform SVD on essential matrix
	Mat W, U, V, Vt;
	cv::SVD::compute(E, W, U, Vt);
	cv::transpose(Vt, V);

	// Find rotation and translation using cv::recoverPose()
	vector<Point2f> first_points = tracked_corners.front();
	vector<Point2f> last_points = tracked_corners.back();
	cv::recoverPose(E, first_points, last_points, M, R, T);
}

Mat Motion::getIntrinsic() { return M; }

Mat Motion::getHomography1() { return H1; }

Mat Motion::getHomography2() { return H2; }

Mat Motion::getRectification1() { return R1; }

Mat Motion::getRectification2() { return R2; }

Mat Motion::getFundamental() { return F; }

Mat Motion::getEssential() { return E; }

Mat Motion::getRotation() { return R; }

Mat Motion::getTranslation() { return T; }

void Motion::drawHorizontalLines(Mat& image)
{
	Point point1, point2;
	Scalar color(0, 0, 255);

	// For each line spacing draw line
	for (int i = 1; i < IMAGE_HEIGHT / LINE_SPACING; i++) {
		point1 = Point(0, i*LINE_SPACING);
		point2 = Point(IMAGE_WIDTH-1, i*LINE_SPACING);
		cv::line(image, point1, point2, color);
	}
}
