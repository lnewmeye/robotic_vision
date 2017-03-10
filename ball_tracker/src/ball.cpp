// EcEn 631 - Robotic Vision
// Ball tracking object
// Luke Newmeyer

#include "ball.h"

// DEBUG for displaying images
#define DEBUG

// Debug options (if DEBUG defined)
#ifdef DEBUG
#include <iostream>
#include <opencv2/highgui.hpp>
#define DEBUG_WINDOW_LEFT "Left Debug"
#define DEBUG_WINDOW_RIGHT "Right Debug"
//#define DEBUG_ABS_DIFF
//#define DEBUG_THRESHOLD
#define DEBUG_ERODE
#endif //DEBUG

using cv::Mat;
using cv::Rect;
using cv::Point;
using cv::Scalar;

Ball::Ball(Mat left_background, Mat right_background)
{
	// Save background images
	this->left_background = left_background.clone();
	this->right_background = right_background.clone();

	// Set image height and width
	image_height = left_background.rows;
	image_width = left_background.cols;

	// Create structure for erosion in Ball::detectBall
	erosion_structure = cv::getStructuringElement(cv::MORPH_ELLIPSE, 
			cv::Size(EROSION_SIZE, EROSION_SIZE));

	// Define search region
	setSearchRegion(INITIAL_LEFT_X, INITIAL_LEFT_Y,
			INITIAL_RIGHT_X, INITIAL_RIGHT_Y);

	// Open debug window (if DEBUG defined)
#ifdef DEBUG
	cv::namedWindow(DEBUG_WINDOW_LEFT, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(DEBUG_WINDOW_RIGHT, CV_WINDOW_AUTOSIZE);
#endif //DEBUG
}

void Ball::detectBall(Mat left_image, Mat right_image,
		Mat& left_display, Mat& right_display)
{
	// Select search region of image
	Mat left_region = left_image(left_search);
	Mat right_region = right_image(right_search);

	// Difference image from background
	Mat left_diff, right_diff;
	Mat left_background_region = left_background(left_search);
	Mat right_background_region = right_background(right_search);
	cv::absdiff(left_region, left_background_region, left_diff);
	cv::absdiff(right_region, right_background_region, right_diff);

	// Display absdiff image (if selected)
#ifdef DEBUG_ABS_DIFF
	cv::imshow(DEBUG_WINDOW_LEFT, left_diff);
	cv::imshow(DEBUG_WINDOW_RIGHT, right_diff);
#endif //DEBUG_ABS_DIFF

	// Threshold absdiff image
	Mat left_threshold, right_threshold;
	cv::threshold(left_diff, left_threshold, THRESHOLD_LEVEL, 255, 0);
	cv::threshold(right_diff, right_threshold, THRESHOLD_LEVEL, 255, 0);

	// Display threshold image (if selected)
#ifdef DEBUG_THRESHOLD
	cv::imshow(DEBUG_WINDOW_LEFT, left_threshold);
	cv::imshow(DEBUG_WINDOW_RIGHT, right_threshold);
#endif //DEBUG_THRESHOLD

	// Erode edges to remove noise
	Mat left_erode, right_erode;
	cv::erode(left_threshold, left_erode, erosion_structure);
	cv::erode(right_threshold, right_erode, erosion_structure);

	// Display erosion image (if selected)
#ifdef DEBUG_ERODE
	cv::imshow(DEBUG_WINDOW_LEFT, left_erode);
	cv::imshow(DEBUG_WINDOW_RIGHT, right_erode);
#endif //DEBUG_ERODE

	// Get statistics of image using connected components
	Mat left_label, left_stats, left_centroids;
	Mat right_label, right_stats, right_centroids;
	cv::connectedComponentsWithStats(left_erode, left_label, left_stats,
			left_centroids);
	cv::connectedComponentsWithStats(right_erode, right_label, right_stats,
			right_centroids);
	
	// Find ball in image statistics
	left_ball = chooseComponent(left_centroids, left_stats,
			left_search_center);
	right_ball = chooseComponent(right_centroids, right_stats,
			right_search_center);
	
	// Convert colors for left and right displays and draw search region
	cv::cvtColor(left_image, left_display, CV_GRAY2BGR);
	cv::cvtColor(right_image, right_display, CV_GRAY2BGR);
	drawSearchRegion(left_display, right_display);

	// Update positions and draw if ball found
	if (left_ball.found && right_ball.found) {

		// Draw ball and search region on output images
		drawBall(left_display, right_display);

		// Push ball position to vector and update search region
		setSearchRegion(left_ball.centroid_x, left_ball.centroid_y,
				right_ball.centroid_x, right_ball.centroid_y);
		left_balls.push_back(left_ball);
		right_balls.push_back(right_ball);
	}

}

void Ball::setSearchRegion(int left_x, int left_y, int right_x,
		int right_y)
{
	// Create variables
	int left, top, width, height;

	// Find left and top values
	left = left_x - SEARCH_BOX_WIDTH/2;
	top = left_y - SEARCH_BOX_HEIGHT/2;
	width = SEARCH_BOX_WIDTH;
	height = SEARCH_BOX_HEIGHT;

	// Keep values in range
	if (top < 0)
		top = 0;
	if (left < 0)
		left = 0;
	if (left + width > image_width)
		width = image_width - left;
	if (top + height > image_height)
		height = image_height - top;

	// Set left search region
	left_search = Rect(left, top, width, height);
	left_search_center = Point(left, top);

	// Find left and top values
	left = right_x - SEARCH_BOX_WIDTH/2;
	top = right_y - SEARCH_BOX_HEIGHT/2;
	width = SEARCH_BOX_WIDTH;
	height = SEARCH_BOX_HEIGHT;

	// Keep values in range
	if (top < 0)
		top = 0;
	if (left < 0)
		left = 0;
	if (left + width > image_width)
		width = image_width - left;
	if (top + height > image_height)
		height = image_height - top;

	// Set right search region
	right_search = Rect(left, top, width, height);
	right_search_center = Point(left, top);
}

Position Ball::chooseComponent(Mat centroids, Mat stats, Point search_center)
{
	// Crate variables
	Position position;
	int largest_area = 0;
	int best_index = -1;
	int area;

	// Find largest mass and set best_index
	for(int i = 1; i < stats.rows; i++) {

		// Find area of current stat
		area = stats.at<int>(i,cv::CC_STAT_AREA);

		// Set best_index if larges area found yet
		if (area > largest_area) {
			largest_area = area;
			best_index = i;
		}
	}

	// Set value of position if a best_index is found
	if (best_index > 0) {
		position.found = true;
		position.box_left = stats.at<int>(best_index,cv::CC_STAT_LEFT) +
				search_center.x;
		position.box_top = stats.at<int>(best_index,cv::CC_STAT_TOP) +
				search_center.y;
		position.box_width = stats.at<int>(best_index,cv::CC_STAT_WIDTH);
		position.box_height = stats.at<int>(best_index,cv::CC_STAT_HEIGHT);
		position.centroid_x = centroids.at<double>(best_index,0) +
				search_center.x;
		position.centroid_y = centroids.at<double>(best_index,1) +
				search_center.y;
	}
	else {
		position.found = false;
	}

	return position;
}

void Ball::drawSearchRegion(Mat& left_image, Mat& right_image)
{
	// Draw rectangle on left image and right images
	Scalar color = Scalar(0, 255, 0);
	cv::rectangle(left_image, left_search, color);
	Scalar color2 = Scalar(0, 255, 0);
	cv::rectangle(right_image, right_search, color2);
}

void Ball::drawBall(Mat& left_image, Mat& right_image)
{
	// Create variables for drawing
	Point center;
	Scalar color;

	// Draw ball for left image if found
	if (left_ball.found) {
		center = Point(left_ball.centroid_x, left_ball.centroid_y);
		color = Scalar(255, 0, 0);
		cv::circle(left_image, center, 3, color, -1, 8, 0);
	}

	// Draw ball for right image if found
	if (right_ball.found) {
		center = Point(right_ball.centroid_x, right_ball.centroid_y);
		color = Scalar(255, 0, 0);
		cv::circle(right_image, center, 3, color, -1, 8, 0);
	}
}
