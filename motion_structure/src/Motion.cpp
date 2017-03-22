// EcEn 631 - Robotic Vision
// Structure from Motion - Motion Class
// Luke Newmeyer

#include "Motion.h"

// DEBUG define (comment out when not needed)
#define DEBUG

// Standard namespaces used
using std::vector;

// OpenCV namespaces used
using cv::Mat;
using cv::Size;
using cv::Rect;
using cv::Point;
using cv::Scalar;
using cv::Point2f;

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
	cv::findFundamentalMat(corners, flow_corners, cv::FM_RANSAC,
			3, 0.99, mask);

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
	cv::cornerSubPix(image, matches, subpix_size, zero_zone, subpix_criteria);

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

