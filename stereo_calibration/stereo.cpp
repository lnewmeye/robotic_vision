// EcEn 631 - Robotic Vision
// Stereo Calibration & Rectification - Stereo Class
// Luke Newmeyer

#include "stereo.h"

// For debugging
#include <iostream>

using cv::Size;
using cv::Rect;

Stereo::Stereo(Size image_size, Size pattern_size)
{
	// Set object variables
	this->image_size = image_size;
	this->pattern_size = pattern_size;
	
	// Pass variables to calibration objects
	left_camera.setSize(image_size, pattern_size);
	right_camera.setSize(image_size, pattern_size);
}

void Stereo::addCalibrationImage(Mat left_image, Mat right_image,
		Mat& left_display, Mat& right_display)
{
	// Create color version of input image
	cv::cvtColor(left_image, left_display, CV_GRAY2RGB);
	cv::cvtColor(right_image, right_display, CV_GRAY2RGB);
	
	// Create variables
	vector<Point2f> left_corners, right_corners;
	Size search_size = Size(5,5);
	Size zero_zone = Size(-1,-1);
	cv::TermCriteria criteria = cv::TermCriteria( CV_TERMCRIT_EPS + 
		CV_TERMCRIT_ITER, 40, 0.001);
	bool left_found, right_found;
	
	// Find chessboard corners, refine corners, and draw
	left_found = cv::findChessboardCorners(left_image, pattern_size, 
			left_corners);
	right_found = cv::findChessboardCorners(right_image, pattern_size, 
			right_corners);
	
	// Draw if pattern found
	if(left_found && right_found) {

		// Refine coordinates in images
		cv::cornerSubPix(left_image, left_corners, search_size, 
				zero_zone, criteria);
		cv::cornerSubPix(right_image, right_corners, search_size, 
				zero_zone, criteria);

		// Draw coordinates on display image
		cv::drawChessboardCorners(left_display, pattern_size, 
				left_corners, left_found);
		cv::drawChessboardCorners(right_display, pattern_size, 
				right_corners, right_found);

		// Push corners to data structure
		left_points.push_back(left_corners);
		right_points.push_back(right_corners);
	}

	//TODO: Remvoe this branch. For debug only
	else
		std::cout << "Pattern not found" << std::endl;
}

void Stereo::runCalibration()
{
	// Get points from individual calibrations
	vector<vector<Point3f>> object_points;
	Mat left_intrinsic = left_camera.getIntrinsic();
	Mat left_distortion = left_camera.getDistortion();
	Mat right_intrinsic = right_camera.getIntrinsic();
	Mat right_distortion = right_camera.getDistortion();
	
	// Create vector of vector of point indicies
	for (unsigned i = 0; i < left_points.size(); i++) {
		
		// Create vector of points
		vector<Point3f> index;
		
		// Loop through rows and columns and add points
		for (int j = 0; j < pattern_size.height; j++) {
			for (int k = 0; k < pattern_size.width; k++) {
				Point3f point;
				point.x = SQUARE_SIZE * k;
				point.y = SQUARE_SIZE * j;
				point.z = 0;
				index.push_back(point);
			}
		}
		
		// Push new vector to set of indicies
		object_points.push_back(index);
	}


	// Print number of left and right points
	std::cout << "Left points = " << left_points.size() << std::endl;
	std::cout << "Right points = " << right_points.size() << std::endl;
	// Run calibration
	cv::stereoCalibrate(object_points, left_points, right_points,
			left_intrinsic, left_distortion, right_intrinsic,
			right_distortion, image_size, rotation, translation,
			essential, fundamental);
}

void Stereo::rectifyImage(Mat input_left, Mat input_right, Mat& output_left, Mat& output_right)
{
	// Get intrinsic and distortion parameters
	Mat left_intrinsic = left_camera.getIntrinsic();
	Mat left_distortion = left_camera.getDistortion();
	Mat right_intrinsic = right_camera.getIntrinsic();
	Mat right_distortion = right_camera.getDistortion();
	
	// Create data structures for outputs
	Mat left_rectification, right_rectification;
	Mat left_projection, right_projection;
	Rect valid_roi_left, valid_roi_right;
	Mat left_map[2], right_map[2];
	Mat disparity;

	// Produce rectification
	cv::stereoRectify(left_intrinsic, left_distortion, right_intrinsic,
			right_distortion, image_size, rotation, translation,
			left_rectification, right_rectification, left_projection,
			right_projection, disparity, 0, -1,
			image_size, &valid_roi_left, &valid_roi_right);
	
	// Precompute maps for remap()
	cv::initUndistortRectifyMap(left_intrinsic, left_distortion,
			left_rectification, left_projection,  image_size, CV_16SC2,
			left_map[0], left_map[1]);
	cv::initUndistortRectifyMap(right_intrinsic, right_distortion,
			right_rectification, right_projection,  image_size, CV_16SC2,
			right_map[0], right_map[1]);
	
	// Remap image
	cv::remap(input_left, output_left, left_map[0], left_map[1], cv::INTER_LINEAR); 
	cv::remap(input_right, output_right, right_map[0], right_map[1], cv::INTER_LINEAR);
}

