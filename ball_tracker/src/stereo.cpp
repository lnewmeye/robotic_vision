// EcEn 631 - Robotic Vision
// Stereo Calibration & Rectification - Stereo Class
// Luke Newmeyer

#include "stereo.h"

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

void Stereo::runCalibration()
{
	// Get points from individual calibrations
	vector<vector<Point3f>> object_points = left_camera.getObjectPoints();
	vector<vector<Point2f>> left_points = left_camera.getCalibrationPoints();
	vector<vector<Point2f>> right_points = right_camera.getCalibrationPoints();
	Mat left_intrinsic = left_camera.getIntrinsic();
	Mat left_distortion = left_camera.getDistortion();
	Mat right_intrinsic = right_camera.getIntrinsic();
	Mat right_distortion = right_camera.getDistortion();

	// Run calibration
	cv::stereoCalibrate(object_points, left_points, right_points,
			left_intrinsic, left_distortion, right_intrinsic,
			right_distortion, image_size, rotation, translation,
			essential, fundamental);

	// Compute rectification
	computeRectification();
}

void Stereo::rectifyImage(Mat input_left, Mat input_right, Mat& output_left, Mat& output_right)
{
	// Setup data structures
	Mat left_map[2], right_map[2];

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

vector<Point2f> Stereo::rectifyPointLeft(vector<Point2f> points)
{
	vector<Point2f> undistorted;

	cv::undistortPoints(points, undistorted, left_intrinsic, left_distortion,
			left_rectification, left_projection);
	return undistorted;
}

vector<Point2f> Stereo::rectifyPointRight(vector<Point2f> points)
{
	vector<Point2f> undistorted;

	cv::undistortPoints(points, undistorted, right_intrinsic, right_distortion,
			right_rectification, right_projection);
	return undistorted;
}

void Stereo::setCalibration(Mat left_intrinsic, Mat left_distortion, Mat right_intrinsic, Mat right_distortion, Mat rotation, Mat translation, Mat essential, Mat fundamental)
{
	// Set parameters for left and right cameras
	left_camera.setCalibration(left_intrinsic, left_distortion);
	right_camera.setCalibration(right_intrinsic, right_distortion);

	// Set local copy of left and right camera parameters
	this->left_intrinsic = left_intrinsic;
	this->left_distortion = left_distortion;
	this->right_intrinsic = right_intrinsic;
	this->right_distortion = right_distortion;

	// Set parameters for stereo calibration
	this->rotation = rotation;
	this->translation = translation;
	this->essential = essential;
	this->fundamental = fundamental;
}

void Stereo::computeRectification()
{
	// Get intrinsic and distortion parameters
	left_intrinsic = left_camera.getIntrinsic();
	left_distortion = left_camera.getDistortion();
	right_intrinsic = right_camera.getIntrinsic();
	right_distortion = right_camera.getDistortion();

	// Create data structures for outputs
	Rect valid_roi_left, valid_roi_right;

	// Produce rectification
	cv::stereoRectify(left_intrinsic, left_distortion, right_intrinsic,
			right_distortion, image_size, rotation, translation,
			left_rectification, right_rectification, left_projection,
			right_projection, disparity, 0, -1,
			image_size, &valid_roi_left, &valid_roi_right);
}
