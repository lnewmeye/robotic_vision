// EcEn 631 - Robotic Vision
// Assignment 4 - 3D Reconstruction and Trajectory Estimation
// Luke Newmeyer

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "stereo.h"

using std::cout;
using std::endl;
using cv::Mat;
using cv::Size;
using cv::Point;
using cv::Scalar;
using cv::Point2f;

// Define calibration parameters
#define CALIBRATION_LEFT_INTRINSIC \
		1153.7918353122, 0.0000000000, 311.6495082713, \
		0.0000000000, 1152.7061368496,  247.7409370695, \
		0.0000000000, 0.0000000000,  1.0000000000
#define CALIBRATION_LEFT_DISTORTION \
		-0.2574338164, \
		 0.3395576609, \
		 0.0011179409, \
		-0.0002030712, \
		-0.5947353243
#define CALIBRATION_RIGHT_INTRINSIC \
		1149.6505965772, 0.0000000000, 326.3569432986, \
		0.0000000000, 1148.0218738819, 224.6062742604, \
		0.0000000000, 0.0000000000, 1.0000000000
#define CALIBRATION_RIGHT_DISTORTION \
		-0.2950621013, \
		 1.1296741454, \
		-0.0010482716, \
		-0.0014052463, \
		-9.9589684633
#define CALIBRATION_STEREO_ROTATION \
		0.9993321128, -0.0096723688, -0.0352388092, \
		0.0104125365,  0.9997277295,  0.0208817147, \
		0.0350272391, -0.0212346934,  0.9991607380
#define CALIBRATION_STEREO_TRANSLATION \
		-11.7644241577, \
		-0.7834686697, \
		-1.7548721424
#define CALIBRATION_STEREO_ESSENTIAL \
		-0.0091700742, 1.7710310594, -0.7461663948, \
		-1.3416247882, -0.2328401698, 11.8163903279, \
		0.6604479052, -11.7687990496, -0.2732698516
#define CALIBRATION_STEREO_FUNDAMENTAL \
		-0.0000004352, 0.0000841265, -0.0615916207, \
		-0.0000637634, -0.0000110764, 0.6702695146, \
		0.0504789932, -0.6677701976, 1.0000000000

// Defines for image data
#define IMAGE_RECONSTRUCTION_LEFT "data/reconstruct_3d_left.bmp"
#define IMAGE_RECONSTRUCTION_RIGHT "data/reconstruct_3d_right.bmp"
#define CHESSBOARD_ROWS 10
#define CHESSBOARD_COLUMNS 7

// Display window parameters
#define WINDOW_LEFT_CAMERA "Left Camera"
#define WINDOW_RIGHT_CAMERA "Right Camera"

int main()
{
	// Open display windows
	cv::namedWindow(WINDOW_LEFT_CAMERA, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(WINDOW_RIGHT_CAMERA, CV_WINDOW_AUTOSIZE);

	// Load image for 3D reconstruction
	Mat left_image = cv::imread(IMAGE_RECONSTRUCTION_LEFT, cv::IMREAD_GRAYSCALE);
	Mat right_image = cv::imread(IMAGE_RECONSTRUCTION_RIGHT, cv::IMREAD_GRAYSCALE);
	Size image_size = left_image.size();
	Size pattern_size = Size(CHESSBOARD_ROWS, CHESSBOARD_COLUMNS);
 
	// Setup matricies for calibration parameters
	Mat left_intrinsic = (cv::Mat_<double>(3,3) << CALIBRATION_LEFT_INTRINSIC);
	Mat left_distortion = (cv::Mat_<double>(5,1) << CALIBRATION_LEFT_DISTORTION);
	Mat right_intrinsic = (cv::Mat_<double>(3,3) << CALIBRATION_RIGHT_INTRINSIC);
	Mat right_distortion = (cv::Mat_<double>(5,1) << CALIBRATION_RIGHT_DISTORTION);
	Mat rotation = (cv::Mat_<double>(3,3) << CALIBRATION_STEREO_ROTATION);
	Mat translation = (cv::Mat_<double>(3,1) << CALIBRATION_STEREO_TRANSLATION);
	Mat essential = (cv::Mat_<double>(3,3) << CALIBRATION_STEREO_ESSENTIAL);
	Mat fundamental = (cv::Mat_<double>(3,3) << CALIBRATION_STEREO_FUNDAMENTAL);

	// Create calibration object and set calibration parameters
	Stereo stereo(image_size, pattern_size);
	stereo.setCalibration(left_intrinsic, left_distortion, right_intrinsic,
			right_distortion, rotation, translation, essential, fundamental);

	// See if image loads correctly
	cv::imshow(WINDOW_LEFT_CAMERA, left_image);
	cv::imshow(WINDOW_RIGHT_CAMERA, right_image);
	cv::waitKey(0);

	// Find corners and undistort outer points
	vector<Point2f> left_corners, right_corners;
	Mat left_display, right_display;
	cv::findChessboardCorners(left_image, pattern_size, left_corners);
	cv::findChessboardCorners(right_image, pattern_size, right_corners);
	cv::TermCriteria criteria = cv::TermCriteria( CV_TERMCRIT_EPS + 
			CV_TERMCRIT_ITER, 40, 0.001);
	Size search_size = Size(5,5);
	Size zero_zone = Size(-1,-1);
	cv::cornerSubPix(left_image, left_corners, search_size, zero_zone, criteria);
	cv::cornerSubPix(right_image, right_corners, search_size, zero_zone, criteria);

	return 0;
}


