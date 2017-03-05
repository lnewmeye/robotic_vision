// EcEn 631 - Robotic Vision
// Assignment 3 - Stero Calibration
// Luke Newmeyer

#include <iostream>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// Class includes
#include "stereo.h"

// Default namespaces
using std::cout;
using std::endl;
using std::string;
using cv::Mat;
using cv::Size;
using cv::Scalar;
using cv::Point;
using cv::Range;

// Image input parameters
#define IMAGE_FOLDER "images/"
#define IMAGE_LEFT_PREFIX "CalibrateL"
#define IMAGE_RIGHT_PREFIX "CalibrateR" 
#define IMAGE_TYPE ".bmp"

// Display window parameters
#define WINDOW_LEFT_CAMERA "Left Camera"
#define WINDOW_RIGHT_CAMERA "Right Camera"

// Chessboard parameters
#define CHESSBOARD_ROWS 10
#define CHESSBOARD_COLUMNS 7


// Function declarations
string generateFilename(string folder, string prefix, int number, string type);
void drawCorners(Mat& image, vector<Point2f> corners);
void drawEpilines(Mat& image, Mat epilines);
void drawGrid(Mat image, Mat& display);

int main()
{
	// Create display windows
	cv::namedWindow(WINDOW_LEFT_CAMERA, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(WINDOW_RIGHT_CAMERA, CV_WINDOW_AUTOSIZE);
	
	// Create image data structures and load initial images
	int image_number = 0;
	string image_name = generateFilename(IMAGE_FOLDER, IMAGE_LEFT_PREFIX,
		image_number, IMAGE_TYPE);
	Mat right_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
	image_name = generateFilename(IMAGE_FOLDER, IMAGE_RIGHT_PREFIX,
		image_number, IMAGE_TYPE);
	Mat left_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
	image_number++;
	Mat left_display, right_display;
	
	// Find image sizes and create calibration objects
	Size image_size = left_image.size();
	Size pattern_size = Size(CHESSBOARD_ROWS, CHESSBOARD_COLUMNS);
	Stereo stereo(image_size, pattern_size);
	
	// Add first images to calibration sequence
	stereo.left_camera.addCalibrationImage(left_image, left_display);
	stereo.right_camera.addCalibrationImage(right_image, right_display);
	
	// Display first images
	cv::imshow(WINDOW_LEFT_CAMERA, left_display);
	cv::imshow(WINDOW_RIGHT_CAMERA, right_display);

	// Push images to calibration
	while (image_number < 100) {
		
		// Read in next image
		image_name = generateFilename(IMAGE_FOLDER, IMAGE_LEFT_PREFIX,
			image_number, IMAGE_TYPE);
		left_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
		image_name = generateFilename(IMAGE_FOLDER, IMAGE_RIGHT_PREFIX,
			image_number, IMAGE_TYPE);
		right_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
		image_number++;

		// Continue loop if no data in image
		if(!right_image.data || !left_image.data) {
			cout << "Skipped image " << image_name << endl;
			continue;
		}
		
		// Add images to calibration sequence
		stereo.left_camera.addCalibrationImage(left_image, left_display);
		stereo.right_camera.addCalibrationImage(right_image, right_display);
		
		// Display image
		cv::imshow(WINDOW_LEFT_CAMERA, left_display);
		cv::imshow(WINDOW_RIGHT_CAMERA, right_display);
		
		// Wait for keypress (display image)
		cv::waitKey(1);
	}
	
	// Run calibration
	stereo.left_camera.runCalibration();
	stereo.right_camera.runCalibration();
	
	// Print camera calibration parameters
	int rms = stereo.left_camera.getRMS();
	cout << "left_rms = " << rms << endl;
	rms = stereo.right_camera.getRMS();
	cout << "right_rms = " << rms << endl;
	Mat param = stereo.left_camera.getIntrinsic();
	cout << "left_intrinsic =" << endl << param << endl;
	param = stereo.left_camera.getDistortion();
	cout << "left_distortion =" << endl << param << endl;
	param = stereo.right_camera.getIntrinsic();
	cout << "right_intrinsic =" << endl << param << endl;
	param = stereo.right_camera.getDistortion();
	cout << "right_distortion =" << endl << param << endl;

	// Calibrate stereo system
	stereo.runCalibration();

	// Print stereo calibration parameters
	param = stereo.getRotation();
	cout << "rotation =" << endl << param << endl;
	param = stereo.getTranslation();
	cout << "translation =" << endl << param << endl;
	param = stereo.getEssential();
	cout << "essential =" << endl << param << endl;
	param = stereo.getFundamental();
	cout << "fundamental =" << endl << param << endl;

	// Open images R00 and L00, undistort, and identify 3 points of interest
	image_number = 0;
	image_name = generateFilename(IMAGE_FOLDER, IMAGE_LEFT_PREFIX,
		image_number, IMAGE_TYPE);
	left_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
	image_name = generateFilename(IMAGE_FOLDER, IMAGE_RIGHT_PREFIX,
		image_number, IMAGE_TYPE);
	right_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
	Mat left_undistort, right_undistort;
	stereo.left_camera.undistortImage(left_image, left_undistort);
	stereo.right_camera.undistortImage(right_image, right_undistort);
	vector<Point2f> left_corners;
	vector<Point2f> right_corners;
	cv::goodFeaturesToTrack(left_image, left_corners, 3, 0.01, 170);
	cv::goodFeaturesToTrack(right_image, right_corners, 3, 0.01, 110);
	cv::cvtColor(left_undistort, left_display, CV_GRAY2RGB);
	cv::cvtColor(right_undistort, right_display, CV_GRAY2RGB);
	drawCorners(left_display, left_corners);
	drawCorners(right_display, right_corners);

	// Compute epipolar lines for images and plot
	Mat fundamental = stereo.getFundamental();
	Mat left_lines, right_lines;
	cv::computeCorrespondEpilines(left_corners, 1, fundamental, left_lines);
	cv::computeCorrespondEpilines(right_corners, 2, fundamental, right_lines);
	cout << "left_lines =" << endl << left_lines << endl;
	cout << "right_lines =" << endl << right_lines << endl;
	drawEpilines(right_display, left_lines);
	drawEpilines(left_display, right_lines);

	// Display images and wait till keypress
	cv::imshow(WINDOW_LEFT_CAMERA, left_display);
	cv::imshow(WINDOW_RIGHT_CAMERA, right_display);
	cv::waitKey(0);

	// Write images to file
	cv::imwrite("output/left_epipolar.jpg", left_display);
	cv::imwrite("output/right_epipolar.jpg", right_display);

	// Rectify images
	Mat left_rectify, right_rectify;
	stereo.rectifyImage(left_image, right_image, left_rectify, right_rectify);
	drawGrid(left_rectify, left_display);
	drawGrid(right_rectify, right_display);

	// Display rectified images
	cv::imshow(WINDOW_LEFT_CAMERA, left_display);
	cv::imshow(WINDOW_RIGHT_CAMERA, right_display);
	cv::waitKey(0);

	// Write images to file
	cv::imwrite("output/left_rectify.jpg", left_display);
	cv::imwrite("output/right_rectify.jpg", right_display);

	// Find absolute difference of images
	cv::absdiff(left_image, left_rectify, left_display);
	cv::absdiff(right_image, right_rectify, right_display);
	cv::imshow(WINDOW_LEFT_CAMERA, left_display);
	cv::imshow(WINDOW_RIGHT_CAMERA, right_display);
	cv::waitKey(0);
	
	// Write images to file
	cv::imwrite("output/left_absdiff.jpg", left_display);
	cv::imwrite("output/right_absdiff.jpg", right_display);

	return 0;
}

void drawCorners(Mat& image, vector<Point2f> corners)
{
	// Loop through and draw circles on corners of output
	for(Point2f corner : corners) {
		cv::circle(image, corner, 4, Scalar(255,0,0), -1, 8, 0);
	}
}

void drawEpilines(Mat& image, Mat epilines)
{
	for(int i = 0; i < epilines.rows; i++) {
		
		// Define values for line
		float x = image.cols;
		float a = epilines.at<float>(i,0);
		float b = epilines.at<float>(i,1);
		float c = epilines.at<float>(i,2);

		// Compute points for line
		Point p1 = Point(0, (int)(-c/b));
		Point p2 = Point(x, (int)(-(c+a*x)/b));

		// Draw line on image
		cv::line(image, p1, p2, Scalar(0,0,255), 1, CV_AA);
	}
}

void drawGrid(Mat image, Mat& display)
{
	// Cnvert to color image
	cv::cvtColor(image, display, CV_GRAY2RGB);
	
	// Create data structures for loop
	int height = image.rows;
	int width = image.cols;
	int y;
	Point p1, p2;
	
	// Draw lines at every 1/10 of the image
	for(int i = 1; i < 10; i++) {
		y = (height * i) / 10;
		p1 = Point(0, y);
		p2 = Point(width, y);
		cv::line(display, p1, p2, Scalar(0, 0, 255), 1, CV_AA);
	}
}

string generateFilename(string folder, string prefix, int number, string type)
{
        string value;

        // Adjust string value to have leading 0's
        if (number < 10) {
                value = "0" + std::to_string(number);
        }
        else {
                value = std::to_string(number);
        }

        // Return file path
        return folder + prefix + value + type;
}

