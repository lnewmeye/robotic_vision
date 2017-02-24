// EcEn 631 - Robotic Vision
// Assignment 3 - Stero Calibration
// Luke Newmeyer

#include <iostream>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// Class includes
#include "calibration.h"

// Default namespaces
using std::cout;
using std::endl;
using std::string;
using cv::Mat;
using cv::Size;

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
	
	Size left_size = left_image.size();
	Size right_size = right_image.size();
	Size pattern_size = Size(CHESSBOARD_ROWS, CHESSBOARD_COLUMNS);
	Calibration left_calibration(left_size, pattern_size);
	Calibration right_calibration(right_size, pattern_size);
	
	
	while (right_image.data && left_image.data) {
		
		// Add inages to calibration sequence
		left_calibration.addCalibrationImage(left_image, left_display);
		right_calibration.addCalibrationImage(right_image, right_display);
		
		// Display image
		cv::imshow(WINDOW_LEFT_CAMERA, left_display);
		cv::imshow(WINDOW_RIGHT_CAMERA, right_display);
		
		// Read in next image
		image_name = generateFilename(IMAGE_FOLDER, IMAGE_LEFT_PREFIX,
			image_number, IMAGE_TYPE);
		left_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
		image_name = generateFilename(IMAGE_FOLDER, IMAGE_RIGHT_PREFIX,
			image_number, IMAGE_TYPE);
		right_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
		image_number++;
		
		// Wait for keypress (display image)
		cv::waitKey(1);
	}
	
	return 0;
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
