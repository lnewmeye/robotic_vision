#include <iostream>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "calibration.h"

using std::cout;
using std::endl;
using std::string;
using std::vector;

using cv::Mat;
using cv::Size;

// Aplication parameters
//#define USE_DEFAULT_PARAMS // Comment out to pull parameters from data source
//#define PRINT_PARAMS // Comment out to not print parameters
#define DATA_SOURCE 0 // Options: images=0, camera=1

// Display parameters
#define WINDOW_NAME "Display Window" // Name of display window
#define DISPLAY_TIME 1 // ms of time to display image

// Image input parameters
#define IMAGE_FOLDER "images/"
#define IMAGE_PREFIX "AR"
#define IMAGE_TYPE ".jpg"
#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480
#define CHESSBOARD_ROWS 10
#define CHESSBOARD_COLUMNS 7

// Ouptut image parameters (comment out when not needed)
//#define OUTPUT_IMAGE "output/task1_corner_detection.jpg"

// Undistortion image parameters
#define UNDISTORT_SEQUENCE {"Close", "Far", "Turn"}
#define UNDISTORT_EXTENSION ".jpg"
#define UNDISTORT_FOLDER "images/"
#define UNDISTORT_QUANTITY 3
const string UNDISTORT_IMAGES[UNDISTORT_QUANTITY] = UNDISTORT_SEQUENCE;

// Default calibration parameters
#define DEFAULT_INTRINSIC 1145.223455665104, 0, 328.945216696177, 0, \
	1143.600007277263, 222.1597641571362, 0, 0, 1
#define DEFAULT_DISTORTION -0.2575507934862325, 0.04884197280891184, \
	-0.001409751152446753, -0.001543707631160005, 0.9076529637760357
const Mat INTRINSIC_PARAMS = (cv::Mat_<double>(3,3) << DEFAULT_INTRINSIC);
const Mat DISTORTION_PARAMS = (cv::Mat_<double>(5,1) << DEFAULT_DISTORTION);

// Function declarations
string generateFilename(string folder, string prefix, int number, string type);
void saveImage(Mat image, string image_name);

int main()
{
	// Create image variables and load first image
	int image_number = 1;
	string image_name = generateFilename(IMAGE_FOLDER, IMAGE_PREFIX,
		image_number, IMAGE_TYPE);
	image_number++;
	Mat image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
	
	// Create Calibration object
	Size image_size = Size(IMAGE_WIDTH, IMAGE_HEIGHT);
	Size pattern_size = Size(CHESSBOARD_ROWS, CHESSBOARD_COLUMNS);
	Calibration calibration(image_size, pattern_size);
	
	// Create variables for loop
	int keypress = 0;
	Mat difference, display;
	//vector<Point2f> corners;
	//vector<vector<Point2f>> calibration_points;
	
	// Create window
	cv::namedWindow(WINDOW_NAME, CV_WINDOW_AUTOSIZE);
	
#ifdef USE_DEFAULT_PARAMS
	calibration.setParams(INTRINSIC_PARAMS, DISTORTION_PARAMS);
#else
	// Loop through iamges while 'q' is not pressed
	while (image.data && keypress != 'q') {
		
		// Locate corners on image and add to vector
		//corners = locateCorners(image, display);
		calibration.addCalibrationImage(image, display);
		
		// Copy corners to data structure
		//calibration_points.push_back(corners);
		
		// Display image
		cv::imshow(WINDOW_NAME, display);
		
		// Write image (if selected)
#ifdef OUTPUT_IMAGE
		saveImage(display, OUTPUT_IMAGE);
#endif
		
		// Wait for keypress
		keypress = cv::waitKey(DISPLAY_TIME);
		
		// Load next image
		image_name = generateFilename(IMAGE_FOLDER, IMAGE_PREFIX,
			image_number, IMAGE_TYPE);
		image_number++;
		image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
	}
	
	// Calibrate camera
	//calibrateCamera(calibration_points);
	calibration.runCalibration();
#ifdef PRINT_PARAMS
	Mat param = calibration.getIntrinsic();
	cout << "Intrinsic = " << endl << param << endl;
	param = calibration.getDistortion();
	cout << "Distortion = " << endl << param << endl;
#endif //PRINT_PARAMS
	
#endif //USE_DEFAULT_PARAMS
	
	// Loop through input images and undistort
	for (int i = 0; i < UNDISTORT_QUANTITY; i++) {
		image_name = UNDISTORT_FOLDER + UNDISTORT_IMAGES[i] + UNDISTORT_EXTENSION;
		image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
		cout << "Read in image: " << image_name << endl;
		
		// Undistort Image
		calibration.undistortImage(image, display);
		cv::absdiff(image, display, difference);
		cv::imshow(WINDOW_NAME, difference);
		cv::waitKey(0);
	}
	
	return 0;
}

void saveImage(Mat image, string image_name)
{
	cv::imwrite(image_name, image);
}

string generateFilename(string folder, string prefix, int number, string type)
{
	// Create string of value
	string value = std::to_string(number);
	
	// Return file path
	return folder + prefix + value + type;
}
