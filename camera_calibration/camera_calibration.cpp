#include <iostream>
#include <fstream>
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
using std::ifstream;

using cv::Mat;
using cv::Size;

// Aplication parameters
#define USE_DEFAULT_PARAMS // Comment out to pull parameters from data source
#define PRINT_PARAMS // Comment out to not print parameters
#define FIND_POSE // Comment out to skip this step
#define DATA_SOURCE 0 // Options: images=0, camera=1
#define WINDOW_NAME "Display Window" // Name of display window
#define DISPLAY_TIME 800 // ms of time to display image

// Image input parameters
#define IMAGE_FOLDER "images/"
#define IMAGE_PREFIX "AR"
#define IMAGE_TYPE ".jpg"
#define CALIBRATION_QUANTITY 50

// Chessboard parameters
#if DATA_SOURCE == 0
	#define CHESSBOARD_ROWS 10
	#define CHESSBOARD_COLUMNS 7
#else //DATA_SOURCE == 1
	#define CHESSBOARD_ROWS 9
	#define CHESSBOARD_COLUMNS 7
#endif

// Ouptut image parameters (comment out when not needed)
//#define OUTPUT_IMAGE "output/distortion_correction.jpg"

// Data points parameters
#define POINTS_FILE_NAME "DataPoints.txt"
#define POINTS_QUANTITY 20

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
void readDataPoints(string file_name, vector<Point3f>& object_points, 
	vector<Point2f>& image_points);

int main()
{
	// Create image varialbles
	string image_name;
	int image_number = 1;
	Mat image;
	
	// Load first image
#if DATA_SOURCE == 0
	image_name = generateFilename(IMAGE_FOLDER, IMAGE_PREFIX, image_number, IMAGE_TYPE);
	image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
	image_number++;
#else //DATA_SOURCE == 1
	cv::VideoCapture camera(0);
    if (!camera.isOpened()) {
            cout << "Error: camera not open" << endl;
            return -1;
    }
	camera >> image;
	cv::cvtColor(image, image, CV_RGB2GRAY);
#endif
	
	// Create Calibration object
	Size image_size = image.size();
	Size pattern_size = Size(CHESSBOARD_ROWS, CHESSBOARD_COLUMNS);
	Calibration calibration(image_size, pattern_size);
	
	// Create variables for loop
	int keypress = 0;
	Mat difference, display;
	
	// Create window
	cv::namedWindow(WINDOW_NAME, CV_WINDOW_AUTOSIZE);
	
#ifdef USE_DEFAULT_PARAMS
	calibration.setParams(INTRINSIC_PARAMS, DISTORTION_PARAMS);
#else
	// Loop through iamges while 'q' is not pressed
	while (image.data && keypress != 'q' && image_number < CALIBRATION_QUANTITY) {
		
		// Locate corners on image and add to vector
		calibration.addCalibrationImage(image, display);
		
		// Display image
		cv::imshow(WINDOW_NAME, display);
		
		// Wait for keypress
		keypress = cv::waitKey(DISPLAY_TIME);
		
		// Load next image
#if DATA_SOURCE == 0
		image_name = generateFilename(IMAGE_FOLDER, IMAGE_PREFIX,
			image_number, IMAGE_TYPE);
		image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
#else //DATA_SOURCE == 1
		camera >> image;
		cv::cvtColor(image, image, CV_RGB2GRAY);
#endif //DATA_SOURCE
		image_number++;
	}
	
	// Calibrate camera
	//calibrateCamera(calibration_points);
	calibration.runCalibration();
#ifdef PRINT_PARAMS
	Mat param = calibration.getIntrinsic();
	cout << "Intrinsic = " << endl << param << endl;
	param = calibration.getDistortion();
	cout << "Distortion = " << endl << param << endl;
	int rms = calibration.getRMS();
	cout << "RMS = " << rms << endl;
#endif //PRINT_PARAMS
	
#endif //USE_DEFAULT_PARAMS
	
	// Loop through input images and undistort
#if DATA_SOURCE == 0
	for (int i = 0; i < UNDISTORT_QUANTITY; i++) {
		image_name = UNDISTORT_FOLDER + UNDISTORT_IMAGES[i] + UNDISTORT_EXTENSION;
		image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
		cout << "Read in image: " << image_name << endl;
		
		// Undistort Image
		calibration.undistortImage(image, display);
		cv::absdiff(image, display, difference);
		cv::imshow(WINDOW_NAME, difference);
		
		// Write image (if selected)
#ifdef OUTPUT_IMAGE
		saveImage(difference, OUTPUT_IMAGE);
#endif
		
		cv::waitKey(0);
	}
#else //DATA_SOURCE == 1
	while (image.data && keypress != 'q') {
		camera >> image;
		calibration.undistortImage(image, display);
		cv::absdiff(image, display, difference);
		cv::imshow(WINDOW_NAME, difference);
		keypress = cv::waitKey(1);
	}
#endif
	
	// Object Pose Estimation
#ifdef FIND_POSE
	vector<Point3f> object_points;
	vector<Point2f> image_points;
	readDataPoints(POINTS_FILE_NAME, object_points, image_points);
	Mat rotation, translation, rotation_matrix;
	calibration.estimatePose(object_points, image_points, rotation, translation);
	cv::Rodrigues(rotation, rotation_matrix);
	cout << "rotation = " << endl << rotation_matrix << endl;
	cout << "translation = " << endl << translation << endl;
#endif
	
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

void readDataPoints(string file_name, vector<Point3f>& object_points, 
	vector<Point2f>& image_points)
{
	// Create variables to read impage points into
	double x, y;
	Point2f image_point;
	
	// Open file to read from
	ifstream infile(file_name);
	
	// Read through image points and push to vector
	for(int i = 0; i < POINTS_QUANTITY; i++) {
		infile >> x >> y;
		image_point.x = x;
		image_point.y = y;
		image_points.push_back(image_point);
	}
	
	// Create variables to read object points into
	double xo, yo, zo;
	Point3f object_point;
	
	// Read through object points and push to vector
	for(int i = 0; i < POINTS_QUANTITY; i++) {
		infile >> xo >> yo >> zo;
		object_point.x = xo;
		object_point.y = yo;
		object_point.z = zo;
		object_points.push_back(object_point);
	}
}

//void translate
