// EcEn 631 - Robotic Vision
// Assigment 6 - Structure from Motion
// Luke Newmeyer

#include <iostream>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// Class includes
#include "Motion.h"

// Standard namespaces used by default
using std::cout;
using std::endl;
using std::vector;

// OpenCV namespaces used by default
using cv::VideoCapture;
using cv::Mat;

// DEBUG option (comment out when not in use)
//#define DEBUG
#ifdef DEBUG
#define DISPLAY_DEBUG "Debug Display"
#endif

// Application options
#define TASK 1 // Options: 1, 2, or 3

// Parameters for display window
#define DISPLAY_WINDOW_1 "Display Window 1"
#define DISPLAY_WINDOW_2 "Display Window 2"
#define DISPLAY_TIME_FAST 1
#define DISPLAY_TIME_SLOW 0

// Parameters for image input
#define IMAGE_TRACKING_1 "data/parallel_cube/ParallelCube%01d.jpg"
#define IMAGE_TRACKING_2 "data/parallel_real/ParallelReal%01d.jpg"
#define IMAGE_TRACKING_3 "data/turned_cube/TurnCube%01d.jpg"
#define IMAGE_TRACKING_4 "data/turned_real/TurnReal%01d.jpg"

// Parameters for image output
//#define IMAGE_OUTPUT_FOLDER "output/"
//#define IMAGE_OUTPUT_TYPE ".jpg"
//#define IMAGE_OUTPUT_FLOW1 "flow1_"
//#define IMAGE_OUTPUT_FLOW2 "flow2_"
//#define IMAGE_OUTPUT_FLOW3 "flow3_"
//#define IMAGE_OUTPUT_MATCH1 "match1_"
//#define IMAGE_OUTPUT_MATCH2 "match2_"
//#define IMAGE_OUTPUT_MATCH3 "match3_"

// Parameters for optical flow task
//#define OPTICAL_CORNER_MAX 500
//#define OPTICAL_CORNER_QUALITY 0.01
//#define OPTICAL_CORNER_DISTANCE 5
//#define OPTICAL_FLOW_LEVEL 3

int main()
{
	// Create windows and objects for project
	VideoCapture image_sequence;
	Mat image, display, display1, display2;
	int keypress;
	cv::namedWindow(DISPLAY_WINDOW_1, CV_WINDOW_AUTOSIZE);
	Motion motion;

	// Create debug window (if defined)
#ifdef DEBUG
	cv::namedWindow(DISPLAY_DEBUG, CV_WINDOW_NORMAL);
#endif

	// Open image sequence
	image_sequence.open(IMAGE_TRACKING_1);

	// Read in first image and set as inital in Motion object
	image_sequence >> image;
	cv::cvtColor(image, image, CV_BGR2GRAY);
	motion.setInitial(image);

	// Read in next image and begin loop
	image_sequence >> image;
	keypress = 0;

	// Loop while images available
	while(image.data && keypress != 'q') {

		// Convert image color
		cv::cvtColor(image, image, CV_BGR2GRAY);

		// Execute processing according to selected task
		#if TASK == 1
		motion.addFrame(image);
		display = image.clone();
		#elif TASK == 2
		display = image.clone();
		#elif TASK == 3
		display = image.clone();
		#endif

		// Display, wait, and load next image
		cv::imshow(DISPLAY_WINDOW_1, display);
		keypress = cv::waitKey(DISPLAY_TIME_FAST);
		image_sequence >> image;
	}

	// Get images with motion displayed and display
	vector<Mat> display_images = motion.getMotionImages();
	for (Mat display_image : display_images) {
		cv::imshow(DISPLAY_WINDOW_1, display_image);
		keypress = cv::waitKey(DISPLAY_TIME_SLOW);
		if (keypress == 'q')
			break;
	}

	// Output averaged intrinsic parameters
	Mat F = motion.averageIntrinsic();
	cout << "Averaged intrinsic parameters:" << endl;
	cout << "F =" << endl << F << endl;

	// Rectify image based on guess of parameters (task 1)
	motion.rectifyImage(display1, display2);
	cv::imshow (DISPLAY_WINDOW_1, display1);
	cv::imshow (DISPLAY_WINDOW_2, display2);
	cv::waitKey(DISPLAY_TIME_SLOW);

	return 0;
}

