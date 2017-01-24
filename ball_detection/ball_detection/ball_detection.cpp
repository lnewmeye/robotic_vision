// EcEn 631 - Robotic Vision
// Assignment 1 - Tennis Ball Detection
// Luke Newmeyer


// Standard includes
#include <iostream>
#include <string>
#include <vector>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


// Standard namespaces
using std::cout;
using std::endl;
using std::string;
using std::vector;

// OpenCV namespaces
using cv::Mat;
using cv::Point;

// Function declarations
void circleFit(Mat input, Mat display);
string generateFilename(string folder, string prefix, int number, string type);
string type2str(int type);

// Display parameters
#define DISPLAY_PERIOD 30 // in ms (-1 for infinite)

// Image set parameters
#define IMAGE_FOLDER "image_sequence\\"
#define IMAGE_TYPE ".jpg"
#define LEFT_PREFIX "1L"
#define RIGHT_PREFIX "1R"
#define INITIAL_IMAGE 5

// Box parameters
#define BOX_WIDTH 100
#define BOX_HEIGHT 150
#define IMAGE_WIDTH 640 //pixels
#define IMAGE_HEIGHT 480 //pixels
#define INITIAL_BOX cv::Range(50, 200), cv::Range(300, 400)

// Threshold parameters
#define THRESHOLD_LEVEL 6

// Begin main
int main()
{
	// Display application usage
	cout << "Use key presses from the following menu:" << endl;
	cout << "\tq: quit program" << endl;
	cout << "\tany key to continue..." << endl;
	cout << endl;
	
	// Create display window
	const string WINDOW_NAME = "Tennis Ball Finder";
	cv::namedWindow(WINDOW_NAME, CV_WINDOW_AUTOSIZE);
	cv::setWindowTitle(WINDOW_NAME, WINDOW_NAME);
	const string TEST_WINDOW = "Testing Window";
	cv::namedWindow(TEST_WINDOW, CV_WINDOW_AUTOSIZE);
	cv::setWindowTitle(WINDOW_NAME, WINDOW_NAME);

	// Open background images
	const string LEFT_BACKGROUND = generateFilename(IMAGE_FOLDER, LEFT_PREFIX, INITIAL_IMAGE, IMAGE_TYPE);
	const string RIGHT_BACKGROUND = generateFilename(IMAGE_FOLDER, RIGHT_PREFIX, INITIAL_IMAGE, IMAGE_TYPE);
	Mat left_background = cv::imread(LEFT_BACKGROUND, cv::IMREAD_GRAYSCALE);
	Mat right_background = cv::imread(RIGHT_BACKGROUND, cv::IMREAD_GRAYSCALE);

	// Create datastructures for data images
	Mat left_image = left_background;
	Mat right_image = right_background;
	Mat left_background_box, right_background_box;
	Mat left_previous, left_box, left_difference, left_threshold, left_erode, left_dilate, left_connected;
	Mat right_previous, right_box, right_difference, right_threshold, right_erode, right_dilate, right_connected;

	int image_count = INITIAL_IMAGE + 1;
	string left_name, right_name;
	char keyvalue = 0;

	// Exit when no additional images available or 'q' is pressed
	while (left_image.data && right_image.data && keyvalue != 'q') {
		
		// Box region of interest
		left_box = left_image(INITIAL_BOX);
		left_background_box = left_background(INITIAL_BOX);
		
		// Take difference from background
		//cv::absdiff(left_box, left_background_box, left_difference);
		cv::absdiff(left_image, left_background, left_difference);
		cv::absdiff(right_image, right_background, right_difference);

		// Threshold image
		cv::threshold(left_difference, left_threshold, THRESHOLD_LEVEL, 256, 0);
		cv::threshold(right_difference, right_threshold, THRESHOLD_LEVEL, 256, 0);

		// Erode edges
		//cv::erode(left_threshold, left_erode, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
		cv::erode(left_threshold, left_erode, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7)));
		cv::erode(right_threshold, right_erode, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7)));

		// Dilate edges
		cv::dilate(left_erode, left_dilate, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
		cv::dilate(right_erode, right_dilate, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

		// Identify connected regions
		//cv::connectedComponents(left_erode, left_connected, 8, CV_32S, cv::CCL_DEFAULT);

		// Find best fit circle
		circleFit(left_dilate, left_image);
		circleFit(right_dilate, right_image);

		// Display image
		cv::imshow(TEST_WINDOW, left_image);
		cv::imshow(WINDOW_NAME, right_image);
		
		// Wait for keypress (or period to elapse)
		keyvalue = cv::waitKey(DISPLAY_PERIOD);

		// Load next images from sequence and increment image count
		left_name = generateFilename(IMAGE_FOLDER, LEFT_PREFIX, image_count, IMAGE_TYPE);
		right_name = generateFilename(IMAGE_FOLDER, RIGHT_PREFIX, image_count, IMAGE_TYPE);
		left_image = cv::imread(left_name, cv::IMREAD_GRAYSCALE);
		right_image = cv::imread(right_name, cv::IMREAD_GRAYSCALE);
		image_count++;
	}

	return 0;
}

void circleFit(Mat input, Mat display)
{
	vector<cv::Vec3f> circles;
	
	cv::HoughCircles(input, circles, cv::HOUGH_GRADIENT, 2, input.rows / 4, 30, 10, 2, 40);
	//left_erode.rows / 8, 200, 100, 1, 100);

	//cv::cvtColor(display, display, CV_GRAY2BGR);

	// Draw circles
	cout << "Number of circles: " << circles.size() << endl;
	for (size_t i = 0; i < circles.size(); i++)
	{
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		circle(display, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
		circle(display, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
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

string type2str(int type) {
	string r;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch (depth) {
	case CV_8U:  r = "8U"; break;
	case CV_8S:  r = "8S"; break;
	case CV_16U: r = "16U"; break;
	case CV_16S: r = "16S"; break;
	case CV_32S: r = "32S"; break;
	case CV_32F: r = "32F"; break;
	case CV_64F: r = "64F"; break;
	default:     r = "User"; break;
	}

	r += "C";
	r += (chans + '0');

	return r;
}