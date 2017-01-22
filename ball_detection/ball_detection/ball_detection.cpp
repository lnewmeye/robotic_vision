// EcEn 631 - Robotic Vision
// Assignment 1 - Tennis Ball Detection
// Luke Newmeyer


// Standard includes
#include <iostream>
#include <string>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


// Standard namespaces
using std::cout;
using std::endl;
using std::string;

// OpenCV namespaces
using cv::Mat;


// Function declarations
string generateFilename(string folder, string prefix, int number, string type);


// Image set parameters
#define IMAGE_FOLDER "image_sequence\\"
#define IMAGE_TYPE ".jpg"
#define LEFT_PREFIX "1L"
#define RIGHT_PREFIX "1R"
#define INITIAL_IMAGE 5

// Box parameters
#define INITIAL_BOX cv::Range(50, 200), cv::Range(300, 400)


// Begin main
int main()
{
	// Display application usage
	cout << "Use key presses from the following menu:" << endl;
	cout << "\tq: quit program" << endl;
	
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
	Mat left_background = cv::imread(LEFT_BACKGROUND);
	Mat right_background = cv::imread(RIGHT_BACKGROUND);

	// Create datastructures for data images
	Mat left_image = left_background;
	Mat right_image = right_background;
	Mat left_background_box, right_background_box;
	Mat left_previous, left_box, left_difference, left_threshold, left_connected;
	Mat right_previous, right_box, right_difference, right_threshold, right_connected;

	int image_count = INITIAL_IMAGE + 1;
	string left_name, right_name;
	char keyvalue = 0;

	while (left_image.data && right_image.data && keyvalue != 'q') {
		
		left_box = left_image(INITIAL_BOX);
		left_background_box = left_background(INITIAL_BOX);
		cv::absdiff(left_box, left_background_box, left_difference);
		cv::threshold(left_difference, left_threshold, 10, 255, 0);
		//cv::connectedComponents(left_threshold, left_connected);

		cv::imshow(TEST_WINDOW, left_threshold);
		cv::imshow(WINDOW_NAME, left_image);
		keyvalue = cv::waitKey(0);

		// Load next images from sequence and increment image count
		left_name = generateFilename(IMAGE_FOLDER, LEFT_PREFIX, image_count, IMAGE_TYPE);
		right_name = generateFilename(IMAGE_FOLDER, RIGHT_PREFIX, image_count, IMAGE_TYPE);
		left_image = cv::imread(left_name);
		right_image = cv::imread(right_name);
		image_count++;
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