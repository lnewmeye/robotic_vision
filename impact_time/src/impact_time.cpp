// EcEn 672 - Robotic Vision
// Assigment 7 - Main file
// Luke Newmeyer

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "ImpactDetector.hpp"

// Image sequence parameters
#define IMAGE_SEQUENCE "data/T%01d.jpg"

// Display window parameters
#define DISPLAY_WINDOW "Display Window"
#define DISPLAY_TIME_FAST 0

// Standard namespaces used
using std::cout;
using std::endl;

// OpenCV namespaces used
using cv::VideoCapture;
using cv::Mat;

int main()
{
	// Create display window
	cv::namedWindow(DISPLAY_WINDOW, CV_WINDOW_AUTOSIZE);

	// Open image sequence
	VideoCapture image_sequence;
	image_sequence.open(IMAGE_SEQUENCE);

	// Declare variables for main loop
	Mat image, display;
	char keypress;
	ImpactDetector impact_detector;

	// Read in first image and set as initial image
	image_sequence >> image;
	cv::cvtColor(image, image, CV_BGR2GRAY);
	impact_detector.setInitial(image);
	image_sequence >> image;

	// Read in image data, convert color, process, and display
	while(!image.empty() && keypress != 'q') {

		// Convert image color to gray scale and process
		cv::cvtColor(image, image, CV_BGR2GRAY);
		impact_detector.detectImpact(image, display);

		// Display image
		cv::imshow(DISPLAY_WINDOW, display);
		keypress = cv::waitKey(DISPLAY_TIME_FAST);

		// Load next image
		image_sequence >> image;
	}

	return 0;
}
