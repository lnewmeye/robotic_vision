// EcEn 631 - Robotic Vision
// Assigment 6 - Structure from Motion
// Luke Newmeyer

#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// Class includes
#include "Motion.h"

// Standard namespaces used by default
using std::cout;
using std::endl;
using std::queue;
using std::vector;
using std::string;

// OpenCV namespaces used by default
using cv::VideoCapture;
using cv::Mat;

// Parameters for display window
#define DISPLAY_WINDOW_1 "Display Window 1"
#define DISPLAY_WINDOW_2 "Display Window 2"
#define DISPLAY_TIME_FAST 1
#define DISPLAY_TIME_SLOW 1

// Parameters for image input
#define IMAGE_TRACKING_1 "data/parallel_cube/ParallelCube%01d.jpg"
#define IMAGE_TRACKING_2 "data/parallel_real/ParallelReal%01d.jpg"
#define IMAGE_TRACKING_3 "data/turned_cube/TurnCube%01d.jpg"
#define IMAGE_TRACKING_4 "data/turned_real/TurnReal%01d.jpg"

// Parameters for image output
#define OUTPUT_RECTIFY_FIRST_1 "output/parallel_cube_first.jpg"
#define OUTPUT_RECTIFY_LAST_1 "output/parallel_cube_last.jpg"
#define OUTPUT_RECTIFY_FIRST_2 "output/parallel_real_first.jpg"
#define OUTPUT_RECTIFY_LAST_2 "output/parallel_real_last.jpg"
#define OUTPUT_RECTIFY_FIRST_3 "output/turned_cube_first.jpg"
#define OUTPUT_RECTIFY_LAST_3 "output/turned_cube_last.jpg"
#define OUTPUT_RECTIFY_FIRST_4 "output/turned_real_first.jpg"
#define OUTPUT_RECTIFY_LAST_4 "output/turned_real_last.jpg"

int main()
{
	// Create windows and objects for project
	VideoCapture image_sequence;
	Mat image, display, display1, display2;
	int keypress;
	cv::namedWindow(DISPLAY_WINDOW_1, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(DISPLAY_WINDOW_2, CV_WINDOW_AUTOSIZE);
	Motion motion;

	// Load vector with image sequence strings
	vector<string> image_names;
	image_names.push_back(IMAGE_TRACKING_1);
	image_names.push_back(IMAGE_TRACKING_2);
	image_names.push_back(IMAGE_TRACKING_3);
	image_names.push_back(IMAGE_TRACKING_4);
	queue<string> output_first;
	output_first.push(OUTPUT_RECTIFY_FIRST_1);
	output_first.push(OUTPUT_RECTIFY_FIRST_2);
	output_first.push(OUTPUT_RECTIFY_FIRST_3);
	output_first.push(OUTPUT_RECTIFY_FIRST_4);
	queue<string> output_last;
	output_last.push(OUTPUT_RECTIFY_LAST_1);
	output_last.push(OUTPUT_RECTIFY_LAST_2);
	output_last.push(OUTPUT_RECTIFY_LAST_3);
	output_last.push(OUTPUT_RECTIFY_LAST_4);

	// Iterate through image names and operate on sequence
	for (string image_name : image_names) {

		// Open image sequence
		cout << "Processing sequence: " << image_name << endl;
		image_sequence.open(image_name);

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
			motion.addFrame(image);
			display = image;

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

		// Rectify image based on guess of parameters (Task 1)
		Mat M, H1, H2, R1, R2;
		motion.rectifyImage(display1, display2);
		cv::imshow (DISPLAY_WINDOW_1, display1);
		cv::imshow (DISPLAY_WINDOW_2, display2);
		cv::waitKey(DISPLAY_TIME_SLOW);
		cv::imwrite(output_first.front(), display1);
		cv::imwrite(output_last.front(), display2);
		output_first.pop();
		output_last.pop();
		M = motion.getIntrinsic();
		H1 = motion.getHomography1();
		H2 = motion.getHomography2();
		R1 = motion.getRectification1();
		R2 = motion.getRectification2();
		cout << "M =" << endl << M << endl;
		cout << "H1 =" << endl << H1 << endl;
		cout << "H2 =" << endl << H2 << endl;
		cout << "R1 =" << endl << R1 << endl;
		cout << "R2 =" << endl << R2 << endl;

		// Compute essential matrix (Task 2)
		Mat F, E, R, T;
		motion.findEssential();
		F = motion.getFundamental();
		E = motion.getEssential();
		R = motion.getRotation();
		T = motion.getTranslation();
		cout << "F =" << endl << F << endl;
		cout << "E =" << endl << E << endl;
		cout << "R =" << endl << R << endl;
		cout << "T =" << endl << T << endl;

		// Find 3d info (Task 3)

		// Reset for next sequence
		motion.reset();
		cout << endl;

	}

	return 0;
}

