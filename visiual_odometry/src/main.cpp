// EcEn 631 - Robotic Vision
// Assigment 7 - Visual Odometry
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
#include "VisualOdometer.hpp"

// Standard namespaces used by default
using std::cout;
using std::endl;
using std::queue;
using std::vector;
using std::string;

// OpenCV namespaces used by default
using cv::VideoCapture;
using cv::VideoWriter;
using cv::Point2f;
using cv::Point3f;
using cv::Point3d;
using cv::Mat;

// Parameters for display window
#define DISPLAY_WINDOW_1 "Display Window 1"
#define DISPLAY_WINDOW_2 "Display Window 2"
#define DISPLAY_TIME_FAST 1
#define DISPLAY_TIME_MEDIUM 0
#define DISPLAY_TIME_SLOW 0

// Parameters for image input
#define IMAGE_SEQUENCE_1 "data/vo_practice_sequence/%06d.png"
#define IMAGE_SEQUENCE_2 "data/byu_hallway_sequence/%06d.png"

#define SEQUENCE_1_INTRINSIC \
		7.070912000000e+02, 0.000000000000e+00, 6.018873000000e+02, \
		0.000000000000e+00, 7.070912000000e+02, 1.831104000000e+02, \
		0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+0
#define SEQUENCE_2_INTRINSIC \
		6.7741251774486568e+02, 0.0000000000000000e+00, 3.2312557438767283e+02, \
		0.0000000000000000e+00, 6.8073800850564749e+02, 2.2477413395670021e+02, \
		0.0000000000000000e+00, 0.0000000000000000e+00, 1.0000000000000000e+00

//#define SEQUENCE_1_SCALE 2.15
#define SEQUENCE_1_SCALE 1000.0 // TODO: Fix this
#define SEQUENCE_2_SCALE 1000.0 // TODO: Fix this

#define OUTPUT_FILE_1 "output/vo_practice_sequence_data.txt"
#define OUTPUT_FILE_2 "output/byu_hallway_sequence_data.txt"
#define OUTPUT_SEQUENCE_1 "output/vo_practice_sequence/%06d.png"
#define OUTPUT_SEQUENCE_2 "output/byu_hallway_sequence/%06d.png"

int main()
{
	// Create windows and objects for project
	VideoCapture image_sequence;
	VideoWriter output_sequence;
	Mat image, display, display1, display2;
	int keypress;
	cv::namedWindow(DISPLAY_WINDOW_1, CV_WINDOW_AUTOSIZE);
	//cv::namedWindow(DISPLAY_WINDOW_2, CV_WINDOW_AUTOSIZE);

	//Motion motion;
	VisualOdometer odometer;

	// Load vector with image sequence strings
	vector<string> image_names;
	//image_names.push_back(IMAGE_SEQUENCE_1);
	image_names.push_back(IMAGE_SEQUENCE_2);

	// Create queue of output files
	queue<string> output_files;
	//output_files.push(OUTPUT_FILE_1);
	output_files.push(OUTPUT_FILE_2);

	// Create queue of output sequences
	queue<string> output_sequences;
	//output_sequences.push(OUTPUT_SEQUENCE_1);
	output_sequences.push(OUTPUT_SEQUENCE_2);

	// Create queue of scale factors
	queue<double> scale_factors;
	//scale_factors.push(SEQUENCE_1_SCALE);
	scale_factors.push(SEQUENCE_2_SCALE);

	// Create queue of camera parameters
	queue<Mat> camera_parameters;
	//camera_parameters.push((cv::Mat_<double>(3,3) << SEQUENCE_1_INTRINSIC));
	camera_parameters.push((cv::Mat_<double>(3,3) << SEQUENCE_2_INTRINSIC));

	// Iterate through image names and operate on sequence
	for (string image_name : image_names) {

		// Open image sequence
		cout << "Processing sequence: " << image_name << endl;
		image_sequence.open(image_name);

		// Read in first image and set as inital in Motion object
		image_sequence >> image;
		cv::cvtColor(image, image, CV_BGR2GRAY);
		odometer.setInitial(image);

		// Open output sequence
		//output_sequence.open(OUTPUT_SEQUENCE_1, 0, 0, image.size());
		output_sequence.open(output_sequences.front(), 0, 0, image.size());
		output_sequences.pop();

		// Set camera parameters
		//Mat M = (cv::Mat_<double>(3,3) << SEQUENCE_1_INTRINSIC);
		Mat M = camera_parameters.front();
		camera_parameters.pop();
		//TODO: Find a better solution to this to accomodate more than one sequence with different parameters
		odometer.setIntrinsic(M);

		// Set scale factor
		odometer.setScale(scale_factors.front());
		scale_factors.pop();

		// Read in next image and begin loop
		image_sequence >> image;
		keypress = 0;

		// Loop while images available
		while(image.data && keypress != 'q') {

			// Convert image color
			cv::cvtColor(image, image, CV_BGR2GRAY);

			// Find odometry of image
			odometer.findOdometry(image);

			// Draw motion on image
			odometer.drawMotion(image, display);

			// Display and wait
			cv::imshow(DISPLAY_WINDOW_1, display);
			keypress = cv::waitKey(DISPLAY_TIME_FAST);

			// Write image out
			output_sequence << display;

			// Load next image
			image_sequence >> image;
		}

		// Get output data to write
		odometer.writeMotion(output_files.front()); 
		output_files.pop();

		// Reset for next sequence
		//TODO: implement this
		cout << endl;

	}

	return 0;
}

