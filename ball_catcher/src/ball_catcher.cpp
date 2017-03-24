// EcEn 631 - Robotic Vision
// Assignment 4 - 3D Reconstruction and Trajectory Estimation
// Luke Newmeyer

#include <iostream>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "Ball.hpp"

using std::cout;
using std::endl;
using std::string;
using cv::Mat;
using cv::Size;
using cv::Point;
using cv::Scalar;
using cv::Point2f;

// Defines for image input data
#define PITCH_FOLDER_1 "data/ball_pitch_1/"
#define PITCH_FOLDER_2 "data/ball_pitch_2/"
#define PITCH_FOLDER_3 "data/ball_pitch_3/"
#define PITCH_FOLDER_4 "data/ball_pitch_4/"
#define PITCH_FOLDER_5 "data/ball_pitch_5/"
#define PITCH_PREFIX_LEFT "PitchL"
#define PITCH_PREFIX_RIGHT "PitchR"
#define PITCH_IMAGE_TYPE ".jpg"

// Display window parameters
#define WINDOW_LEFT_CAMERA "Left Camera"
#define WINDOW_RIGHT_CAMERA "Right Camera"
#define WAIT_TIME_FAST 300
#define WAIT_TIME_SLOW 0

// Functions for application
string generateFilename(string folder, string prefix, int number, string type);

int main()
{
	// Open display windows
	cv::namedWindow(WINDOW_LEFT_CAMERA, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(WINDOW_RIGHT_CAMERA, CV_WINDOW_AUTOSIZE);

	// Setup variables for loop
	Ball ball;
	Mat left_image, right_image;
	Mat left_display, right_display;

	// Load first images to process
	int image_number = 0;
	string image_name = generateFilename(PITCH_FOLDER_1, PITCH_PREFIX_LEFT, 
			image_number, PITCH_IMAGE_TYPE);
	left_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
	image_name = generateFilename(PITCH_FOLDER_1, PITCH_PREFIX_RIGHT,
			image_number, PITCH_IMAGE_TYPE);
	right_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
	image_number++;

	// Loop through images to detect ball
	while(left_image.data && right_image.data) {

		// Process image in ball object
		int ball_count = ball.captureBall(left_image, right_image);

		if (ball_count >= 3) {
			double x, y;
			ball.predictTrajectory(x, y);
			cout << "(X, Y) = " << x << ", " << y << endl;
		}

		// Output display
		left_display = left_image;
		right_display = right_image;
		cv::imshow(WINDOW_LEFT_CAMERA, left_display);
		cv::imshow(WINDOW_RIGHT_CAMERA, right_display);
		cv::waitKey(1);

		// Load next image
		image_name = generateFilename(PITCH_FOLDER_1, PITCH_PREFIX_LEFT, 
			image_number, PITCH_IMAGE_TYPE);
		left_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
		image_name = generateFilename(PITCH_FOLDER_1, PITCH_PREFIX_RIGHT,
			image_number, PITCH_IMAGE_TYPE);
		right_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
		image_number++;
	}

	// Load first images to process
	image_number = 0;
	image_name = generateFilename(PITCH_FOLDER_2, PITCH_PREFIX_LEFT, 
			image_number, PITCH_IMAGE_TYPE);
	left_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
	image_name = generateFilename(PITCH_FOLDER_2, PITCH_PREFIX_RIGHT,
			image_number, PITCH_IMAGE_TYPE);
	right_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
	image_number++;

	// Loop through images to detect ball
	while(left_image.data && right_image.data) {

		// Process image in ball object
		int ball_count = ball.captureBall(left_image, right_image);

		if (ball_count >= 3) {
			double x, y;
			ball.predictTrajectory(x, y);
			cout << "(X, Y) = " << x << ", " << y << endl;
		}

		// Output display
		left_display = left_image;
		right_display = right_image;
		cv::imshow(WINDOW_LEFT_CAMERA, left_display);
		cv::imshow(WINDOW_RIGHT_CAMERA, right_display);
		cv::waitKey(1);

		// Load next image
		image_name = generateFilename(PITCH_FOLDER_2, PITCH_PREFIX_LEFT, 
			image_number, PITCH_IMAGE_TYPE);
		left_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
		image_name = generateFilename(PITCH_FOLDER_2, PITCH_PREFIX_RIGHT,
			image_number, PITCH_IMAGE_TYPE);
		right_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
		image_number++;
	}

	// Load first images to process
	image_number = 0;
	image_name = generateFilename(PITCH_FOLDER_3, PITCH_PREFIX_LEFT, 
			image_number, PITCH_IMAGE_TYPE);
	left_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
	image_name = generateFilename(PITCH_FOLDER_3, PITCH_PREFIX_RIGHT,
			image_number, PITCH_IMAGE_TYPE);
	right_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
	image_number++;

	// Loop through images to detect ball
	while(left_image.data && right_image.data) {

		// Process image in ball object
		int ball_count = ball.captureBall(left_image, right_image);

		if (ball_count >= 3) {
			double x, y;
			ball.predictTrajectory(x, y);
			cout << "(X, Y) = " << x << ", " << y << endl;
		}

		// Output display
		left_display = left_image;
		right_display = right_image;
		cv::imshow(WINDOW_LEFT_CAMERA, left_display);
		cv::imshow(WINDOW_RIGHT_CAMERA, right_display);
		cv::waitKey(1);

		// Load next image
		image_name = generateFilename(PITCH_FOLDER_3, PITCH_PREFIX_LEFT, 
			image_number, PITCH_IMAGE_TYPE);
		left_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
		image_name = generateFilename(PITCH_FOLDER_3, PITCH_PREFIX_RIGHT,
			image_number, PITCH_IMAGE_TYPE);
		right_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
		image_number++;
	}


	// Load first images to process
	image_number = 0;
	image_name = generateFilename(PITCH_FOLDER_4, PITCH_PREFIX_LEFT, 
			image_number, PITCH_IMAGE_TYPE);
	left_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
	image_name = generateFilename(PITCH_FOLDER_4, PITCH_PREFIX_RIGHT,
			image_number, PITCH_IMAGE_TYPE);
	right_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
	image_number++;

	// Loop through images to detect ball
	while(left_image.data && right_image.data) {

		// Process image in ball object
		int ball_count = ball.captureBall(left_image, right_image);

		if (ball_count >= 3) {
			double x, y;
			ball.predictTrajectory(x, y);
			cout << "(X, Y) = " << x << ", " << y << endl;
		}

		// Output display
		left_display = left_image;
		right_display = right_image;
		cv::imshow(WINDOW_LEFT_CAMERA, left_display);
		cv::imshow(WINDOW_RIGHT_CAMERA, right_display);
		cv::waitKey(1);

		// Load next image
		image_name = generateFilename(PITCH_FOLDER_4, PITCH_PREFIX_LEFT, 
			image_number, PITCH_IMAGE_TYPE);
		left_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
		image_name = generateFilename(PITCH_FOLDER_4, PITCH_PREFIX_RIGHT,
			image_number, PITCH_IMAGE_TYPE);
		right_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
		image_number++;
	}


	// Load first images to process
	image_number = 0;
	image_name = generateFilename(PITCH_FOLDER_5, PITCH_PREFIX_LEFT, 
			image_number, PITCH_IMAGE_TYPE);
	left_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
	image_name = generateFilename(PITCH_FOLDER_5, PITCH_PREFIX_RIGHT,
			image_number, PITCH_IMAGE_TYPE);
	right_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
	image_number++;

	// Loop through images to detect ball
	while(left_image.data && right_image.data) {

		// Process image in ball object
		int ball_count = ball.captureBall(left_image, right_image);

		if (ball_count >= 3) {
			double x, y;
			ball.predictTrajectory(x, y);
			cout << "(X, Y) = " << x << ", " << y << endl;
		}

		// Output display
		left_display = left_image;
		right_display = right_image;
		cv::imshow(WINDOW_LEFT_CAMERA, left_display);
		cv::imshow(WINDOW_RIGHT_CAMERA, right_display);
		cv::waitKey(1);

		// Load next image
		image_name = generateFilename(PITCH_FOLDER_5, PITCH_PREFIX_LEFT, 
			image_number, PITCH_IMAGE_TYPE);
		left_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
		image_name = generateFilename(PITCH_FOLDER_5, PITCH_PREFIX_RIGHT,
			image_number, PITCH_IMAGE_TYPE);
		right_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
		image_number++;
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
