// EcEn 631 - Robotic Vision
// Assignment 4 - 3D Reconstruction and Trajectory Estimation
// Luke Newmeyer

#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "stereo.h"
#include "ball.h"

using std::cout;
using std::endl;
using std::vector;
using std::string;
using cv::Mat;
using cv::Size;
using cv::Point;
using cv::Scalar;
using cv::Point2f;

// Define calibration parameters
#define CALIBRATION_LEFT_INTRINSIC \
		1690.816493140574, 0, 334.2100930518936, \
		0, 1694.136704168955, 236.2425753638842, \
		0, 0, 1
#define CALIBRATION_LEFT_DISTORTION \
		-0.5191186765058345, \
		1.538827670363897, \
		0.002422391389104507, \
		-0.001102616739162721, \
		-33.84065471582704
#define CALIBRATION_RIGHT_INTRINSIC \
		1686.489467639931, 0, 335.7403477559992, \
		0, 1690.138405505449, 214.4290609858097, \
		0, 0, 1
#define CALIBRATION_RIGHT_DISTORTION \
		-0.5051369316322002, \
		1.510079171649894, \
		0.003553366568451884, \
		0.001672805989432977, \
		-22.97201594178351
#define CALIBRATION_STEREO_ROTATION \
		0.9999152095720598, 0.006481027324160356, 0.01129468686095925, \
		-0.006655606693022458, 0.9998578969319266, 0.0154883453736245, \
		-0.01119270146173091, -0.01556220510365956, 0.9998162537218027
#define CALIBRATION_STEREO_TRANSLATION \
		-20.34597495695317, \
		0.02813668716162538, \
		-0.7068003812449918
#define CALIBRATION_STEREO_ESSENTIAL \
		-0.005019110887567181, 0.7062620738457547, 0.03907868556501053, \
		-0.934466874979228, -0.3212090278976019, 20.33425337079917, \
		0.1072805056596847, -20.34326608612715, -0.3154432821672009
#define CALIBRATION_STEREO_FUNDAMENTAL \
		1.811838880307009e-08, -2.544524831406549e-06, 0.0003565473306358181, \
		3.366030588070112e-06, 1.154755166395077e-06, -0.1252428709078041, \
		-0.001380984238462987, 0.1242142714653703, 1

// Defines for image input data
#define IMAGE_RECONSTRUCTION_LEFT "data/reconstruct_3d_left.jpg"
#define IMAGE_RECONSTRUCTION_RIGHT "data/reconstruct_3d_right.jpg"
#define CHESSBOARD_ROWS 10
#define CHESSBOARD_COLUMNS 7
#define PITCH_FOLDER "data/ball_pitch_3/"
#define PITCH_PREFIX_LEFT "PitchL"
#define PITCH_PREFIX_RIGHT "PitchR"
#define PITCH_IMAGE_TYPE ".jpg"

// Defines for image output data
#define OUTPUT_FOLDER "output/"
#define OUTPUT_PREFIX_LEFT "track_left"
#define OUTPUT_PREFIX_RIGHT "track_right"
#define	OUTPUT_IMAGE_TYPE ".jpg"
#define OUTPUT_COMPOSITE_LEFT "track_left_composite"
#define OUTPUT_COMPOSITE_RIGHT "track_right_composite"
#define OUTPUT_CORNERS_LEFT "left_corners"
#define OUTPUT_CORNERS_RIGHT "right_corners"

// Display window parameters
#define WINDOW_LEFT_CAMERA "Left Camera"
#define WINDOW_RIGHT_CAMERA "Right Camera"
#define WAIT_TIME_FAST 300
#define WAIT_TIME_SLOW 0

// Functions for application
void findTrajectory(vector<Point3f> points, Point3f& poly_x, Point3f& poly_y);
string generateFilename(string folder, string prefix, int number, string type);
vector<Point2f> getOuterCorners(vector<Point2f> corners);
void drawCorners(Mat& image, vector<Point2f> corners);
void printPointData(vector<Point3f> corners);
void printPointData2(vector<Point2f> corners);

int main()
{
	// Open display windows
	cv::namedWindow(WINDOW_LEFT_CAMERA, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(WINDOW_RIGHT_CAMERA, CV_WINDOW_AUTOSIZE);

	// Load image for 3D reconstruction
	Mat left_image = cv::imread(IMAGE_RECONSTRUCTION_LEFT, cv::IMREAD_GRAYSCALE);
	Mat right_image = cv::imread(IMAGE_RECONSTRUCTION_RIGHT, cv::IMREAD_GRAYSCALE);
	Size image_size = left_image.size();
	Size pattern_size = Size(CHESSBOARD_ROWS, CHESSBOARD_COLUMNS);

	// Setup matricies for calibration parameters
	Mat left_intrinsic = (cv::Mat_<double>(3,3) << CALIBRATION_LEFT_INTRINSIC);
	Mat left_distortion = (cv::Mat_<double>(5,1) << CALIBRATION_LEFT_DISTORTION);
	Mat right_intrinsic = (cv::Mat_<double>(3,3) << CALIBRATION_RIGHT_INTRINSIC);
	Mat right_distortion = (cv::Mat_<double>(5,1) << CALIBRATION_RIGHT_DISTORTION);
	Mat rotation = (cv::Mat_<double>(3,3) << CALIBRATION_STEREO_ROTATION);
	Mat translation = (cv::Mat_<double>(3,1) << CALIBRATION_STEREO_TRANSLATION);
	Mat essential = (cv::Mat_<double>(3,3) << CALIBRATION_STEREO_ESSENTIAL);
	Mat fundamental = (cv::Mat_<double>(3,3) << CALIBRATION_STEREO_FUNDAMENTAL);

	// Create calibration object and set calibration parameters
	Stereo stereo(image_size, pattern_size);
	stereo.setCalibration(left_intrinsic, left_distortion, right_intrinsic,
			right_distortion, rotation, translation, essential, fundamental);

	// Find corners and undistort outer points
	vector<Point2f> left_corners, right_corners;
	Mat left_display, right_display;
	cv::findChessboardCorners(left_image, pattern_size, left_corners);
	cv::findChessboardCorners(right_image, pattern_size, right_corners);
	cv::TermCriteria criteria = cv::TermCriteria( CV_TERMCRIT_EPS + 
			CV_TERMCRIT_ITER, 40, 0.001);
	Size search_size = Size(5,5);
	Size zero_zone = Size(-1,-1);
	cv::cornerSubPix(left_image, left_corners, search_size, zero_zone, criteria);
	cv::cornerSubPix(right_image, right_corners, search_size, zero_zone, criteria);
	left_corners = getOuterCorners(left_corners);
	right_corners = getOuterCorners(right_corners);

	// Display corners on image
	cv::cvtColor(left_image, left_display, CV_GRAY2BGR);
	cv::cvtColor(right_image, right_display, CV_GRAY2BGR);
	drawCorners(left_display, left_corners);
	drawCorners(right_display, right_corners);
	cv::imshow(WINDOW_LEFT_CAMERA, left_display);
	cv::imshow(WINDOW_RIGHT_CAMERA, right_display);
	cv::waitKey(WAIT_TIME_SLOW);

	// Save images
	string folder = OUTPUT_FOLDER;
	string prefix = OUTPUT_CORNERS_LEFT;
	string type = OUTPUT_IMAGE_TYPE;
	string image_name = folder + prefix + type;
	cv::imwrite(image_name, left_display);
	prefix = OUTPUT_CORNERS_RIGHT;
	image_name = folder + prefix + type;
	cv::imwrite(image_name, right_display);

	// Undistort points
	vector<Point2f> left_corners_undistort, right_corners_undistort;
	left_corners_undistort = stereo.rectifyPointLeft(left_corners);
	right_corners_undistort = stereo.rectifyPointRight(right_corners);

	// Undistort image
	Mat left_undistort, right_undistort;
	stereo.rectifyImage(left_image, right_image, left_undistort,
			right_undistort);

	// Display undistorted corners on image
	cv::cvtColor(left_undistort, left_display, CV_GRAY2BGR);
	cv::cvtColor(right_undistort, right_display, CV_GRAY2BGR);
	drawCorners(left_display, left_corners_undistort);
	drawCorners(right_display, right_corners_undistort);
	cv::imshow(WINDOW_LEFT_CAMERA, left_display);
	cv::imshow(WINDOW_RIGHT_CAMERA, right_display);
	cv::waitKey(WAIT_TIME_SLOW);

	// Transform points and print data
	vector<Point3f> left_corners_transform;
	left_corners_transform = stereo.transformPoints(left_corners_undistort,
			right_corners_undistort);
	printPointData(left_corners_transform);

	// Set background for ball detection
	int image_number = 0;
	image_name = generateFilename(PITCH_FOLDER, PITCH_PREFIX_LEFT, 
			image_number, PITCH_IMAGE_TYPE);
	left_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
	image_name = generateFilename(PITCH_FOLDER, PITCH_PREFIX_RIGHT,
			image_number, PITCH_IMAGE_TYPE);
	right_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
	image_number++;
	Ball ball;

	// Setup variables for loop
	vector<Point3f> trajectory;
	vector<Point2f> left_balls, right_balls;
	Point3f poly_x, poly_y;
	char keypress = 0;
	bool detected = false;
	int output_number = 0;

	// Loop through images to detect ball
	while(left_image.data && right_image.data && keypress != 'q') {

		// Process image in ball object
		detected = ball.detectBall(left_image, right_image, left_display,
				right_display);

		// Compute trajectory
		if (detected) {

			// Rectify and transform points
			left_balls = stereo.rectifyPointLeft(ball.getBallsLeft());
			right_balls = stereo.rectifyPointRight(ball.getBallsRight());
			trajectory = stereo.transformPoints(left_balls, right_balls);
			printPointData(trajectory);

			// Compute trajectory using least squares
			if (trajectory.size() >= 3) {
				findTrajectory(trajectory, poly_x, poly_y);
				cout << "X trajectory: " << poly_x << endl;
				cout << "Y trajectory: " << poly_y << endl;
			}
		}

		// For now set image to display in window
		cv::imshow(WINDOW_LEFT_CAMERA, left_display);
		cv::imshow(WINDOW_RIGHT_CAMERA, right_display);
		keypress = cv::waitKey(WAIT_TIME_FAST);

		// Write image output
		if (detected) {
			image_name = generateFilename(OUTPUT_FOLDER, OUTPUT_PREFIX_LEFT,
					output_number, OUTPUT_IMAGE_TYPE);
			cv::imwrite(image_name, left_display);
			image_name = generateFilename(OUTPUT_FOLDER, OUTPUT_PREFIX_RIGHT,
					output_number, OUTPUT_IMAGE_TYPE);
			cv::imwrite(image_name, right_display);
			output_number++;
		}
				
		// Load next image
		image_name = generateFilename(PITCH_FOLDER, PITCH_PREFIX_LEFT, 
			image_number, PITCH_IMAGE_TYPE);
		left_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
		image_name = generateFilename(PITCH_FOLDER, PITCH_PREFIX_RIGHT,
			image_number, PITCH_IMAGE_TYPE);
		right_image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
		image_number++;
	}

	// Draw composite image
	ball.getBackground(left_image, right_image);
	cv::cvtColor(left_image, left_display, CV_GRAY2BGR);
	cv::cvtColor(right_image, right_display, CV_GRAY2BGR);
	ball.drawComposite(left_display, right_display);
	cv::imshow(WINDOW_LEFT_CAMERA, left_display);
	cv::imshow(WINDOW_RIGHT_CAMERA, right_display);
	cv::waitKey(WAIT_TIME_SLOW);

	// Write image output
	folder = OUTPUT_FOLDER;
	prefix = OUTPUT_COMPOSITE_LEFT;
	type = OUTPUT_IMAGE_TYPE;
	image_name = folder + prefix + type;
	cv::imwrite(image_name, left_display);
	prefix = OUTPUT_COMPOSITE_RIGHT;
	image_name = folder + prefix + type;
	cv::imwrite(image_name, right_display);

	// Compute 3D trajectory of ball
	left_balls = stereo.rectifyPointLeft(ball.getBallsLeft());
	cout << "unrectified" << endl;
	printPointData2(ball.getBallsLeft());
	cout << "rectified" << endl;
	printPointData2(left_balls);
	right_balls = stereo.rectifyPointRight(ball.getBallsRight());
	trajectory = stereo.transformPoints(left_balls, right_balls);

	// Print trajectory
	printPointData(trajectory);

	return 0;
}

void findTrajectory(vector<Point3f> points, Point3f& poly_x, Point3f& poly_y)
{
	// Create matrix structures
	Mat A_x(points.size(), 3, CV_32FC1);
	Mat b_x(points.size(), 1, CV_32FC1);
	Mat A_y(points.size(), 3, CV_32FC1);
	Mat b_y(points.size(), 1, CV_32FC1);

	// Form points into matrix A and vector b
	int index = 0;
	float z1, z2, z3;
	for (Point3f point : points) {

		// Set values of z
		z1 = 1;
		z2 = point.z;
		z3 = z2*z2;

		// Set values in matricies for x trajectory
		A_x.at<float>(index,0) = z3;
		A_x.at<float>(index,1) = z2;
		A_x.at<float>(index,2) = z1;
		b_x.at<float>(index,0) = point.x;

		// Set values in matricies for y trajectory
		A_y.at<float>(index,0) = z3;
		A_y.at<float>(index,1) = z2;
		A_y.at<float>(index,2) = z1;
		b_y.at<float>(index,0) = point.y;

		// Update index
		index++;
	}

	// Compute least squares solutions
	Mat x, y;
	cv::solve(A_x, b_x, x, cv::DECOMP_QR);
	cv::solve(A_y, b_y, y, cv::DECOMP_QR);

	// Translate output to Point3f type
	poly_x.x = x.at<float>(0,0);
	poly_x.y = x.at<float>(1,0);
	poly_x.z = x.at<float>(2,0);
	poly_y.x = y.at<float>(0,0);
	poly_y.y = y.at<float>(1,0);
	poly_y.z = y.at<float>(2,0);
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

vector<Point2f> getOuterCorners(vector<Point2f> corners)
{
	// Select points to undistort
	vector<Point2f> points;
	points.push_back(corners[0]);
	points.push_back(corners[CHESSBOARD_ROWS-1]);
	points.push_back(corners[(CHESSBOARD_COLUMNS-1)*CHESSBOARD_ROWS]);
	points.push_back(corners[CHESSBOARD_COLUMNS*CHESSBOARD_ROWS-1]);

	return points;
}

void drawCorners(Mat& image, vector<Point2f> corners)
{
	// Loop through and draw circles on corners of output
	for(Point2f corner : corners) {
		cv::circle(image, corner, 4, Scalar(255,0,0), -1, 8, 0);
	}
}

void printPointData(vector<Point3f> corners)
{
	for(Point3f corner : corners) {
		cout << corner << endl;
	}
}

void printPointData2(vector<Point2f> corners)
{
	for(Point2f corner : corners) {
		cout << corner << endl;
	}
}
