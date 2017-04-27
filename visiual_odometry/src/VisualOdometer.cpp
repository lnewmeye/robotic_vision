// EcEn 631 - Robotic Vision
// Visual Odometry - VisualOdometer Class
// Luke Newmeyer

#include "VisualOdometer.hpp"

using std::ofstream;
using std::vector;
using std::string;
using std::endl;
using cv::Mat;
using cv::Point2f;
using cv::Scalar;

// DEBUG define (comment out when not needed)
#define DEBUG
#ifdef DEBUG
#include <iostream>
#include <opencv2/highgui.hpp>
#define DEBUG_WINDOW_NAME "Debug Window"
#endif

VisualOdometer::VisualOdometer()
{
	// Set rotation matrix to defualt
	P = Mat::eye(4, 4, CV_64F);
	R = Mat::eye(3, 3, CV_64F);
	T = Mat::zeros(3, 1, CV_64F);
}

void VisualOdometer::setInitial(Mat image)
{
	motion_tracker.setInitial(image);
}

void VisualOdometer::findOdometry(Mat image)
{
	// Track motion
	F = motion_tracker.trackMotion(image);

	// Compute essential matrix
	//E = Mt * F * M;
	E = cv::findEssentialMat(motion_tracker.getPreviousPoints(),
			motion_tracker.getNewPoints(), M);

	Mat T_last = T.clone();

	// Perform SVD on essential matrix and normalize
	Mat W, U, V, Vt;
	cv::SVD::compute(E, W, U, Vt);
	W = cv::Mat_<double>::zeros(3, 3);
	W.at<double>(0,0) = 1;
	W.at<double>(1,1) = 1;
	E = U * W * Vt;

	// Esimate rotation and translation
	cv::recoverPose(E, motion_tracker.getPreviousPoints(), 
			motion_tracker.getNewPoints(), M, R, T);

	// Reverse T if reversed
	if (T.at<double>(2) > 0) {
		T = -1 * T;
		std::cout << "translation reversed" << std::endl;
	}

	// the R was poorly resolved from recoverPose set to identity
	if (R.at<double>(0,0) < 0 || R.at<double>(1,1) < 0 || 
			R.at<double>(2,2) < 0) {
		R = Mat::eye(3,3, CV_64F);
		std::cout << "no rotation used" << std::endl;
	}

	if (motion_tracker.getNewPoints().size() < ODOMETER_MINIMUM_POINTS) {
		R = Mat::eye(3,3,CV_64F);
		std::cout << "no rotation used" << std::endl;
		T = T_last;
	}

	// Scale translation matrix
	T *= scale;

	// Concatenate rotation and translation into motion matrix
	P = concatenateMotion(R, T, P);

	// Save motion to history
	rotation_history.push_back(R.clone());
	translation_history.push_back(T.clone());
	motion_history.push_back(P.clone());
}

void VisualOdometer::setIntrinsic(Mat M)
{
	this->M = M.clone();
	cv::transpose(M, Mt);
}

void VisualOdometer::setScale(double scale)
{
}

void VisualOdometer::drawMotion(Mat image, Mat& display)
{
	// Convert to RGB to draw in color
	cv::cvtColor(image, display, CV_GRAY2BGR);

	// Get corner positions for MotionTracker
	vector<Point2f> previous_points = motion_tracker.getPreviousPoints();
	vector<Point2f> new_points = motion_tracker.getNewPoints();

	// Create variables for loop
	Point2f corner, tip;
	
	// Draw points and arrows on image
	for (unsigned i = 0; i < previous_points.size(); i++) {

		// Get point and arrow tip
		corner = new_points[i];
		tip = previous_points[i];

		// Draw circle and flow vector
		cv::circle(display, corner, 4, Scalar(0,255,0), -1, 8, 0);
		cv::arrowedLine(display, corner, tip, Scalar(0,0,255));
	}
}

/*void VisualOdometer::writeMotion(string filename)
{
	// Open file
	ofstream file;
	file.open(filename);

	// Write data to file
	for (unsigned i = 0; i < rotation_history.size(); i++) {

		// Load point in history
		Mat Pi = rotation_history[i];

		// Write history to file
		file << Pi.at<double>(0,0) << " ";
		file << Pi.at<double>(0,1) << " ";
		file << Pi.at<double>(0,2) << " ";
		file << Pi.at<double>(0,3) << " ";
		file << Pi.at<double>(1,0) << " ";
		file << Pi.at<double>(1,1) << " ";
		file << Pi.at<double>(1,2) << " ";
		file << Pi.at<double>(1,3) << " ";
		file << Pi.at<double>(2,0) << " ";
		file << Pi.at<double>(2,1) << " ";
		file << Pi.at<double>(2,2) << " ";
		file << Pi.at<double>(2,3) << " ";
		file << endl;
	}

	// Close file
	file.close();
}*/

void VisualOdometer::writeMotion(string filename)
{
	// Open file
	ofstream file;
	file.open(filename);

	// Write data to file
	for (unsigned i = 0; i < rotation_history.size(); i++) {

		// Load point in history
		Mat Ri = rotation_history[i];
		Mat Ti = rotation_history[i];

		// Write history to file
		file << Ri.at<double>(0,0) << " ";
		file << Ri.at<double>(0,1) << " ";
		file << Ri.at<double>(0,2) << " ";
		file << Ti.at<double>(0,0) << " ";
		file << Ri.at<double>(1,0) << " ";
		file << Ri.at<double>(1,1) << " ";
		file << Ri.at<double>(1,2) << " ";
		file << Ti.at<double>(1,0) << " ";
		file << Ri.at<double>(2,0) << " ";
		file << Ri.at<double>(2,1) << " ";
		file << Ri.at<double>(2,2) << " ";
		file << Ti.at<double>(2,0) << " ";
		file << endl;
	}

	// Close file
	file.close();
}


Mat VisualOdometer::concatenateMotion(Mat Ri, Mat Ti, Mat Pi)
{
	// Form new rotation and translation matrix
	Mat Pj = cv::Mat_<double>::zeros(4,4);

	// Insert rotation
	Pj.at<double>(0,0) = Ri.at<double>(0,0);
	Pj.at<double>(0,1) = Ri.at<double>(0,1);
	Pj.at<double>(0,2) = Ri.at<double>(0,2);
	Pj.at<double>(1,0) = Ri.at<double>(1,0);
	Pj.at<double>(1,1) = Ri.at<double>(1,1);
	Pj.at<double>(1,2) = Ri.at<double>(1,2);
	Pj.at<double>(2,0) = Ri.at<double>(2,0);
	Pj.at<double>(2,1) = Ri.at<double>(2,1);
	Pj.at<double>(2,2) = Ri.at<double>(2,2);

	// Insert tranlsation
	Pj.at<double>(0,3) = Ti.at<double>(0,0);
	Pj.at<double>(1,3) = Ti.at<double>(1,0);
	Pj.at<double>(2,3) = Ti.at<double>(2,0);

	// Insert bottom row
	Pj.at<double>(3,0) = 0;
	Pj.at<double>(3,1) = 0;
	Pj.at<double>(3,2) = 0;
	Pj.at<double>(3,3) = 1;

	return Pj * Pi;
}
