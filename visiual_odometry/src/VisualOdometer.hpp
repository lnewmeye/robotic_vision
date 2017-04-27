// EcEn 631 - Robotic Vision
// Visual Odometry - VisualOdometer Class
// Luke Newmeyer

#include <fstream>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include "MotionTracker.hpp"

#define ODOMETER_MINIMUM_POINTS 10

class VisualOdometer
{
	public:
		// Constroctor
		VisualOdometer();

		// Odometry functions
		void setInitial(cv::Mat image);
		void findOdometry(cv::Mat image);

		// Set functions (to set private information)
		void setIntrinsic(cv::Mat M);
		void setScale(double scale);

		// Display functions
		void drawMotion(cv::Mat image, cv::Mat& display);
		void writeMotion(std::string filename);

		// Internal functions
		cv::Mat concatenateMotion(cv::Mat Ri, cv::Mat Ti, cv::Mat Pi);

	private:
		MotionTracker motion_tracker;
		std::vector<cv::Mat> rotation_history;
		std::vector<cv::Mat> translation_history;
		std::vector<cv::Mat> motion_history;
		double scale;

		// Matricies
		cv::Mat M; // Intrinsic Matrix
		cv::Mat Mt; // Intrinsic Matrix (transpose)
		cv::Mat F; // Fundamental Matrix
		cv::Mat E; // Essential Matrix
		cv::Mat R; // Rotation Matrix (for most recent motion)
		cv::Mat T; // Translation Matrix (for most recent motion)
		cv::Mat P; // Motion Matrix (for most recent motion)
};
