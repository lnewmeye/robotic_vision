// EcEn 631 - Robotic Vision
// Stereo Calibration & Rectification - Stereo Class
// Luke Newmeyer

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include "calibration.h"

class Stereo
{
	public:
		// Functions for operating stereo calibration
		Stereo(cv::Size image_size, cv::Size pattern_size);
		~Stereo() {}
		void addCalibrationImage(cv::Mat left_image, cv::Mat right_image,
				cv::Mat& left_display, cv::Mat& right_display);
		void runCalibration();
		void rectifyImage(Mat input_left, Mat input_right,
				Mat& output_left, Mat& output_right);

		// Functions for geting variables
		cv::Mat getRotation() {return rotation;}
		cv::Mat getTranslation() {return translation;}
		cv::Mat getEssential() {return essential;}
		cv::Mat getFundamental() {return fundamental;}

		// Calibration objects (public is needed)
		Calibration left_camera;
		Calibration right_camera;

	private:
		cv::Size image_size;
		cv::Size pattern_size;
		std::vector<vector<Point2f>> left_points;
		std::vector<vector<Point2f>> right_points;
		cv::Mat rotation;
		cv::Mat translation;
		cv::Mat essential;
		cv::Mat fundamental;
};
