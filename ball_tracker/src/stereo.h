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
		void runCalibration();
		void setCalibration(cv::Mat left_intrinsic, cv::Mat left_distortion,
				cv::Mat right_intrinsic, cv::Mat right_distortion,
				cv::Mat rotation, cv::Mat translation, cv::Mat essential,
				cv::Mat fundamental);
		void rectifyImage(Mat input_left, Mat input_right,
				Mat& output_left, Mat& output_right);
		std::vector<cv::Point2f> rectifyPointLeft(
				std::vector<cv::Point2f> points);
		std::vector<cv::Point2f> rectifyPointRight(
				std::vector<cv::Point2f> points);
		std::vector<Point3f> transformPoints(std::vector<Point2f> points_left,
				std::vector<Point2f> points_right);

		// Functions for geting variables
		cv::Mat getRotation() {return rotation;}
		cv::Mat getTranslation() {return translation;}
		cv::Mat getEssential() {return essential;}
		cv::Mat getFundamental() {return fundamental;}

		// Calibration objects (public is needed)
		Calibration left_camera;
		Calibration right_camera;

	private:
		// Private functions for intermediate steps
		void computeRectification();

		// Private variables
		cv::Size image_size;
		cv::Size pattern_size;
		cv::Mat rotation;
		cv::Mat translation;
		cv::Mat essential;
		cv::Mat fundamental;
		cv::Mat disparity;
		cv::Mat left_intrinsic;
		cv::Mat left_distortion;
		cv::Mat left_rectification;
		cv::Mat left_projection;
		cv::Mat right_intrinsic;
		cv::Mat right_distortion;
		cv::Mat right_rectification;
		cv::Mat right_projection;
};
