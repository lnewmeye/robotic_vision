// EcEn 631 - Robotic Vision
// Ball tracking object
// Luke Newmeyer

#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>

// Image parameters
#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480

// Image processing parameters
#define THRESHOLD_LEVEL 10
#define EROSION_SIZE 7

// Search box parameters
#define INITIAL_LEFT_X 357
#define INITIAL_LEFT_Y 111
#define INITIAL_RIGHT_X 293
#define INITIAL_RIGHT_Y 117
#define SEARCH_BOX_WIDTH 200
#define SEARCH_BOX_HEIGHT 200

// Ball detection parameters
#define BALL_DETECTIONS_MAX 32
#define FRAME_WAIT_MAX 10

// Ball catcher parameters
#define BALL_CATCHER_X 10.0
#define BALL_CATCHER_Y 23.0
#define BALL_CATCHER_Z 6.0

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

// State machine type
typedef enum CatchState {
	CATCH_DETECT,
	CATCH_TRACK,
	CATCH_WAIT
} CatchState;

// Position struct for ball position data
typedef struct Position {
	bool found;
	int box_left;
	int box_top;
	int box_width;
	int box_height;
	double centroid_x;
	double centroid_y;
} Position;

class Ball
{
	public:
		// Public functions
		Ball();
		Ball(cv::Mat left_background, cv::Mat right_background);
		int captureBall(cv::Mat left_image, cv::Mat right_image);
		void initialize(cv::Mat left_background, cv::Mat right_background);
		bool detectBall(cv::Mat left_image, cv::Mat right_image);
		bool detectBall(cv::Mat left_image, cv::Mat right_image,
				cv::Mat& left_display, cv::Mat& right_dislay);
		void getBackground(cv::Mat& left_image, cv::Mat& right_image);
		void drawComposite(cv::Mat& left_image, cv::Mat& right_image);
		std::vector<cv::Point2f> getBallsLeft();
		std::vector<cv::Point2f> getBallsRight();
		void predictTrajectory(double& x_prediction, double& y_prediction);

	private:
		// Private variables
		CatchState catch_state;
		int ball_count;
		int frame_count;
		cv::Mat left_background;
		cv::Mat right_background;
		cv::Mat erosion_structure;
		cv::Rect left_search;
		cv::Rect right_search;
		cv::Point left_search_center;
		cv::Point right_search_center;
		Position left_ball;
		Position right_ball;
		std::vector<cv::Point2f> left_points;
		std::vector<cv::Point2f> right_points;
		std::vector<Position> left_balls;
		std::vector<Position> right_balls;
		int image_height;
		int image_width;
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

		// Private functions
		void reset(cv::Mat left_image, cv::Mat right_image);
		void setSearchRegion(int left_x, int left_y, int right_x,
				int right_y);
		Position chooseComponent(cv::Mat centroids, cv::Mat stats,
				cv::Point search_center);
		void drawBall(cv::Mat& image, Position ball);
		void drawSearchRegion(cv::Mat& left_image, cv::Mat& right_image);
		std::vector<cv::Point3f> transformPoints();
		cv::Mat solveLsqQuadratic(std::vector<float> Z, std::vector<float> B);
		cv::Mat solveLsqLinear(std::vector<float> z, std::vector<float> b);
};
