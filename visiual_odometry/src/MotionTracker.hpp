// EcEn 631 - Robotic Vision
// Visual Odometry - MotionTracker Class
// Luke Newmeyer

#include <vector>
#include <deque>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

#define MOTION_CORNER_QUANTITY 5000
#define MOTION_CORNER_QUALITY 0.001
#define MOTION_CORNER_DISTANCE 10

#define OPTICAL_FLOW_LEVEL 4

#define POINT_TRACK_MAX_FRAMES 5

#define MOTION_SUBPIX_SIZE 5
#define MOTION_ZERO_ZONE 5

class MotionTracker
{
	public:
		// Object constructor
		MotionTracker();

		// Motion tracking main functions
		void setInitial(cv::Mat image);
		cv::Mat trackMotion(cv::Mat image);

		// Different motion tracking algorithms developed
		cv::Mat firstAttempt(cv::Mat image);
		cv::Mat secondAttempt(cv::Mat image);
		cv::Mat thirdAttempt(cv::Mat image);
		cv::Mat fourthAttempt(cv::Mat image);
		cv::Mat fifthAttempt(cv::Mat image);

		// Get functions (to pass private information)
		std::vector<cv::Point2f> getPreviousPoints();
		std::vector<cv::Point2f> getNewPoints();

	private:
		// Motion tracking main data structures
		cv::Size image_size;
		std::vector<cv::Point2f> previous_points;
		std::vector<cv::Point2f> new_points;
		cv::Mat previous_image;

		// Matricies
		cv::Mat F; // Fundamental matrix

		// Special variables for third attempt
		std::deque<std::vector<std::vector<cv::Point2f>>> point_track;

		// Special variables for fourth attempt (also used in fifth)
		cv::Ptr<cv::FeatureDetector> detector;

		// Special variables for fifth attempt
		cv::Ptr<cv::DescriptorMatcher> matcher;

		// Point criteria
		cv::Size SUBPIX_SIZE;
		cv::Size ZERO_ZONE;
		cv::TermCriteria SUBPIX_CRITERIA;
		cv::TermCriteria FLOW_CRITERIA;
		cv::Size FLOW_SIZE;
};
