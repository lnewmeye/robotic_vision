// EcEn 672 - Robotic Vision
// Assigment 7 - ImpactDetector Class
// Luke Newmeyer

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#define IMPACT_CORNER_MAX 500
#define IMPACT_CORNER_QUALITY 0.01
#define IMPACT_CORNER_DISTANCE 10

#define IMPACT_CORNER_DRAW_SIZE 4

class ImpactDetector
{
	public:
		void setInitial(cv::Mat image);
		void detectImpact(cv::Mat image, cv::Mat& display);

	private:
		void removeBadCorners(std::vector<cv::Point2d>* corners,
				std::vector<uchar> status);
		void drawCorners(cv::Mat& image, std::vector<cv::Point2d> corners, 
				cv::Scalar color);

		// Priavate data structures
		std::vector<cv::Mat> previous_images;
		std::vector<std::vector<cv::Point2d>> previous_corners;
};
