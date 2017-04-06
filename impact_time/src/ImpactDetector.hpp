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
#define IMPACT_CAMERA_CENTER_X 320
#define IMPACT_CAMERA_CENTER_Y 240
#define IMPACT_REGION_X 280
#define IMPACT_REGION_Y 120
#define IMPACT_REGION_WIDTH 90
#define IMPACT_REGION_HEIGHT 250
#define IMPACT_CAMERA_FOCAL 825.0
#define IMPACT_CAN_WIDTH 59.0

class ImpactDetector
{
	public:
		void setInitial(cv::Mat image, cv::Mat& display);
		double detectImpact(cv::Mat image, cv::Mat& display);

	private:
		//void removeBadCorners(std::vector<cv::Point2f>* corners,
		//		std::vector<uchar> status);
		std::vector<double> findDistances(std::vector<cv::Point2f> points);
		double computeRatio(std::vector<double> last_distances,
				std::vector<double> new_distances);
		double estimateFrames(std::vector<double> ratios);
		double estimateDistance(std::vector<cv::Point2f> corners);
		void drawCorners(cv::Mat& image, std::vector<cv::Point2f> corners, 
				cv::Scalar color);
		void drawRegion(cv::Mat& image, cv::Rect region, cv::Scalar color);

		// Priavate data structures
		std::vector<cv::Mat> previous_images;
		std::vector<std::vector<cv::Point2f>> previous_corners;
		std::vector<std::vector<double>> previous_distances;
		std::vector<double> previous_ratios;
};
