// EcEn 631 - Robotic Vision
// Structure from Motion - Motion Class
// Luke Newmeyer

// Includes for class
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>

// Motion class defines
#define MOTION_CORNER_QUANTITY 1000
#define MOTION_CORNER_QUALITY 0.01
#define MOTION_CORNER_DISTANCE 2

// Parameters for template matching task
#define MATCH_SELECTION_WIDTH 60
#define MATCH_SELECTION_HEIGHT 60
#define MATCH_BLOCK_WIDTH 30
#define MATCH_BLOCK_HEIGHT 30

class Motion
{
	public:
		// Object operations
		void setInitial(cv::Mat image);
		void addFrame(cv::Mat image);

		// Object get functions
		std::vector<cv::Mat> getMotionImages();

	private:
		// Private functions
		std::vector<cv::Point2f> opticalMatchingFine(cv::Mat& initial, 
				cv::Mat& image, std::vector<cv::Point2f> corners);
		void drawMotion(cv::Mat& image, std::vector<cv::Point2f> corners, 
			std::vector<cv::Point2f> flow);

		// Private variables
		std::vector<std::vector<cv::Point2f>> tracked_corners;
		std::vector<cv::Mat> previous_images;
};
