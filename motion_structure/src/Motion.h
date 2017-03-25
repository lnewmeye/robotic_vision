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

// Guessed parameters (for task 1)
#define IMAGE_CENTER_GUESS_X 320
#define IMAGE_CENTER_GUESS_Y 240
#define IMAGE_FOCAL_GUESS 1145
#define IMAGE_DISTORTION_GUESS \
	-0.2575507934862325, \
	 0.04884197280891184, \
	-0.001409751152446753, \
	-0.001543707631160005, \
	 0.9076529637760357

// Input image parameters
#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480

class Motion
{
	public:
		// Object operations
		Motion();
		void setInitial(cv::Mat image);
		void addFrame(cv::Mat image);
		cv::Mat averageIntrinsic();
		void rectifyImage(cv::Mat& display, cv::Mat& display2);

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
		std::vector<cv::Mat> F_measurement;
		cv::Size image_size;
		cv::Mat F;  // Fundamental matrix
		cv::Mat E;  // Essential matrix
		cv::Mat M;  // Camera matrix
		cv::Mat H1; // Homogrpahy image 1
		cv::Mat H2; // Homography image 2
		cv::Mat R1; // Rectirication image 1
		cv::Mat R2; // Rectirication image 2
		cv::Mat D;  // Distortion matrix
		cv::Mat R;  // Rotation matrix
		cv::Mat T;  // Translation matrix
};
