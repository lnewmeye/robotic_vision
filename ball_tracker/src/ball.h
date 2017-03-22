// EcEn 631 - Robotic Vision
// Ball tracking object
// Luke Newmeyer

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

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
		void initialize(cv::Mat left_background, cv::Mat right_background);
		bool detectBall(cv::Mat left_image, cv::Mat right_image,
				cv::Mat& left_display, cv::Mat& right_dislay);
		void getBackground(cv::Mat& left_image, cv::Mat& right_image);
		void drawComposite(cv::Mat& left_image, cv::Mat& right_image);
		std::vector<cv::Point2f> getBallsLeft();
		std::vector<cv::Point2f> getBallsRight();

	private:
		// Private variables
		cv::Mat left_background;
		cv::Mat right_background;
		cv::Mat erosion_structure;
		cv::Rect left_search;
		cv::Rect right_search;
		cv::Point left_search_center;
		cv::Point right_search_center;
		Position left_ball;
		Position right_ball;
		Position left_ball_previous;
		Position right_ball_previous;
		std::vector<Position> left_balls;
		std::vector<Position> right_balls;
		int image_height;
		int image_width;

		// Private functions
		void setSearchRegion(int left_x, int left_y, int right_x,
				int right_y);
		Position chooseComponent(cv::Mat centroids, cv::Mat stats,
				cv::Point search_center);
		void drawBall(cv::Mat& image, Position ball);
		void drawSearchRegion(cv::Mat& left_image, cv::Mat& right_image);
};
