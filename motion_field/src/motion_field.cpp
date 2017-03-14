// EcEn 631 - Robotic Vision
// Assigment 5 - Motion Field
// Luke Newmeyer

#include <iostream>
#include <vector>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

// Standard namespaces used by default
using std::cout;
using std::endl;
using std::vector;
using std::string;

// OpenCV namespaces used by default
using cv::VideoCapture;
using cv::Mat;
using cv::Rect;
using cv::Size;
using cv::Point;
using cv::Scalar;
using cv::Point2f;

// DEBUG option
//#define DEBUG
#ifdef DEBUG
#define DISPLAY_DEBUG "Debug Display"
#endif

// Parameters for display window
#define DISPLAY_WINDOW_1 "Display Window 1"
#define DISPLAY_WINDOW_2 "Display Window 2"
#define DISPLAY_WINDOW_3 "Display Window 3"
#define DISPLAY_TIME_FAST 10

// Parameters for image input
#define IMAGE_OPTICAL_FLOW "data/optical_flow/O%01d.jpg"

// Parameters for image output
#define IMAGE_OUTPUT_FOLDER "output/"
#define IMAGE_OUTPUT_TYPE ".jpg"
#define IMAGE_OUTPUT_FLOW1 "flow1_"
#define IMAGE_OUTPUT_FLOW2 "flow2_"
#define IMAGE_OUTPUT_FLOW3 "flow3_"
#define IMAGE_OUTPUT_MATCH1 "match1_"
#define IMAGE_OUTPUT_MATCH2 "match2_"
#define IMAGE_OUTPUT_MATCH3 "match3_"

// Parameters for optical flow task
#define OPTICAL_CORNER_MAX 500
#define OPTICAL_CORNER_QUALITY 0.01
#define OPTICAL_CORNER_DISTANCE 5
#define OPTICAL_FLOW_LEVEL 3

// Parameters for template matching task
#define MATCH_SELECTION_WIDTH 40
#define MATCH_SELECTION_HEIGHT 40
#define MATCH_BLOCK_WIDTH 20
#define MATCH_BLOCK_HEIGHT 20

// Function declarations
void drawCorners(Mat& image, vector<Point2f> corners);
void drawFlow(Mat& image, vector<Point2f> corners, vector<Point2f> flow,
		vector<unsigned char> status);
string generateFilename(string folder, string prefix, int number, string type);
vector<Point2f> opticalMatching(Mat& initial, Mat& image, vector<Point2f> corners);

int main()
{
	// Create VideoCaputre to read in Optical Flow images
	VideoCapture flow_images(IMAGE_OPTICAL_FLOW);
	if (!flow_images.isOpened()) {
		cout << "Error: couldn't open optical flow images" << endl;
		return -1;
	}

	// Open display windows
	cv::namedWindow(DISPLAY_WINDOW_1, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(DISPLAY_WINDOW_2, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(DISPLAY_WINDOW_3, CV_WINDOW_AUTOSIZE);
	#ifdef DEBUG
	cv::namedWindow(DISPLAY_DEBUG, CV_WINDOW_NORMAL);
	#endif

	// Read in first set of images
	Mat initial, image, image1, image2, image3;
	flow_images >> initial;
	flow_images >> image1;
	flow_images >> image2;
	flow_images >> image3;

	// Convert initial images to grayscale
	cv::cvtColor(initial, initial, CV_BGR2GRAY);
	cv::cvtColor(image1, image1, CV_BGR2GRAY);
	cv::cvtColor(image2, image2, CV_BGR2GRAY);
	cv::cvtColor(image3, image3, CV_BGR2GRAY);

	// Create variables for optical flow loop
	vector<Point2f> corners;
	Mat display1, display2, display3;
	vector<Point2f> flow1, flow2, flow3;
	vector<unsigned char> status1, status2, status3;
	vector<float> error;
	cv::TermCriteria subpix_criteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 
			40, 0.001);
    cv::TermCriteria flow_criteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,
			20, 0.03);
	Size subpix_size(5,5);
	Size zero_zone(-1,-1);
	Size flow_size(31,31);
	int image_number = 0;
	string image_name;
	int keypress = 0;

	while(image1.data && keypress != 'q') {

		// Get features to track in initial image
		cv::goodFeaturesToTrack(initial, corners, OPTICAL_CORNER_MAX,
				OPTICAL_CORNER_QUALITY, OPTICAL_CORNER_DISTANCE);
		cv::cornerSubPix(initial, corners, subpix_size, zero_zone, 
				subpix_criteria);

		// Compute optical flow on images 1, 2, and 3
		cv::calcOpticalFlowPyrLK(initial, image1, corners, flow1,
				status1, error, flow_size, OPTICAL_FLOW_LEVEL,
				flow_criteria, 0, 0.001);
		if(image2.data) {
			cv::calcOpticalFlowPyrLK(initial, image2, corners, flow2,
					status2, error, flow_size, OPTICAL_FLOW_LEVEL,
					flow_criteria, 0, 0.001);
		}
		if(image3.data) {
			cv::calcOpticalFlowPyrLK(initial, image3, corners, flow3,
					status3, error, flow_size, OPTICAL_FLOW_LEVEL,
					flow_criteria, 0, 0.001);
		}

		// Draw flow on image
		cv::cvtColor(image1, display1, CV_GRAY2BGR);
		drawFlow(display1, corners, flow1, status1);
		if(image2.data) {
			cv::cvtColor(image2, display2, CV_GRAY2BGR);
			drawFlow(display2, corners, flow2, status2);
		}
		if(image3.data) {
			cv::cvtColor(image3, display3, CV_GRAY2BGR);
			drawFlow(display3, corners, flow3, status3);
		}

		// Display image and wait keypress
		cv::imshow(DISPLAY_WINDOW_1, display1);
		if(image2.data)
			cv::imshow(DISPLAY_WINDOW_2, display2);
		if(image3.data)
			cv::imshow(DISPLAY_WINDOW_3, display3);
		keypress = cv::waitKey(DISPLAY_TIME_FAST);

		// Write images to file
		image_name = generateFilename(IMAGE_OUTPUT_FOLDER,
				IMAGE_OUTPUT_FLOW1, image_number, IMAGE_OUTPUT_TYPE);
		cv::imwrite(image_name, display1);
		if(image2.data) {
			image_name = generateFilename(IMAGE_OUTPUT_FOLDER,
					IMAGE_OUTPUT_FLOW2, image_number, IMAGE_OUTPUT_TYPE);
			cv::imwrite(image_name, display2);
		}
		if(image3.data) {
			image_name = generateFilename(IMAGE_OUTPUT_FOLDER,
					IMAGE_OUTPUT_FLOW3, image_number, IMAGE_OUTPUT_TYPE);
			cv::imwrite(image_name, display3);
		}
		image_number++;

		// Swap images and load new image
		flow_images >> image;
		initial = image1.clone();
		image1 = image2.clone();
		image2 = image3.clone();
		cv::cvtColor(image, image3, CV_BGR2GRAY);
	}

	// Create VideoCaputre to read in Template Match Images
	VideoCapture match_images(IMAGE_OPTICAL_FLOW);
	if (!match_images.isOpened()) {
		cout << "Error: couldn't open template match images" << endl;
		return -1;
	}

	// Read in first set of images
	match_images >> initial;
	match_images >> image1;
	match_images >> image2;
	match_images >> image3;

	// Convert initial images to grayscale
	cv::cvtColor(initial, initial, CV_BGR2GRAY);
	cv::cvtColor(image1, image1, CV_BGR2GRAY);
	cv::cvtColor(image2, image2, CV_BGR2GRAY);
	cv::cvtColor(image3, image3, CV_BGR2GRAY);

	// Create/reset variables for loop
	image_number = 0;
	keypress = 0;

	// Compute optical flow using matching
	while(image1.data && keypress != 'q') {

		// Get features to track in initial image
		cv::goodFeaturesToTrack(initial, corners, OPTICAL_CORNER_MAX,
				OPTICAL_CORNER_QUALITY, OPTICAL_CORNER_DISTANCE);
		cv::cornerSubPix(initial, corners, subpix_size, zero_zone, 
				subpix_criteria);

		// Compute optical flow on images 1, 2, and 3
		flow1 = opticalMatching(initial, image1, corners);
		if(image2.data)
			flow2 = opticalMatching(initial, image1, corners);
		if(image3.data)
			flow3 = opticalMatching(initial, image3, corners);

		// Draw flow on image
		cv::cvtColor(image1, display1, CV_GRAY2BGR);
		drawFlow(display1, corners, flow1, status1);
		if(image2.data) {
			cv::cvtColor(image2, display2, CV_GRAY2BGR);
			drawFlow(display2, corners, flow2, status2);
		}
		if(image3.data) {
			cv::cvtColor(image3, display3, CV_GRAY2BGR);
			drawFlow(display3, corners, flow3, status3);
		}

		// Display image and wait keypress
		cv::imshow(DISPLAY_WINDOW_1, display1);
		if(image2.data)
			cv::imshow(DISPLAY_WINDOW_2, display2);
		if(image3.data)
			cv::imshow(DISPLAY_WINDOW_3, display3);
		keypress = cv::waitKey(DISPLAY_TIME_FAST);

		// Write images to file
		image_name = generateFilename(IMAGE_OUTPUT_FOLDER,
				IMAGE_OUTPUT_MATCH1, image_number, IMAGE_OUTPUT_TYPE);
		cv::imwrite(image_name, display1);
		if(image2.data) {
			image_name = generateFilename(IMAGE_OUTPUT_FOLDER,
					IMAGE_OUTPUT_MATCH2, image_number, IMAGE_OUTPUT_TYPE);
			cv::imwrite(image_name, display2);
		}
		if(image3.data) {
			image_name = generateFilename(IMAGE_OUTPUT_FOLDER,
					IMAGE_OUTPUT_MATCH3, image_number, IMAGE_OUTPUT_TYPE);
			cv::imwrite(image_name, display3);
		}
		image_number++;

		// Swap images and load new image
		match_images >> image;
		initial = image1.clone();
		image1 = image2.clone();
		image2 = image3.clone();
		cv::cvtColor(image, image3, CV_BGR2GRAY);
	}

	return 0;
}

void drawCorners(Mat& image, vector<Point2f> corners)
{
	// Iterate through corners and draw on display
	for (Point2f corner : corners) {
		cv::circle(image, corner, 4, Scalar(0,255,0), -1, 8, 0);
	}
}

void drawFlow(Mat& image, vector<Point2f> corners, vector<Point2f> flow,
		vector<unsigned char> status)
{
	// Create variables for loop
	Point2f corner, tip;

	// Iterate through vectors and plot if status shows flow to be found
	for (unsigned i = 0; i < status.size(); i++) {

		// Check if pattern found
		if (status[i]) {

			// Get corner and flow values
			corner = corners[i];
			tip = flow[i];

			// Draw circle and flow vector
			cv::circle(image, corner, 4, Scalar(0,255,0), -1, 8, 0);
			cv::arrowedLine(image, tip, corner, Scalar(0,0,255));
		}
	}
}

string generateFilename(string folder, string prefix, int number, string type)
{
        string value;

        // Adjust string value to have leading 0's
        if (number < 10) {
                value = "0" + std::to_string(number);
        }
        else {
                value = std::to_string(number);
        }

        // Return file path
        return folder + prefix + value + type;
}

vector<Point2f> opticalMatching(Mat& initial, Mat& image,
		vector<Point2f> corners)
{
	// Create variables for loop
	int x1, x2, y1, y2;
	Mat selection, block, result;
	Rect selection_rect, block_rect;
	int image_width = initial.cols;
	int image_height = initial.rows;
	vector<Point2f> matches;
	Point max_location;
	Point2f match;

	// Iterate through corners and find best fit flow
	for (Point2f corner : corners) {

		// Find positions for defining selection
		x1 = corner.x - MATCH_SELECTION_WIDTH/2;
		y1 = corner.y - MATCH_SELECTION_HEIGHT/2;
		x2 = x1 + MATCH_SELECTION_WIDTH;
		y2 = y1 + MATCH_SELECTION_HEIGHT;

		// Correct positions of wrong
		if (x1 <= 0)
			x1 = 1;
		if (y1 <= 0)
			y1 = 1;
		if (x2 >= image_width)
			x2 = image_width-1;
		if (y2 >= image_height)
			y2 = image_height-1;

		// Create rects defining regions for matching
		selection_rect = Rect(Point(x1,y1), Point(x2,y2));

		// Find positions for defining block
		x1 = corner.x - MATCH_BLOCK_WIDTH/2;
		y1 = corner.y - MATCH_BLOCK_HEIGHT/2;
		x2 = x1 + MATCH_BLOCK_WIDTH;
		y2 = y1 + MATCH_BLOCK_HEIGHT;

		// Correct positions of wrong
		if (x1 <= 0)
			x1 = 1;
		if (y1 <= 0)
			y1 = 1;
		if (x2 >= image_width)
			x2 = image_width-1;
		if (y2 >= image_height)
			y2 = image_height-1;

		// Set rect defining block
		block_rect = Rect(Point(x1,y1), Point(x2,y2));

		// Select regions of initial and match images
		selection = image(selection_rect);
		block = initial(block_rect);

		// Match block in selection
		cv::matchTemplate(selection, block, result, cv::TM_CCOEFF);
		normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, Mat());
		cv::minMaxLoc(result, NULL, NULL, NULL, &max_location);
		match.x = max_location.x + corner.x - MATCH_BLOCK_WIDTH/2;
		match.y = max_location.y + corner.y - MATCH_BLOCK_HEIGHT/2;
		matches.push_back(match);
	}

	return matches;
}
