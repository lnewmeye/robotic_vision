#include <iostream>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using std::cout;
using std::endl;
using std::string;
using std::vector;

using cv::Mat;
using cv::Size;

// Display parameters
#define WINDOW_NAME "Display Window" // Name of display window
#define DISPLAY_TIME -1 // ms of time to display image

// Image input parameters
#define IMAGE_FOLDER "images/"
#define IMAGE_PREFIX "AR"
#define IMAGE_TYPE ".jpg"

// Ouptut image parameters (comment out when not needed)
//#define OUTPUT_IMAGE "output/task1_corner_detection.jpg"

// Function declarations
void locateCorners(Mat input_image, Mat& output_image);
string generateFilename(string folder, string prefix, int number, string type);
void saveImage(Mat image, string image_name);

int main()
{
	// Create image variables and load first image
	int image_number = 1;
	string image_name = generateFilename(IMAGE_FOLDER, IMAGE_PREFIX,
		image_number, IMAGE_TYPE);
	image_number++;
	Mat image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
	
	// Create variables for loop
	int keypress = 0;
	Mat display;
	
	// Create window
	cv::namedWindow(WINDOW_NAME, CV_WINDOW_AUTOSIZE);
	
	// Loop through iamges while 'q' is not pressed
	while (image.data && keypress != 'q') {
		
		// Locate corners on image
		locateCorners(image, display);
		
		// Display image
		cv::imshow(WINDOW_NAME, display);
		
		// Write image (if selected)
#ifdef OUTPUT_IMAGE
		saveImage(display, OUTPUT_IMAGE);
#endif
		
		// Wait for keypress
		keypress = cv::waitKey(DISPLAY_TIME);
		
		// Load nxt image
		image_name = generateFilename(IMAGE_FOLDER, IMAGE_PREFIX,
		image_number, IMAGE_TYPE);
		image_number++;
		image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
	}
	
	return 0;
}

void locateCorners(Mat input_image, Mat& output_image)
{
	// Create color version of input image
	cv::cvtColor(input_image, output_image, CV_GRAY2RGB);
	
	// Create variables
	vector<cv::Point2f> corners;
	Size pattern_size = Size(10,7);
	Size search_size = Size(5,5);
	Size zero_zone = Size(-1,-1);
	cv::TermCriteria criteria = cv::TermCriteria( CV_TERMCRIT_EPS + 
		CV_TERMCRIT_ITER, 40, 0.001);
	bool pattern_found;
	
	// Find chessboard corners, refine corners, and draw
	pattern_found = findChessboardCorners(input_image, pattern_size, corners);
	cv::cornerSubPix(input_image, corners, search_size, zero_zone, criteria);
	drawChessboardCorners(output_image, pattern_size, corners, pattern_found);
}

void saveImage(Mat image, string image_name)
{
	cv::imwrite(image_name, image);
}

string generateFilename(string folder, string prefix, int number, string type)
{
	// Create string of value
	string value = std::to_string(number);
	
	// Return file path
	return folder + prefix + value + type;
}
