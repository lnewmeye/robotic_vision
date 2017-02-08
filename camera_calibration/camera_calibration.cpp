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
using cv::Point2f;
using cv::Point3f;
using cv::Vec2f;
using cv::Vec3f;

// Display parameters
#define WINDOW_NAME "Display Window" // Name of display window
#define DISPLAY_TIME 1 // ms of time to display image

// Image input parameters
#define IMAGE_FOLDER "images/"
#define IMAGE_PREFIX "AR"
#define IMAGE_TYPE ".jpg"
#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480
#define CHESSBOARD_ROWS 10
#define CHESSBOARD_COLUMNS 7

// Ouptut image parameters (comment out when not needed)
//#define OUTPUT_IMAGE "output/task1_corner_detection.jpg"

// Function declarations
vector<Point2f> locateCorners(Mat input_image, Mat& output_image);
string generateFilename(string folder, string prefix, int number, string type);
void saveImage(Mat image, string image_name);
void calibrateCamera(vector<vector<Point2f>> points);

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
	vector<Point2f> corners;
	vector<vector<Point2f>> calibration_points;
	
	// Create window
	cv::namedWindow(WINDOW_NAME, CV_WINDOW_AUTOSIZE);
	
	// Loop through iamges while 'q' is not pressed
	while (image.data && keypress != 'q') {
		
		// Locate corners on image and add to vector
		corners = locateCorners(image, display);
		
		// Copy corners to data structure
		calibration_points.push_back(corners);
			
		// Display image
		cv::imshow(WINDOW_NAME, display);
		
		// Write image (if selected)
#ifdef OUTPUT_IMAGE
		saveImage(display, OUTPUT_IMAGE);
#endif
		
		// Wait for keypress
		keypress = cv::waitKey(DISPLAY_TIME);
		
		// Load next image
		image_name = generateFilename(IMAGE_FOLDER, IMAGE_PREFIX,
			image_number, IMAGE_TYPE);
		image_number++;
		image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
	}
	
	// Calibrate camera
	calibrateCamera(calibration_points);
	
	return 0;
}

vector<Point2f> locateCorners(Mat input_image, Mat& output_image)
{
	// Create color version of input image
	cv::cvtColor(input_image, output_image, CV_GRAY2RGB);
	
	// Create variables
	vector<Point2f> corners;
	Size pattern_size = Size(CHESSBOARD_ROWS, CHESSBOARD_COLUMNS);
	Size search_size = Size(5,5);
	Size zero_zone = Size(-1,-1);
	cv::TermCriteria criteria = cv::TermCriteria( CV_TERMCRIT_EPS + 
		CV_TERMCRIT_ITER, 40, 0.001);
	bool pattern_found;
	
	// Find chessboard corners, refine corners, and draw
	pattern_found = findChessboardCorners(input_image, pattern_size, corners);
	cv::cornerSubPix(input_image, corners, search_size, zero_zone, criteria);
	drawChessboardCorners(output_image, pattern_size, corners, pattern_found);
	
	// Return found corners
	return corners;
}

void calibrateCamera(vector<vector<Point2f>> points)
{
	// Create vector of vector of point indicies
	vector<vector<Point3f>> indicies;
	for (int i = 0; i < points.size(); i++) {
		
		// Create vector of points
		vector<Point3f> index;
		
		// Loop through rows and columns and add points
		for (int j = 0; j < CHESSBOARD_ROWS; j++) {
			for (int k = 0; k < CHESSBOARD_COLUMNS; k++) {
				Point3f point;
				point.x = k;
				point.y = j;
				point.z = 0;
				index.push_back(point);
			}
		}
		
		// Push new vector to set of indicies
		indicies.push_back(index);
	}
	
	// Create data structures to get data
	Mat intrinsic = Mat::eye(3, 3, CV_64F);
	Mat distortion = Mat::zeros(5, 1, CV_64F);
	vector<Mat> rvecs, tvecs;
	Size image_size = Size(IMAGE_WIDTH, IMAGE_HEIGHT);
	int rms;
	
	// Run camera calibration
	rms = cv::calibrateCamera(indicies, points, image_size, intrinsic, 
		distortion, rvecs, tvecs);
	
	cout << "Root Mean Square = " << rms << endl;
	cout << "Intrinsic parameters = " << endl << intrinsic << endl;
	cout << "Distortion parameters = " << endl << distortion << endl;
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
