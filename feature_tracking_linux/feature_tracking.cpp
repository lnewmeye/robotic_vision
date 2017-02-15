#include <iostream>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using std::cout;
using std::cin;
using std::endl;
using std::string;
using std::vector;

using cv::Mat;
using cv::VideoCapture;
using cv::namedWindow;
using cv::setWindowTitle;
using cv::createTrackbar;
using cv::setTrackbarMin;
using cv::setTrackbarMax;
using cv::setTrackbarPos;
using cv::getTrackbarPos;
using cv::imshow;
using cv::waitKey;
using cv::Point2f;
using cv::cvtColor;
using cv::threshold;
using cv::Canny;
using cv::goodFeaturesToTrack;
using cv::circle;
using cv::Scalar;
using cv::Vec2f;
using cv::HoughLinesP;
using cv::Vec4i;
using cv::Point;
using cv::absdiff;
using cv::VideoWriter;
using cv::Size;

// Options for video output
#define WRITE_VIDEO false
#define VIDEO_TITLE "video_output.avi"
#define VIDEO_RATE 30

// Options for auto rotate frame processing
#define AUTO_ROTATE true
#define FRAME_COUNT 2*VIDEO_RATE

// State type enumeration for state machine
enum State {VIDEO, BLACK, THRESHOLD, CANNY, CORNER, LINE, DIFFERENCE, QUIT};

// Function declarations
string type2str(int type);
void cornerDetect(Mat input, Mat output, int max_corners, double corner_quality);
//void lineDetect(Mat input, Mat output);
void lineDetect(Mat input, Mat output, int canny_th1, int canny_th2, 
	int line_threshold, int line_length, int line_gap);

// Begin main
int main()
{
	// Open camera and error and exit if failure to open
	VideoCapture camera(0);
	if (!camera.isOpened()) {
		cout << "Error: camera not open" << endl;
		return -1;
	}

	// If selected open video to write and exit if failure to open
#if WRITE_VIDEO == true
	VideoWriter video;
	Mat out;
	Size image_size = Size((int)camera.get(CV_CAP_PROP_FRAME_WIDTH),
		(int)camera.get(CV_CAP_PROP_FRAME_HEIGHT));
	video.open(VIDEO_TITLE, VideoWriter::fourcc('M', 'J', 'P', 'G'), VIDEO_RATE, image_size, true);
	if (!video.isOpened())
	{
		cout << "Could not open video output" << endl;
		return -1;
	}
#endif

	// If selected set variables sequence for auto rotate
#if AUTO_ROTATE == true
	int frame_count = 0;
	int rotate_count = 0;
	char keypress_sequence[6] = {'t', 'c', 'r', 'l', 'd', 'q'};
#endif
	
	// Create window for video output with trackbars for user control
	const string WINDOW_NAME = "Live Video";
	const string TRACKBAR_1 = "Option 1";
	const string TRACKBAR_2 = "Option 2";
	const string TRACKBAR_3 = "Option 3";
	namedWindow(WINDOW_NAME, CV_WINDOW_AUTOSIZE);
	setWindowTitle(WINDOW_NAME, "Live Video");
	createTrackbar(TRACKBAR_1, WINDOW_NAME, 0, 1);
	createTrackbar(TRACKBAR_2, WINDOW_NAME, 0, 1);
	createTrackbar(TRACKBAR_3, WINDOW_NAME, 0, 1);
	
	// Display options
	cout << "Use keypresses from the following menu to select image output" << endl;
	cout << "\tv: Video (no processing)" << endl;
	cout << "\tb: Black and white" << endl;
	cout << "\tt: Threshold" << endl;
	cout << "\tc: Canny edge" << endl;
	cout << "\tr: Corner detection" << endl;
	cout << "\tl: Line detection" << endl;
	cout << "\td: Difference image" << endl;
	cout << "\tp: Display Mat matrix type" << endl;
	cout << "\tq: Exit program" << endl;
	cout << endl;

	// Define constants for image threshold
	const double THRESHOLD_DEFAULT = 100;
	const double THRESHOLD_MIN = 0;
	const double THRESHOLD_MAX = 255;
	const int THRESHOLD_TYPE = 0; // Binary

	// Define constatns for canny edge detection
	const double CANNY_MIN = 1;
	const double CANNY_MAX = 255;
	const double CANNY_TH1_DEFAULT = 100;
	const double CANNY_TH2_DEFAULT = 145;

	// Define constants for corner detection
	const int CORNER_COUNT_MIN = 1;
	const int CORNER_COUNT_MAX = 100;
	const int CORNER_COUNT_DEFAULT = 100;
	const int CORNER_QUALITY_MIN = 5;
	const int CORNER_QUALITY_MAX = 15;
	const int CORNER_QUALITY_DEFAULT = 10;
	const double CORNER_QUALITY_STEP = 0.001;

	// Define constants for line detection
	const int LINE_THRESHOLD_MIN = 1;
	const int LINE_THRESHOLD_MAX = 50;
	const int LINE_THRESHOLD_DEFAULT = 1;
	const int LINE_LENGTH_MIN = 10;
	const int LINE_LENGTH_MAX = 100;
	const int LINE_LENGTH_DEFAULT = 70;
	const int LINE_GAP_MIN = 1;
	const int LINE_GAP_MAX = 100;
	const int LINE_GAP_DEFAULT = 10;

	// Instantiate variables for main loop
	char keypress = 0;
	State state = VIDEO;
	Mat frame, intermediate, display, previous;
	int threshold_value = THRESHOLD_DEFAULT;
	int canny_th1 = CANNY_TH1_DEFAULT;
	int canny_th2 = CANNY_TH2_DEFAULT;
	int max_corners = CORNER_COUNT_DEFAULT;
	double corner_quality = CORNER_QUALITY_DEFAULT;
	int line_threshold = LINE_THRESHOLD_DEFAULT;
	int line_length = LINE_LENGTH_DEFAULT;
	int line_gap = LINE_GAP_DEFAULT;

	// Loop frame capture, processing, and statemachine while kepyress is not q
	while (keypress != 'q')
	{
		// Capture frame
		camera >> frame;
		
		// Go through state cases
		switch (state) {
		case VIDEO:
			display = frame.clone();
			break;
		case BLACK:
			cvtColor(frame, display, CV_BGR2GRAY);
			break;
		case THRESHOLD:
			cvtColor(frame, intermediate, CV_BGR2GRAY);
			threshold_value = getTrackbarPos(TRACKBAR_1, WINDOW_NAME);
			threshold(intermediate, display, threshold_value, THRESHOLD_MAX, THRESHOLD_TYPE);
			break;
		case CANNY:
			canny_th1 = getTrackbarPos(TRACKBAR_1, WINDOW_NAME);
			canny_th2 = getTrackbarPos(TRACKBAR_2, WINDOW_NAME);
			Canny(frame, display, canny_th1, canny_th2, 3, false);
			break;
		case CORNER:
			display = frame.clone();
			max_corners = getTrackbarPos(TRACKBAR_1, WINDOW_NAME);
			corner_quality = getTrackbarPos(TRACKBAR_2, WINDOW_NAME)*CORNER_QUALITY_STEP;
			cornerDetect(frame, display, max_corners, corner_quality);
			break;
		case LINE:
			display = frame.clone();
			line_threshold = getTrackbarPos(TRACKBAR_1, WINDOW_NAME);
			line_length = getTrackbarPos(TRACKBAR_2, WINDOW_NAME);
			line_gap = getTrackbarPos(TRACKBAR_3, WINDOW_NAME);
			lineDetect(frame, display, canny_th1, canny_th2, line_threshold, line_length, line_gap);
			break;
		case DIFFERENCE:
			absdiff(previous, frame, display);
			break;
		}

		// Display video and capture keypress
		imshow(WINDOW_NAME, display);
		keypress = waitKey(1);

		// If selected increment frame count and auto rotate if count reached
#if AUTO_ROTATE == true
		frame_count++;
		if (frame_count >= FRAME_COUNT) {
			keypress = keypress_sequence[rotate_count];
			rotate_count++;
			frame_count = 0;
		}
#endif

		// Go through keypress cases and update state
		switch (keypress) {
		case 'v': cout << "Video (no processing)..." << endl;
			setTrackbarMin(TRACKBAR_1, WINDOW_NAME, 0);
			setTrackbarMax(TRACKBAR_1, WINDOW_NAME, 0);
			setTrackbarMin(TRACKBAR_2, WINDOW_NAME, 0);
			setTrackbarMax(TRACKBAR_2, WINDOW_NAME, 0);
			setTrackbarMin(TRACKBAR_3, WINDOW_NAME, 0);
			setTrackbarMax(TRACKBAR_3, WINDOW_NAME, 0);
			state = VIDEO;
			break;
		case 'b': cout << "Black and white..." << endl;
			setTrackbarMin(TRACKBAR_1, WINDOW_NAME, 0);
			setTrackbarMax(TRACKBAR_1, WINDOW_NAME, 0);
			setTrackbarMin(TRACKBAR_2, WINDOW_NAME, 0);
			setTrackbarMax(TRACKBAR_2, WINDOW_NAME, 0);
			setTrackbarMin(TRACKBAR_3, WINDOW_NAME, 0);
			setTrackbarMax(TRACKBAR_3, WINDOW_NAME, 0);
			state = BLACK;
			break;
		case 't': cout << "Threshold image..." << endl;
			setTrackbarMin(TRACKBAR_1, WINDOW_NAME, THRESHOLD_MIN);
			setTrackbarMax(TRACKBAR_1, WINDOW_NAME, THRESHOLD_MAX);
			setTrackbarPos(TRACKBAR_1, WINDOW_NAME, threshold_value);
			setTrackbarMin(TRACKBAR_2, WINDOW_NAME, 0);
			setTrackbarMax(TRACKBAR_2, WINDOW_NAME, 0);
			setTrackbarMin(TRACKBAR_3, WINDOW_NAME, 0);
			setTrackbarMax(TRACKBAR_3, WINDOW_NAME, 0);
			state = THRESHOLD;
			break;
		case 'c': cout << "Canny edge detection..." << endl;
			setTrackbarMin(TRACKBAR_1, WINDOW_NAME, CANNY_MIN);
			setTrackbarMax(TRACKBAR_1, WINDOW_NAME, CANNY_MAX);
			setTrackbarPos(TRACKBAR_1, WINDOW_NAME, canny_th1);
			setTrackbarMin(TRACKBAR_2, WINDOW_NAME, CANNY_MIN);
			setTrackbarMax(TRACKBAR_2, WINDOW_NAME, CANNY_MAX);
			setTrackbarPos(TRACKBAR_2, WINDOW_NAME, canny_th2);
			setTrackbarMin(TRACKBAR_3, WINDOW_NAME, 0);
			setTrackbarMax(TRACKBAR_3, WINDOW_NAME, 0);
			state = CANNY;
			break;
		case 'r': cout << "Corner detection..." << endl;
			setTrackbarMin(TRACKBAR_1, WINDOW_NAME, CORNER_COUNT_MIN);
			setTrackbarMax(TRACKBAR_1, WINDOW_NAME, CORNER_COUNT_MAX);
			setTrackbarPos(TRACKBAR_1, WINDOW_NAME, max_corners);
			setTrackbarMin(TRACKBAR_2, WINDOW_NAME, CORNER_QUALITY_MIN);
			setTrackbarMax(TRACKBAR_2, WINDOW_NAME, CORNER_QUALITY_MAX);
			setTrackbarPos(TRACKBAR_2, WINDOW_NAME, corner_quality);
			setTrackbarMin(TRACKBAR_3, WINDOW_NAME, 0);
			setTrackbarMax(TRACKBAR_3, WINDOW_NAME, 0);
			state = CORNER;
			break;
		case 'l': cout << "Line detection..." << endl;
			state = LINE;
			setTrackbarMin(TRACKBAR_1, WINDOW_NAME, LINE_THRESHOLD_MIN);
			setTrackbarMax(TRACKBAR_1, WINDOW_NAME, LINE_THRESHOLD_MAX);
			setTrackbarPos(TRACKBAR_1, WINDOW_NAME, line_threshold);
			setTrackbarMin(TRACKBAR_2, WINDOW_NAME, LINE_LENGTH_MIN);
			setTrackbarMax(TRACKBAR_2, WINDOW_NAME, LINE_LENGTH_MAX);
			setTrackbarPos(TRACKBAR_2, WINDOW_NAME, line_length);
			setTrackbarMin(TRACKBAR_3, WINDOW_NAME, LINE_GAP_MIN);
			setTrackbarMax(TRACKBAR_3, WINDOW_NAME, LINE_GAP_MAX);
			setTrackbarPos(TRACKBAR_3, WINDOW_NAME, line_gap);
			break;
		case 'd': cout << "Difference image..." << endl;
			state = DIFFERENCE;
			setTrackbarMin(TRACKBAR_1, WINDOW_NAME, 0);
			setTrackbarMax(TRACKBAR_1, WINDOW_NAME, 0);
			setTrackbarMin(TRACKBAR_2, WINDOW_NAME, 0);
			setTrackbarMax(TRACKBAR_2, WINDOW_NAME, 0);
			setTrackbarMin(TRACKBAR_3, WINDOW_NAME, 0);
			setTrackbarMax(TRACKBAR_3, WINDOW_NAME, 0);
			break;
		case 'p': cout << "Image type: " << type2str(display.type()) << endl;
			break;
		case 'q': cout << endl << "Exiting program..." << endl;
			break;
		}

		// Save frame as "previous" (for difference processing)
		previous = frame.clone();

		// If selected write frame to output
#if WRITE_VIDEO == true
		out = display.clone();
		video << out;
#endif

	}

	return 0;
}

string type2str(int type) {
	string r;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch (depth) {
	case CV_8U:  r = "8U"; break;
	case CV_8S:  r = "8S"; break;
	case CV_16U: r = "16U"; break;
	case CV_16S: r = "16S"; break;
	case CV_32S: r = "32S"; break;
	case CV_32F: r = "32F"; break;
	case CV_64F: r = "64F"; break;
	default:     r = "User"; break;
	}

	r += "C";
	r += (chans + '0');

	return r;
}

void cornerDetect(Mat input, Mat output, int max_corners, double corner_quality)
{
	// Define constants for function
	const double MINIMUM_DISTANCE = 10;
	const int BLOCK_SIZE = 3;
	const bool USE_HARRIS_DETECTOR = false;
	const double FREE_PARAMETER = 0.04;
	const int CIRCLE_RADIUS = 4;
	const Scalar CIRCLE_COLOR = Scalar(0, 0, 255);
	const int CIRCLE_THICKNESS = -1; // fill circle
	const int CIRCLE_TYPE = 8;
	const int CIRCLE_SHIFT = 0;
	
	// Convert input to grey scale
	cvtColor(input, input, CV_BGR2GRAY);

	// Instantiate corner data structure
	vector<Point2f> corners;

	// Find corners using cv::goodFeaturesToTrack()
	goodFeaturesToTrack(input, corners, max_corners, corner_quality, MINIMUM_DISTANCE, 
		Mat(), BLOCK_SIZE, USE_HARRIS_DETECTOR, FREE_PARAMETER);
	
	// Loop through and draw circles on corners of output
	for (int i = 0; i < corners.size(); i++) {
		circle(output, corners[i], CIRCLE_RADIUS, CIRCLE_COLOR, CIRCLE_THICKNESS, 
			CIRCLE_TYPE, CIRCLE_SHIFT);
	}
}

void lineDetect(Mat input, Mat output, int canny_th1, int canny_th2, 
	int line_threshold, int line_length, int line_gap)
{
	// Define constants for function
	const double DISTANCE_RESOLUTION = 1.0;
	const double ANGLE_RESOLUTION = CV_PI / 180; 
	
	Mat dst;

	// Instantiate corner data structure
	vector<Vec4i> lines;
	
	// Convert to Canny output
	Canny(input, dst, canny_th1, canny_th2, 3, false);
	
	// Find lines using Probabilistic Hough Line Transform
	HoughLinesP(dst, lines, DISTANCE_RESOLUTION, ANGLE_RESOLUTION, line_threshold, line_length, line_gap);
	
	// Loop through lins and draw on output image
	for (size_t i = 0; i < lines.size(); i++) {
		Vec4i l = lines[i];
		line(output, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, CV_AA);
	}
}
