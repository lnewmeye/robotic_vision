// EcEn 631 - Robotic Vision
// Structure from Motion - Motion Class
// Luke Newmeyer

#include "Motion.hpp"

// DEBUG define (comment out when not needed)
//#define DEBUG
#ifdef DEBUG
#include <iostream>
#include <opencv2/highgui.hpp>
#define DEBUG_WINDOW_NAME "Debug Window"
#endif

// Standard namespaces used
using std::vector;

// OpenCV namespaces used
using cv::Mat;
using cv::Size;
using cv::Rect;
using cv::Vec2d;
using cv::Vec4d;
using cv::Point;
using cv::Scalar;
using cv::Point2f;
using cv::Point3f;
using cv::Point3d;
using cv::KeyPoint;

Motion::Motion()
{
	image_size = Size(IMAGE_WIDTH, IMAGE_HEIGHT);

#ifdef DEBUG
	cv::namedWindow(DEBUG_WINDOW_NAME, cv::WINDOW_KEEPRATIO);
#endif
}

void Motion::reset()
{
	// Empty vectors
	tracked_corners.clear();
	previous_images.clear();
	F_measurement.clear();

	// Reset matricies
	F.release();
	E.release();
	M.release();
	H1.release();
	H2.release();
	R1.release();
	R2.release();
	D.release();
	R.release();
	T.release();
}

void Motion::setInitial(Mat image)
{
	// Stup constants used for corner identification
	Size SUBPIX_SIZE(5,5);
	Size ZERO_ZONE(-1,-1);
	cv::TermCriteria SUBPIX_CRITERIA(CV_TERMCRIT_EPS +
			CV_TERMCRIT_ITER, 40, 0.001);

	// Find features to track in image
	vector<Point2f> corners_found;
	cv::goodFeaturesToTrack(image, corners_found, MOTION_CORNER_QUANTITY,
			MOTION_CORNER_QUALITY, MOTION_CORNER_DISTANCE);
	cv::cornerSubPix(image, corners_found, SUBPIX_SIZE, ZERO_ZONE,
			SUBPIX_CRITERIA);

	// Push corners_found to corners_tracked and save previous image
	tracked_corners.push_back(corners_found);
	previous_images.push_back(image.clone());
}

void Motion::addFrame(Mat image)
{
	// Get previous image and corners to compare with, push new image
	Mat previous_image = previous_images.back();
	previous_images.push_back(image.clone());
	vector<Point2f> corners = tracked_corners.back();
	vector<Point2f> flow_corners;
	vector<uchar> mask;
	uchar match;

	// Match points between images
	//cv::calcOpticalFlowPyrLK(previous_image, image, corners, flow2,
			//status2, error, flow_size, OPTICAL_FLOW_LEVEL,
			//flow_criteria, 0, 0.001);
	flow_corners = opticalMatchingFine(previous_image, image, corners);
	tracked_corners.push_back(flow_corners);
	corners = tracked_corners.front();
	F = cv::findFundamentalMat(corners, flow_corners, cv::FM_RANSAC,
			1, 0.70, mask);
	//F = cv::findFundamentalMat(corners, flow_corners, cv::FM_8POINT,
			//3, 0.99, mask);

	// Add fundamental matrix to vector of measurements
	F_measurement.push_back(F.clone());
	//std::cout << "F =" << std::endl << F << std::endl;

	// Remove corners not matching between imges
	for (unsigned j = mask.size(); j > 0; j--) {

		// Pop off from mask
		match = mask.back();
		mask.pop_back();

		// Remove item off of not a match
		if (!match) {
			for (unsigned k = 0; k < tracked_corners.size(); k++) {
				tracked_corners[k].erase(tracked_corners[k].begin()+j-1);
			}
		}
	}

	// Remove corners that go to far to right
	for (unsigned j = tracked_corners.back().size(); j > 0; j--) {
		//if (tracked_corners.back()[j].x > 540) {
		if (tracked_corners.back()[j].x > 580) {
			for (unsigned k = 0; k < tracked_corners.size(); k++) {
				tracked_corners[k].erase(tracked_corners[k].begin()+j);
			}
		}
	}
}

vector<Point2f> Motion::opticalMatchingFine(Mat& initial, Mat& image,
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

		// Replace corner if it comes back negative
		if (corner.x < 1 || corner.y < 1)
			corner = Point(1,1);

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

	// Resolve corner find grain location
	Size subpix_size(12,12);
	Size zero_zone(-1,-1);
	cv::TermCriteria subpix_criteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,
			40, 0.001);
	//cv::cornerSubPix(image, matches, subpix_size, zero_zone, subpix_criteria);

	return matches;
}

vector<Mat> Motion::getMotionImages()
{
	// Create vector of images to be returned
	vector<Point2f> corners, flow_corners;
	vector<Mat> display;
	Mat image;

	// Draw on images
	for (unsigned i = 1; i < previous_images.size(); i++) {

		// Get corners and image to be displayed
		corners = tracked_corners[i-1];
		flow_corners = tracked_corners[i];
		cv::cvtColor(previous_images[i-1], image, CV_GRAY2BGR);

		// Draw image and push to vector
		drawMotion(image, flow_corners, corners);
		display.push_back(image.clone());
	}

	return display;
}

Mat Motion::getMatchedImage()
{
	// Translate first points to type KeyPoint()
	vector<Point2f> corners_first = tracked_corners.front();
	vector<KeyPoint> matches_first;
	for (Point2f point : corners_first) {
		matches_first.push_back(KeyPoint(point, 1.0));
	}

	// Translate last points to type KeyPoint()
	vector<Point2f> corners_last = tracked_corners.back();
	vector<KeyPoint> matches_last;
	for (Point2f point : corners_last) {
		matches_last.push_back(KeyPoint(point, 1.0));
	}

	// Match keypoints on images
	Mat image_matched;
	vector<cv::DMatch> empty;
	Mat image_first = previous_images.front();
	Mat image_last = previous_images.back();
	cv::drawMatches(image_first, matches_first, image_last,
			matches_last, empty, image_matched, Scalar(0,0,255),
			Scalar(0,255,0), vector<char>(), 
			cv::DrawMatchesFlags::DEFAULT);

	return image_matched;
}

void Motion::drawMotion(Mat& image, vector<Point2f> corners,
	vector<Point2f> flow)
{
	// Create variables for loop
	Point2f corner, tip;

	// Loop through vectors and plot if status shows flow to be found
	for (unsigned i = 0; i < corners.size(); i++) {

		// Get corner and flow values
		corner = corners[i];
		tip = flow[i];

		// Draw circle and flow vector
		cv::circle(image, corner, 4, Scalar(0,255,0), -1, 8, 0);
		cv::arrowedLine(image, corner, tip, Scalar(0,0,255));
	}
}

Mat Motion::averageIntrinsic()
{
	// Reset matrix F with zeros
	F = cv::Mat_<double>::zeros(3, 3);

	// Iterate through measured parameters and sum
	for (Mat measurement : F_measurement) {
		F = F + measurement;
	}

	// Average measurements out
	F = F / (double)F_measurement.size();

	return F.clone();
}

void Motion::rectifyImage(Mat& display1, Mat& display2)
{
	// Note: this function is designed specifically for task 1 of
	// assignment 6 robotic vision. It estimates both intrinsic and
	// extrinsic parameters

	// Average intrinsic parameters to get best fundamental matrix
	//averageIntrinsic();
	vector<unsigned char> mask;
	vector<Point2f> corners = tracked_corners.front();
	vector<Point2f> flow_corners = tracked_corners.back();
	F = cv::findFundamentalMat(corners, flow_corners, cv::FM_8POINT,
			3, 0.99, mask);

	// Form camera matrix M
	M = cv::Mat_<double>::zeros(3, 3);
	M.at<double>(0,0) = IMAGE_FOCAL_GUESS;
	M.at<double>(1,1) = IMAGE_FOCAL_GUESS;
	M.at<double>(0,2) = IMAGE_CENTER_GUESS_X;
	M.at<double>(1,2) = IMAGE_CENTER_GUESS_Y;
	M.at<double>(2,2) = 1;

	// Form distortion matrix D
	D = (cv::Mat_<double>(5,1) << IMAGE_DISTORTION_GUESS);

	// Stereo rectify points in first and last images
	vector<Point2f> first_points = tracked_corners.front();
	vector<Point2f> last_points = tracked_corners.back();
	cv::stereoRectifyUncalibrated(first_points, last_points, F,
			image_size, H1, H2);

	// Transform homogrophies to rectifications
	cv::solve(M, H1*M, R1, cv::DECOMP_LU);
	cv::solve(M, H2*M, R2, cv::DECOMP_LU);

	// Rectify images
	Mat map1[2], map2[2];
	cv::initUndistortRectifyMap(M, D, R1, M,  image_size, CV_16SC2,
			map1[0], map1[1]);
	cv::initUndistortRectifyMap(M, D, R2, M,  image_size, CV_16SC2,
			map2[0], map2[1]);

	// Remap image
	Mat image1 = previous_images.front();
	Mat image2 = previous_images.back();
	cv::remap(image1, display1, map1[0], map1[1], cv::INTER_LINEAR);
	cv::remap(image2, display2, map2[0], map2[1], cv::INTER_LINEAR);

	// Convert to BGR and draw horizontal lines
	cv::cvtColor(display1, display1, cv::COLOR_GRAY2BGR);
	cv::cvtColor(display2, display2, cv::COLOR_GRAY2BGR);
	drawHorizontalLines(display1);
	drawHorizontalLines(display2);
}

void Motion::findEssential()
{
	// Note: this function is designed specifically for task 2 of
	// assignment 6 robotic vision. It computes the essential matrix
	// and additional parameters

	// Load intrinsic and distortion parameters
	//vector<uchar> mask;
	//vector<Point2f> corners = tracked_corners.front();
	//vector<Point2f> flow_corners = tracked_corners.back();
	// TODO: Decide when to find fundamental?
	//F = cv::findFundamentalMat(corners, flow_corners, cv::FM_RANSAC,
			//3, 0.99, mask);
	D = (cv::Mat_<double>(5,1) << IMAGE_DISTORTION_KNOWN);
	M = (cv::Mat_<double>(3,3) << IMAGE_INTRINSIC_KNOWN);

	// Compute essential matrix
	Mat Mt;
	cv::transpose(M, Mt);
	E = Mt * F * M;

	// Perform SVD on essential matrix and normalize
	Mat W, U, V, Vt;
	cv::SVD::compute(E, W, U, Vt);
	W = cv::Mat_<double>::zeros(3, 3);
	W.at<double>(0,0) = 1;
	W.at<double>(1,1) = 1;
	E = U * W * Vt;

	// Find rotation and translation using cv::recoverPose()
	vector<Point2f> first_points = tracked_corners.front();
	vector<Point2f> last_points = tracked_corners.back();
	cv::recoverPose(E, first_points, last_points, M, R, T);
}

vector<Point3d> Motion::findMotion(Mat& display1, Mat& display2)
{
	// Note: this function is designed specifically for task 3 of
	// assignment 6 robotic vision. It estimates motion of select
	// points in the first and last images

	// Select points on first and final image
	vector<Point2f> first_points = tracked_corners.front();
	vector<Point2f> last_points = tracked_corners.back();
	int index1, index2, index3, index4;
	index1 = 45; index2 = 44; index3 = 43; index4 = 42;
	vector<Point2f> motion1, motion2;
	vector<Point2f> undistort1, undistort2;
	motion1.push_back(first_points[index1]);
	motion1.push_back(first_points[index2]);
	motion1.push_back(first_points[index3]);
	motion1.push_back(first_points[index4]);
	motion2.push_back(last_points[index1]);
	motion2.push_back(last_points[index2]);
	motion2.push_back(last_points[index3]);
	motion2.push_back(last_points[index4]);

	// Draw points on first and last images
	Scalar color;
	cv::cvtColor(previous_images.front(), display1, cv::COLOR_GRAY2BGR);
	cv::cvtColor(previous_images.back(), display2, cv::COLOR_GRAY2BGR);
	color = Scalar(0, 0, 255);
	cv::circle(display1, first_points[index1], 3, color, -1);
	cv::circle(display2, last_points[index1], 3, color, -1);
	color = Scalar(0, 255, 0);
	cv::circle(display1, first_points[index2], 3, color, -1);
	cv::circle(display2, last_points[index2], 3, color, -1);
	color = Scalar(255, 0, 0);
	cv::circle(display1, first_points[index3], 3, color, -1);
	cv::circle(display2, last_points[index3], 3, color, -1);
	color = Scalar(0, 255, 255);
	cv::circle(display1, first_points[index4], 3, color, -1);
	cv::circle(display2, last_points[index4], 3, color, -1);

	// Undistort points
	cv::undistortPoints(motion1, undistort1, M, D);
	cv::undistortPoints(motion2, undistort2, M, D);

	// Find disparity matrix from stereo system
	cv::stereoRectify(M, D, M, D, image_size, R, T, R1, R2,
			P1, P2, Q);

	// Create disparity vector for transforming points
	vector<Point3d> disparity;
	for (unsigned i = 0; i < undistort1.size(); i++) {
		Point3d point;
		point.x = undistort1[i].x;
		point.y = undistort1[i].y;
		point.z = -sqrt(pow(undistort1[i].x-undistort2[i].x, 2.0) + 
		                pow(undistort1[i].y*undistort2[i].y, 2.0));
		//point.z = undistort1[i].x - undistort2[i].x;
		disparity.push_back(point);
	}

	// Transform points to get 3D information
	vector<Point3d> points_3d;
	cv::perspectiveTransform(disparity, points_3d, Q);

	// Scale points to real world measurements
	double scale = 0.011790085704189131;
	for (unsigned i = 0; i < points_3d.size(); i++) {
		points_3d[i].x *= scale;
		points_3d[i].y *= scale;
		points_3d[i].z *= scale;
	}

	//return output;
	return points_3d;
}

Mat Motion::getIntrinsic() { return M; }

Mat Motion::getHomography1() { return H1; }

Mat Motion::getHomography2() { return H2; }

Mat Motion::getRectification1() { return R1; }

Mat Motion::getRectification2() { return R2; }

Mat Motion::getFundamental() { return F; }

Mat Motion::getEssential() { return E; }

Mat Motion::getRotation() { return R; }

Mat Motion::getTranslation() { return T; }

void Motion::drawHorizontalLines(Mat& image)
{
	Point point1, point2;
	Scalar color(0, 0, 255);

	// For each line spacing draw line
	for (int i = 1; i < IMAGE_HEIGHT / LINE_SPACING; i++) {
		point1 = Point(0, i*LINE_SPACING);
		point2 = Point(IMAGE_WIDTH-1, i*LINE_SPACING);
		cv::line(image, point1, point2, color);
	}
}

/*void jeffCode()
{
	// perform stereo rectification virtually make both image planes the same
	// frame. Q is the 4x4 disparity-to-depth mapping matrix
	Mat P1, P2, Q;
	stereoRectify(  intrinsic, distortion,
									intrinsic, distortion,
									img_a.size(),  R,  t,
									R1, R2, P1, P2, Q  );

	// our points are already undistorted
	// populate vector of Point3f by cycling through each point
	std::vector<Point3f> points3d_a, points3d_b;
	for (int kk=0; kk < features_ua.size() ; kk++)
	{
		points3d_a.push_back(Point3f(features_ua[kk].x, features_ua[kk].y, features_ua[kk].x-features_ub[kk].x));
		points3d_b.push_back(Point3f(features_ub[kk].x, features_ub[kk].y, features_ua[kk].x-features_ub[kk].x));
	}

	// transform the points to calculate 3D information of 4 points
	perspectiveTransform(points3d_a, points3d_a, Q);
	perspectiveTransform(points3d_b, points3d_b, Q);

	// the points3d_a and points3d_b are the 3d estimates of the points
	// from the perspective of each camera, correlated with
	// features_keep_a and features_b
}*/
