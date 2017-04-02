// EcEn 631 - Robotic Vision
// Structure from Motion - Motion Class
// Luke Newmeyer

#include "Motion.h"

// DEBUG define (comment out when not needed)
#define DEBUG
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
	F = cv::findFundamentalMat(corners, flow_corners, cv::FM_RANSAC,
			3, 0.99, mask);

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
		if (tracked_corners.back()[j].x > 590) {
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
	averageIntrinsic();

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
	vector<uchar> mask;
	vector<Point2f> corners = tracked_corners.front();
	vector<Point2f> flow_corners = tracked_corners.back();
	F = cv::findFundamentalMat(corners, flow_corners, cv::FM_RANSAC,
			3, 0.99, mask);
	D = (cv::Mat_<double>(5,1) << IMAGE_DISTORTION_KNOWN);
	M = (cv::Mat_<double>(3,3) << IMAGE_INTRINSIC_KNOWN);

	// Compute essential matrix
	Mat Mt;
	cv::transpose(M, Mt);
	E = Mt * F * M;

	// Perform SVD on essential matrix
	Mat W, U, V, Vt;
	cv::SVD::compute(E, W, U, Vt);
	cv::transpose(Vt, V);

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
	index1 = 42; index2 = 1; index3 = 2; index4 = 0;
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
	Scalar color(0, 0, 255);
	cv::cvtColor(previous_images.front(), display1, cv::COLOR_GRAY2BGR);
	cv::cvtColor(previous_images.back(), display2, cv::COLOR_GRAY2BGR);
	cv::circle(display1, first_points[index1], 3, color, -1);
	cv::circle(display2, last_points[index1], 3, color, -1);
	cv::circle(display1, first_points[index2], 3, color, -1);
	cv::circle(display2, last_points[index2], 3, color, -1);
	cv::circle(display1, first_points[index3], 3, color, -1);
	cv::circle(display2, last_points[index3], 3, color, -1);
	cv::circle(display1, first_points[index4], 3, color, -1);
	cv::circle(display2, last_points[index4], 3, color, -1);

	// Undistort points
	cv::undistortPoints(motion1, undistort1, M, D);
	cv::undistortPoints(motion2, undistort2, M, D);

	// Construct homography matrix
	//H = cv::findHomography(undistort1, undistort2);
	//std::cout << "H =" << std::endl << H << std::endl;
	//H = cv::Mat_<double>(4,4);
	//H.at<double>(0,0) = R.at<double>(0,0);
	//H.at<double>(0,1) = R.at<double>(0,1);
	//H.at<double>(0,2) = R.at<double>(0,2);
	//H.at<double>(1,0) = R.at<double>(1,0);
	//H.at<double>(1,1) = R.at<double>(1,1);
	//H.at<double>(1,2) = R.at<double>(1,2);
	//H.at<double>(2,0) = R.at<double>(2,0);
	//H.at<double>(2,1) = R.at<double>(2,1);
	//H.at<double>(2,2) = R.at<double>(2,2);
	//H.at<double>(0,3) = T.at<double>(0,0);
	//H.at<double>(1,3) = T.at<double>(1,0);
	//H.at<double>(2,3) = T.at<double>(2,0);
	//H.at<double>(3,0) = 0;
	//H.at<double>(3,1) = 0;
	//H.at<double>(3,2) = 0;
	//H.at<double>(3,3) = 1;

	//R1 = cv::Mat_<double>(3,4);
	//R1.at<double>(0,0) = R.at<double>(0,0);
	//R1.at<double>(0,1) = R.at<double>(0,1);
	//R1.at<double>(0,2) = R.at<double>(0,2);
	//R1.at<double>(1,0) = R.at<double>(1,0);
	//R1.at<double>(1,1) = R.at<double>(1,1);
	//R1.at<double>(1,2) = R.at<double>(1,2);
	//R1.at<double>(2,0) = R.at<double>(2,0);
	//R1.at<double>(2,1) = R.at<double>(2,1);
	//R1.at<double>(2,2) = R.at<double>(2,2);
	//R1.at<double>(3,0) = T.at<double>(0,0);
	//R1.at<double>(3,1) = T.at<double>(0,1);
	//R1.at<double>(3,2) = T.at<double>(0,2);
	//R1 = M*R1;

	//R2 = cv::Mat_<double>(3,4);
	//R2.at<double>(0,0) = 1;
	//R2.at<double>(0,1) = 0;
	//R2.at<double>(0,2) = 0;
	//R2.at<double>(1,0) = 0;
	//R2.at<double>(1,1) = 1;
	//R2.at<double>(1,2) = 0;
	//R2.at<double>(2,0) = 0;
	//R2.at<double>(2,1) = 0;
	//R2.at<double>(2,2) = 1;
	//R2.at<double>(3,0) = 0;
	//R2.at<double>(3,1) = 0;
	//R2.at<double>(3,2) = 0;
	//R2 = M*R2;

	// Find disparity of points
	//vector<Point3f> disparity;
	//Point3f point;
	//Mat triangle1(1,4,CV_64FC2);
	//Mat triangle2(1,4,CV_64FC2);
	//for (unsigned i = 0; i < undistort1.size(); i++) {
		//triangle1.at<Vec2d>(0,i)[0] = undistort1[i].x;
		//triangle1.at<Vec2d>(0,i)[1] = undistort1[i].y;
		//triangle2.at<Vec2d>(0,i)[0] = undistort2[i].x;
		//triangle2.at<Vec2d>(0,i)[1] = undistort2[i].y;
		//Vec2d row = triangle1.at<Vec2d>(0,i);
		//row[0] = undistort1[i].x;
		//row[1] = undistort1[i].y;
		//row = triangle2.at<Vec2d>(0,i);
		//row[0] = undistort2[i].x;
		//row[1] = undistort2[i].y;
		//Mat triangle1.at<double>(i,0) = undistort1[i].x;
		//Mat triangle1.at<double>(i,1) = undistort1[i].x;
		//point.x = undistort1[i].x;
		//point.y = undistort1[i].y;
		//point.z = cv::norm(Mat(undistort1[i]), Mat(undistort2[i]));
		//disparity.push_back(point);
	//}

	// Transform points
	//vector<Point3f> points_3d;
	//Mat points_3d(1,4,CV_64FC4);
	//cv::perspectiveTransform(disparity, points_3d, H);
	//cv::triangulatePoints(R1, R2, triangle1, triangle2, points_3d);

	// Scale points
	//double scale = 1.0;
	//points_3d[0] *= scale;
	//points_3d[1] *= scale;
	//points_3d[2] *= scale;
	//points_3d[3] *= scale;

	//vector<Point3f> output;
	//for (int i = 0; i < points_3d.rows; i++) {
		//Vec4d row = points_3d.at<Vec4d>(0,i);
		//Point3f point;
		//point.x = row[0];
		//point.y = row[1];
		//point.z = row[2];
		//output.push_back(point*scale);
	//}

	// Create variables needed to solve linear set of equations
	Mat A, b, x;
	Mat b1, b2, b3;
	Mat r1, r2, r3;
	Mat c1, c2, c3;
	double f;
	Point2f point_l, point_r;
	Point3d point_3d;
	vector<Point3d> points_3d;
	double xr, yr, xl, yl;

	double scale = -38.350840455358046;

	for (unsigned i = 0; i < undistort1.size(); i++) {

		// Select point to be solved
		point_l = undistort1[i];
		point_r = undistort2[i];

		// Set values for scalars
		f = M.at<double>(0,0);
		xl = point_l.x;
		yl = point_l.y;
		xr = point_r.x;
		yr = point_r.y;

		// Set values for columns of R
		r1 = cv::Mat_<double>(1,3);
		r1.at<double>(0,0) = R.at<double>(0,0);
		r1.at<double>(0,1) = R.at<double>(0,1);
		r1.at<double>(0,2) = R.at<double>(0,2);
		r2 = cv::Mat_<double>(1,3);
		r2.at<double>(0,0) = R.at<double>(1,0);
		r2.at<double>(0,1) = R.at<double>(1,1);
		r2.at<double>(0,2) = R.at<double>(1,2);
		r3 = cv::Mat_<double>(1,3);
		r3.at<double>(0,0) = R.at<double>(2,0);
		r3.at<double>(0,1) = R.at<double>(2,1);
		r3.at<double>(0,2) = R.at<double>(2,2);

		// Setup matrix A for solution
		c1 = (xr * r3) - (f * r1);
		c2 = (yr * r2) - (f * r1);
		c3 = cv::Mat_<double>(1,3);
		c3.at<double>(0,0) = -f;
		c3.at<double>(0,1) = -f;
		c3.at<double>(0,2) = xl+yl;
		A = cv::Mat_<double>(3,3);
		A.at<double>(0,0) = c1.at<double>(0,0);
		A.at<double>(0,1) = c1.at<double>(0,1);
		A.at<double>(0,2) = c1.at<double>(0,2);
		A.at<double>(1,0) = c2.at<double>(0,0);
		A.at<double>(1,1) = c2.at<double>(0,1);
		A.at<double>(1,2) = c2.at<double>(0,2);
		A.at<double>(2,0) = c3.at<double>(0,0);
		A.at<double>(2,1) = c3.at<double>(0,1);
		A.at<double>(2,2) = c3.at<double>(0,2);

		std::cout << "A =" << std::endl << A << std::endl;

		// Setup matrix b for solutin
		b1 = cv::Mat_<double>(1,1);
		b1 = ((xr * r3) - (f * r1)) * T;
		b2 = cv::Mat_<double>(1,1);
		b2 = ((yr * r2) - (f * r1)) * T;
		b3 = cv::Mat_<double>(1,1);
		b3.at<double>(0,0) = 0;
		b = cv::Mat_<double>(3,1);
		b.at<double>(0,0) = b1.at<double>(0,0);
		b.at<double>(1,0) = b2.at<double>(0,0);
		b.at<double>(2,0) = b3.at<double>(0,0);

		std::cout << "b =" << std::endl << b << std::endl;

		// Solve for point
		cv::solve(A, b, x);
		point_3d.x = x.at<double>(0,0) * scale;
		point_3d.y = x.at<double>(0,1) * scale;
		point_3d.z = x.at<double>(0,2) * scale;
		points_3d.push_back(point_3d);

		std::cout << "x =" << std::endl << x << std::endl;
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

void jeffCode()
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
}
