// EcEn 631 - Robotic Vision
// Visual Odometry - MotionTracker Class
// Luke Newmeyer

#include "MotionTracker.hpp"

using std::vector;
using cv::Size;
using cv::Mat;
using cv::Point2d;
using cv::Point2f;
using cv::KeyPoint;

// DEBUG define (comment out when not needed)
#define DEBUG
#ifdef DEBUG
#include <iostream>
#include <opencv2/highgui.hpp>
#define DEBUG_WINDOW_NAME "Debug Window"
#endif

MotionTracker::MotionTracker()
{
	SUBPIX_SIZE = Size(MOTION_SUBPIX_SIZE, MOTION_SUBPIX_SIZE);
	ZERO_ZONE = Size(MOTION_ZERO_ZONE, MOTION_ZERO_ZONE);
	SUBPIX_CRITERIA = cv::TermCriteria(CV_TERMCRIT_EPS +
			CV_TERMCRIT_ITER, 40, 0.001);
    FLOW_CRITERIA = cv::TermCriteria(cv::TermCriteria::COUNT |
			cv::TermCriteria::EPS, 20, 0.03);
	FLOW_SIZE = Size(31,31);

	// Special for fourth attempt
	detector = cv::ORB::create(MOTION_CORNER_QUANTITY);
	matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
}

void MotionTracker::setInitial(Mat image)
{
	// Set as previous image
	previous_image = image.clone();
	image_size = image.size();

	// Use Harris Corner (goodFeaturesToTrack) to find corners
	cv::goodFeaturesToTrack(image, previous_points, MOTION_CORNER_QUANTITY,
			MOTION_CORNER_QUALITY, MOTION_CORNER_DISTANCE);
	cv::cornerSubPix(image, previous_points, SUBPIX_SIZE, ZERO_ZONE,
			SUBPIX_CRITERIA);

	// Use ORB to find corners
	//vector<uchar> mask;
	//orb->detectAndCompute(image, mask, key_points, first_desc);
	//orb(image, mask, key_points, first_desc);
}

Mat MotionTracker::trackMotion(Mat image)
{
	// Using first attempt algorithm
	F = firstAttempt(image);
	//F = secondAttempt(image);
	//F = thirdAttempt(image);
	//F = fourthAttempt(image);
	//F = fifthAttempt(image);

	// Save image into previous image
	previous_image = image.clone();

	return F;
}

Mat MotionTracker::firstAttempt(Mat image)
{
	// Use Harris Corner (goodFeaturesToTrack) to find corners
	cv::goodFeaturesToTrack(image, previous_points, MOTION_CORNER_QUANTITY,
			MOTION_CORNER_QUALITY, MOTION_CORNER_DISTANCE);
	cv::cornerSubPix(image, previous_points, SUBPIX_SIZE, ZERO_ZONE,
			SUBPIX_CRITERIA);

	// Track Corners using PyrLK
	vector<unsigned char> status;
	vector<float> error;
	Size flow_size(31,31);
	cv::calcOpticalFlowPyrLK(previous_image, image, previous_points, 
			new_points, status, error, flow_size, OPTICAL_FLOW_LEVEL,
			FLOW_CRITERIA, 0, 0.001);

	// Find fundamental matrix
	vector<uchar> mask;
	F = cv::findFundamentalMat(previous_points, new_points, cv::FM_RANSAC,
			1, 0.70, mask);

	// Remove non-matching corners
	uchar match;
	for (unsigned i = mask.size(); i > 0; i--) {

		// Pop off from mask
		match = mask.back();
		mask.pop_back();

		// Remove item off of not a match
		if (!match) {
			previous_points.erase(previous_points.begin()+i-1);
			new_points.erase(new_points.begin()+i-1);
		}
	}

	return F;
}

Mat MotionTracker::secondAttempt(Mat image)
{
	// Track Corners using PyrLK
	vector<unsigned char> status;
	vector<float> error;
	Size flow_size(31,31);
	cv::calcOpticalFlowPyrLK(previous_image, image, previous_points, 
			new_points, status, error, flow_size, OPTICAL_FLOW_LEVEL,
			FLOW_CRITERIA, 0, 0.001);

	// Find fundamental matrix
	vector<uchar> mask;
	F = cv::findFundamentalMat(previous_points, new_points, cv::FM_RANSAC,
			1, 0.70, mask);

	// Remove non-matching corners
	uchar match;
	for (unsigned i = mask.size(); i > 0; i--) {

		// Pop off from mask
		match = mask.back();
		mask.pop_back();

		// Remove item off of not a match
		if (!match) {
			previous_points.erase(previous_points.begin()+i-1);
			new_points.erase(new_points.begin()+i-1);
		}
	}

	return F;
}

Mat MotionTracker::thirdAttempt(Mat image)
{
	// Remove old point track if above max frames
	if (point_track.size() >= POINT_TRACK_MAX_FRAMES) {
		point_track.pop_front();
	}


	// Find new set of points
	cv::goodFeaturesToTrack(previous_image, previous_points, 
			MOTION_CORNER_QUANTITY, MOTION_CORNER_QUALITY, 
			MOTION_CORNER_DISTANCE);
	cv::cornerSubPix(previous_image, previous_points, SUBPIX_SIZE, ZERO_ZONE,
			SUBPIX_CRITERIA);

	// Push new points
	vector<vector<Point2f>> new_track;
	new_track.push_back(previous_points);
	point_track.push_back(new_track);

	// Track new point set in each new frame
	for (unsigned i = 0; i < point_track.size(); i++) {
	//for (vector<vector<Point2f>> track : point_track) {

		// Track points in next image and remove outliers
		previous_points = point_track[i].back();
		vector<unsigned char> status;
		vector<float> error;
		cv::calcOpticalFlowPyrLK(previous_image, image, previous_points, 
				new_points, status, error, FLOW_SIZE, OPTICAL_FLOW_LEVEL,
				FLOW_CRITERIA, 0, 0.001);

		// Remove outliers through fundamental matrix
		vector<uchar> mask;
		F = cv::findFundamentalMat(previous_points, new_points, cv::FM_RANSAC,
				1, 0.70, mask);
		point_track[i].push_back(new_points);

		// Remove non-matching corners
		uchar match;
		for (unsigned j = mask.size(); j > 0; j--) {

			// Pop off from mask
			match = mask.back();
			mask.pop_back();

			// Remove item off of not a match
			if (!match) {

				// Iterate through track and remove from each set
				for (unsigned k = 0; k < point_track[i].size(); k++) {
					point_track[i][k].erase(point_track[i][k].begin()+j-1);
				}
			}
		}
	}

	// Find fundamental matrix for most recent points
	vector<vector<Point2f>> old_track = point_track.front();
	new_points = old_track.back();
	previous_points = old_track.at(old_track.size() - 2); // TODO: Check this is right
	F = cv::findFundamentalMat(previous_points, new_points, cv::FM_RANSAC,
			1, 0.70);

	// Save new image as previous for next iteration
	previous_image = image.clone();

	return Mat::eye(3, 3, CV_64F);
}

Mat MotionTracker::fourthAttempt(Mat image)
{
	vector<cv::KeyPoint> previous_keypoints;
	detector->detect(previous_image, previous_keypoints);
	//detector->convert(previous_keypoints, previus_points);
	cv::KeyPoint::convert(previous_keypoints, previous_points);

	// Track Corners using PyrLK
	vector<unsigned char> status;
	vector<float> error;
	Size flow_size(31,31);
	cv::calcOpticalFlowPyrLK(previous_image, image, previous_points, 
			new_points, status, error, flow_size, OPTICAL_FLOW_LEVEL,
			FLOW_CRITERIA, 0, 0.001);

	// Find fundamental matrix
	vector<uchar> mask;
	F = cv::findFundamentalMat(previous_points, new_points, cv::FM_RANSAC,
			1, 0.70, mask);

	// Remove non-matching corners
	uchar match;
	for (unsigned i = mask.size(); i > 0; i--) {

		// Pop off from mask
		match = mask.back();
		mask.pop_back();

		// Remove item off of not a match
		if (!match) {
			previous_points.erase(previous_points.begin()+i-1);
			new_points.erase(new_points.begin()+i-1);
		}
	}

	return F;
}

Mat MotionTracker::fifthAttempt(cv::Mat image)
{
	// Detect previous features with ORB detector
	Mat previous_descriptors;
	vector<KeyPoint> previous_keypoints;
	detector->detectAndCompute(previous_image, cv::noArray(), 
			previous_keypoints, previous_descriptors);

	// Detect features with ORB detector
    Mat new_descriptors;
	vector<KeyPoint> new_keypoints;
	//detector->detect(previous_image, previous_keypoints);
	detector->detectAndCompute(image, cv::noArray(), 
			new_keypoints, new_descriptors);

	// Match features with Brute-Force Hamming
	vector<vector<cv::DMatch>> matches;
	vector<KeyPoint> matched1, matched2;
	matcher->knnMatch(previous_descriptors, new_descriptors, matches, 2);

	double nn_match_ratio = 0.8;
	for(unsigned i = 0; i < matches.size(); i++) {
		if(matches[i][0].distance < nn_match_ratio * matches[i][1].distance) {
			matched1.push_back(previous_keypoints[matches[i][0].queryIdx]);
			matched2.push_back(     new_keypoints[matches[i][0].trainIdx]);
		}
	}

	// Convert keypoint matches to Point2f
	cv::KeyPoint::convert(matched1, previous_points);
	cv::KeyPoint::convert(matched2, new_points);

	// Find fundamental matrix
	vector<uchar> mask;
	F = cv::findFundamentalMat(previous_points, new_points, cv::FM_RANSAC,
			1, 0.70, mask);

	// Remove outliers
	uchar match;
	for (unsigned i = mask.size(); i > 0; i--) {

		// Pop off from mask
		match = mask.back();
		mask.pop_back();

		// Remove item off of not a match
		if (!match) {
			previous_points.erase(previous_points.begin()+i-1);
			new_points.erase(new_points.begin()+i-1);
		}
	}

	return F;
}

vector<Point2f> MotionTracker::getPreviousPoints()
{
	return previous_points;
}

vector<Point2f> MotionTracker::getNewPoints()
{
	return new_points;
}
