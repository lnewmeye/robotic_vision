#include "calibration.h"

Calibration::Calibration(Size image_size, Size pattern_size)
{
	this->image_size = image_size;
	this->pattern_size = pattern_size;
}

void Calibration::setParams(Mat intrinsic_params, Mat distortion_params)
{
	this->intrinsic_params = intrinsic_params.clone();
	this->distortion_params = distortion_params.clone();
}

void Calibration::addCalibrationImage(Mat image, Mat& display)
{
	// Create color version of input image
	cv::cvtColor(image, display, CV_GRAY2RGB);
	
	// Create variables
	vector<Point2f> corners;
	Size search_size = Size(5,5);
	Size zero_zone = Size(-1,-1);
	cv::TermCriteria criteria = cv::TermCriteria( CV_TERMCRIT_EPS + 
		CV_TERMCRIT_ITER, 40, 0.001);
	bool pattern_found;
	
	// Find chessboard corners, refine corners, and draw
	pattern_found = cv::findChessboardCorners(image, pattern_size, corners);
	cv::cornerSubPix(image, corners, search_size, zero_zone, criteria);
	cv::drawChessboardCorners(display, pattern_size, corners, pattern_found);
	
	// Push corners to data structure
	calibration_points.push_back(corners);
}

void Calibration::runCalibration()
{
	// Create vector of vector of point indicies
	vector<vector<Point3f>> indicies;
	for (unsigned i = 0; i < calibration_points.size(); i++) {
		
		// Create vector of points
		vector<Point3f> index;
		
		// Loop through rows and columns and add points
		for (int j = 0; j < pattern_size.height; j++) {
			for (int k = 0; k < pattern_size.width; k++) {
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
	
	// Create data structures for unused data
	vector<Mat> rvecs, tvecs;
	
	// Run camera calibration
	rms = cv::calibrateCamera(indicies, calibration_points, image_size, 
		intrinsic_params, distortion_params, rvecs, tvecs);
}

void Calibration::undistortImage(Mat image_in, Mat& image_out)
{
	cv::undistort(image_in, image_out, intrinsic_params, distortion_params);
}
