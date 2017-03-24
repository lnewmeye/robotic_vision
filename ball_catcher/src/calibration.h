#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

using std::vector;
using cv::Mat;
using cv::Size;
using cv::Point2f;
using cv::Point3f;

#define SQUARE_SIZE 3.88

class Calibration
{
	public:
		// Constructur/Destructor
		Calibration() {}
		~Calibration() {}

		// Set parameters
		void setSize(Size image_size, Size pattern_size);
		
		// Basic variable gets
		Mat getIntrinsic() {return intrinsic_params;}
		Mat getDistortion() {return distortion_params;}
		int getRMS() {return rms;}
		vector<vector<Point2f>> getCalibrationPoints() {return calibration_points;}
		vector<vector<Point3f>> getObjectPoints() {return object_points;}
		
		// Class operations
		void setCalibration(Mat intrinsic_params, Mat distortion_params);
		void addCalibrationImage(Mat image, Mat& display);
		void runCalibration();
		void undistortImage(Mat image_in, Mat& image_out);
		void estimatePose(vector<Point3f> object_points, 
			vector<Point2f> image_points, Mat& rotation, Mat& translation);
		
	private:
		// Image parameters
		Size image_size;
		Size pattern_size;
		
		// Calibration parameters
		Mat intrinsic_params = Mat::eye(3, 3, CV_64F);
		Mat distortion_params = Mat::zeros(5, 1, CV_64F);
		int rms = -1;
		
		// Calibration data
		vector<vector<Point2f>> calibration_points;
		vector<vector<Point3f>> object_points;
};
