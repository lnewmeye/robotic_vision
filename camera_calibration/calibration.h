#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

using std::vector;
using cv::Mat;
using cv::Size;
using cv::Point2f;
using cv::Point3f;

class Calibration
{
	public:
		// Constructur/Destructor
		Calibration(Size image_size, Size pattern_size);
		~Calibration() {}
		
		// Basic variable gets
		Mat getIntrinsic() {return intrinsic_params;}
		Mat getDistortion() {return distortion_params;}
		
		// Class operations
		void setParams(Mat intrinsic_params, Mat distortion_params);
		void addCalibrationImage(Mat image, Mat& display);
		void runCalibration();
		void undistortImage(Mat image_in, Mat& image_out);
		
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
};
