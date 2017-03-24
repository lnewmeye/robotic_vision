
void drawCorners(Mat& image, vector<Point2f> corners)
{
	// Loop through and draw circles on corners of output
	for(Point2f corner : corners) {
		cv::circle(image, corner, 4, Scalar(255,0,0), -1, 8, 0);
	}
}


void printPointData(vector<Point3f> corners)
{
	for(Point3f corner : corners) {
		cout << corner << endl;
	}
}

void printPointData2(vector<Point2f> corners)
{
	for(Point2f corner : corners) {
		cout << corner << endl;
	}
}
