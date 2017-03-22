
void drawCorners(Mat& image, vector<Point2f> corners)
{
	// Iterate through corners and draw on display
	for (Point2f corner : corners) {
		cv::circle(image, corner, 4, Scalar(0,255,0), -1, 8, 0);
	}
}

void drawFlow(Mat& image, vector<Point2f> corners, vector<Point2f> flow,
		vector<unsigned char> status)
{
	// Create variables for loop
	Point2f corner, tip;

	// Iterate through vectors and plot if status shows flow to be found
	for (unsigned i = 0; i < status.size(); i++) {

		// Check if pattern found
		if (status[i]) {

			// Get corner and flow values
			corner = corners[i];
			tip = flow[i];

			// Draw circle and flow vector
			cv::circle(image, corner, 4, Scalar(0,255,0), -1, 8, 0);
			cv::arrowedLine(image, corner, tip, Scalar(0,0,255));
		}
	}
}

void drawFlow(Mat& image, vector<Point2f> corners, vector<Point2f> flow)
{
	// Create variables for loop
	Point2f corner, tip;

	// Iterate through vectors and plot if status shows flow to be found
	for (unsigned i = 0; i < corners.size(); i++) {

		// Get corner and flow values
		corner = corners[i];
		tip = flow[i];

		// Draw circle and flow vector
		cv::circle(image, corner, 4, Scalar(0,255,0), -1, 8, 0);
		cv::arrowedLine(image, corner, tip, Scalar(0,0,255));
	}
}

string generateFilename(string folder, string prefix, int number, string type)
{
        string value;

        // Adjust string value to have leading 0's
        if (number < 10) {
                value = "0" + std::to_string(number);
        }
        else {
                value = std::to_string(number);
        }

        // Return file path
        return folder + prefix + value + type;
}

vector<Point2f> opticalMatching(Mat& initial, Mat& image,
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

	return matches;
}


vector<Point2f> opticalMatchingFine(Mat& initial, Mat& image,
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
	cv::cornerSubPix(image, matches, subpix_size, zero_zone, subpix_criteria);

	return matches;
}

void trackMultiFrame(VideoCapture sequence)
{
	// Setup variables for process
	Mat image, image1, image2, display;
	vector<Point2f> corners, flow_corners;
	vector<vector<Point2f>> tracked_corners;
	vector<Mat> previous_images;
	vector<uchar> mask;
	uchar match;

	// Find 1000 points in initial image
	sequence >> image;
	cv::cvtColor(image, image1, CV_BGR2GRAY);
	cv::goodFeaturesToTrack(image1, corners, TRACK_CORNER_MAX,
			TRACK_CORNER_QUALITY, TRACK_CORNER_DISTANCE); 

	// Refine points
	Size subpix_size(5,5);
	Size zero_zone(-1,-1);
	cv::TermCriteria subpix_criteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 
			40, 0.001);
	cv::cornerSubPix(image1, corners, subpix_size, zero_zone,
			subpix_criteria);

	// Push corners to tracked vector and image
	tracked_corners.push_back(corners);
	previous_images.push_back(image1.clone());

	// Track points through frames 10 through 15
	for (int i = 10; i < 15; i++) {

		// Get new images
		image2 = image1.clone();
		sequence >> image;
		cv::cvtColor(image, image1, CV_BGR2GRAY);
		previous_images.push_back(image1.clone());

		// Match points between images
		flow_corners = opticalMatchingFine(image2, image1, corners);
		tracked_corners.push_back(flow_corners);
		cv::findFundamentalMat(corners, flow_corners, cv::FM_RANSAC,
				3, 0.99, mask);

		// Remove corners not matching between imges
		for (int j = mask.size(); j > 0; j--) {

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

		// Temporarily display image
		flow_corners = tracked_corners.back();
		corners = tracked_corners.at(tracked_corners.size()-2);
		//cv::cvtColor(image1, display, CV_GRAY2BGR);
		//drawFlow(display, corners, flow_corners);
		//cv::imshow(DISPLAY_WINDOW_1, display);
		//cv::waitKey(0);

		corners = flow_corners;
	}

	// Display image sequence
	for (unsigned i = 1; i < previous_images.size(); i++) {

		// Get corners and image to be displayed
		corners = tracked_corners[i-1];
		flow_corners = tracked_corners[i];
		cv::cvtColor(previous_images[i-1], display, CV_GRAY2BGR);

		// Draw image
		drawFlow(display, flow_corners, corners);
		//cv::imshow(DISPLAY_WINDOW_3, display);
		cv::imwrite("output/tmp/tmp" + 
				std::to_string(i) + ".jpg", display);
		cv::waitKey(DISPLAY_TIME_SLOW);
	}
}
