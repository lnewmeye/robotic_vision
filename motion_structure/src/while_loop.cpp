
	// Create variables for optical flow loop
	vector<Point2f> corners;
	Mat display, display1, display2, display3;
	vector<Point2f> flow1, flow2, flow3;
	vector<unsigned char> status1, status2, status3;
	vector<float> error;
	cv::TermCriteria subpix_criteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 
			40, 0.001);
    cv::TermCriteria flow_criteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,
			20, 0.03);
	Size subpix_size(5,5);
	Size zero_zone(-1,-1);
	Size flow_size(31,31);
	int image_number = 0;
	string image_name;
	int keypress = 0;

	while(image1.data && keypress != 'q') {

		// Get features to track in initial image
		cv::goodFeaturesToTrack(initial, corners, OPTICAL_CORNER_MAX,
				OPTICAL_CORNER_QUALITY, OPTICAL_CORNER_DISTANCE);
		cv::cornerSubPix(initial, corners, subpix_size, zero_zone, 
				subpix_criteria);

		// Compute optical flow on images 1, 2, and 3
		cv::calcOpticalFlowPyrLK(initial, image1, corners, flow1,
				status1, error, flow_size, OPTICAL_FLOW_LEVEL,
				flow_criteria, 0, 0.001);
		if(image2.data) {
			cv::calcOpticalFlowPyrLK(initial, image2, corners, flow2,
					status2, error, flow_size, OPTICAL_FLOW_LEVEL,
					flow_criteria, 0, 0.001);
		}
		if(image3.data) {
			cv::calcOpticalFlowPyrLK(initial, image3, corners, flow3,
					status3, error, flow_size, OPTICAL_FLOW_LEVEL,
					flow_criteria, 0, 0.001);
		}

		// Draw flow on image
		cv::cvtColor(image1, display1, CV_GRAY2BGR);
		drawFlow(display1, corners, flow1, status1);
		if(image2.data) {
			cv::cvtColor(image2, display2, CV_GRAY2BGR);
			drawFlow(display2, corners, flow2, status2);
		}
		if(image3.data) {
			cv::cvtColor(image3, display3, CV_GRAY2BGR);
			drawFlow(display3, corners, flow3, status3);
		}

		// Display image and wait keypress
		cv::imshow(DISPLAY_WINDOW_1, display1);
		if(image2.data)
			cv::imshow(DISPLAY_WINDOW_2, display2);
		if(image3.data)
			cv::imshow(DISPLAY_WINDOW_3, display3);
		keypress = cv::waitKey(DISPLAY_TIME_FAST);

		// Write images to file
		image_name = generateFilename(IMAGE_OUTPUT_FOLDER,
				IMAGE_OUTPUT_FLOW1, image_number, IMAGE_OUTPUT_TYPE);
		cv::imwrite(image_name, display1);
		if(image2.data) {
			image_name = generateFilename(IMAGE_OUTPUT_FOLDER,
					IMAGE_OUTPUT_FLOW2, image_number, IMAGE_OUTPUT_TYPE);
			cv::imwrite(image_name, display2);
		}
		if(image3.data) {
			image_name = generateFilename(IMAGE_OUTPUT_FOLDER,
					IMAGE_OUTPUT_FLOW3, image_number, IMAGE_OUTPUT_TYPE);
			cv::imwrite(image_name, display3);
		}
		image_number++;

		// Swap images and load new image
		flow_images >> image;
		initial = image1.clone();
		image1 = image2.clone();
		image2 = image3.clone();
		cv::cvtColor(image, image3, CV_BGR2GRAY);
	}

	// Create VideoCaputre to read in Template Match Images
	VideoCapture match_images(IMAGE_OPTICAL_FLOW);
	if (!match_images.isOpened()) {
		cout << "Error: couldn't open template match images" << endl;
		return -1;
	}

	// Read in first set of images
	match_images >> initial;
	match_images >> image1;
	match_images >> image2;
	match_images >> image3;

	// Convert initial images to grayscale
	cv::cvtColor(initial, initial, CV_BGR2GRAY);
	cv::cvtColor(image1, image1, CV_BGR2GRAY);
	cv::cvtColor(image2, image2, CV_BGR2GRAY);
	cv::cvtColor(image3, image3, CV_BGR2GRAY);

	// Create/reset variables for loop
	image_number = 0;
	keypress = 0;

	// Compute optical flow using matching
	while(image1.data && keypress != 'q') {

		// Get features to track in initial image
		cv::goodFeaturesToTrack(initial, corners, OPTICAL_CORNER_MAX,
				OPTICAL_CORNER_QUALITY, OPTICAL_CORNER_DISTANCE);
		cv::cornerSubPix(initial, corners, subpix_size, zero_zone, 
				subpix_criteria);

		// Compute optical flow on images 1, 2, and 3
		flow1 = opticalMatching(initial, image1, corners);
		if(image2.data)
			flow2 = opticalMatching(initial, image1, corners);
		if(image3.data)
			flow3 = opticalMatching(initial, image3, corners);

		// Draw flow on image
		cv::cvtColor(image1, display1, CV_GRAY2BGR);
		drawFlow(display1, corners, flow1, status1);
		if(image2.data) {
			cv::cvtColor(image2, display2, CV_GRAY2BGR);
			drawFlow(display2, corners, flow2, status2);
		}
		if(image3.data) {
			cv::cvtColor(image3, display3, CV_GRAY2BGR);
			drawFlow(display3, corners, flow3, status3);
		}

		// Display image and wait keypress
		cv::imshow(DISPLAY_WINDOW_1, display1);
		if(image2.data)
			cv::imshow(DISPLAY_WINDOW_2, display2);
		if(image3.data)
			cv::imshow(DISPLAY_WINDOW_3, display3);
		keypress = cv::waitKey(DISPLAY_TIME_FAST);

		// Write images to file
		image_name = generateFilename(IMAGE_OUTPUT_FOLDER,
				IMAGE_OUTPUT_MATCH1, image_number, IMAGE_OUTPUT_TYPE);
		cv::imwrite(image_name, display1);
		if(image2.data) {
			image_name = generateFilename(IMAGE_OUTPUT_FOLDER,
					IMAGE_OUTPUT_MATCH2, image_number, IMAGE_OUTPUT_TYPE);
			cv::imwrite(image_name, display2);
		}
		if(image3.data) {
			image_name = generateFilename(IMAGE_OUTPUT_FOLDER,
					IMAGE_OUTPUT_MATCH3, image_number, IMAGE_OUTPUT_TYPE);
			cv::imwrite(image_name, display3);
		}
		image_number++;

		// Swap images and load new image
		match_images >> image;
		initial = image1.clone();
		image1 = image2.clone();
		image2 = image3.clone();
		cv::cvtColor(image, image3, CV_BGR2GRAY);
	}

	// Create VideoCaputre to read image tracking data (1)
	VideoCapture image_sequence(IMAGE_TRACKING_1);
	if (!image_sequence.isOpened()) {
		cout << "Error: couldn't open image sequence" << endl;
		return -1;
	}

	trackMultiFrame(image_sequence);

	// Open next image tracking sequence
	image_sequence.open(IMAGE_TRACKING_2);
	if (!image_sequence.isOpened()) {
		cout << "Error: couldn't open image sequence" << endl;
		return -1;
	}

	trackMultiFrame(image_sequence);

	// Open next image tracking sequence
	image_sequence.open(IMAGE_TRACKING_3);
	if (!image_sequence.isOpened()) {
		cout << "Error: couldn't open image sequence" << endl;
		return -1;
	}

	trackMultiFrame(image_sequence);

	// Open next image tracking sequence
	image_sequence.open(IMAGE_TRACKING_4);
	if (!image_sequence.isOpened()) {
		cout << "Error: couldn't open image sequence" << endl;
		return -1;
	}

	trackMultiFrame(image_sequence);


