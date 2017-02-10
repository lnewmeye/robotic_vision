# Camera Calibration

This project is part of EcEn 631 Robotic Vision. It comprises all of Assignment 2 for the class. Due to some poor decisions as a designer it is a little complicated to manage the application's function. A small description follows

## Application Options

All the options can be controlled from the application parameters coded as #define values.

USE_DEFAULT_PARAMS: Sets whether the application should calibrate from some input or skip and use the DEFAULT_INTRINSIC and DEFAULT_DISTORTION parameters.
PRINT_PARAMS: This tells whether the parameters (once computed) should be output to the terminal
FIND_POSE: An additional step to compute the rotation and translation of an object from a dataset provided
DATA_SOURCE: Data can be pulled from images provided by Dr. Lee or from a personal webcam.
WINDOW_NAME: Pretty usless, just provides the label on the window display
DISPLAY_TIME: This one is more confusing. It sets the time between images being captured. If taking data from the images provided this can be set very low. If using a personal webcam, this should be set high so that time is give for the user to move the webcam and achieve a wide variety of images.

## Running the Application

The application can be built by a simple make command. To run the program, however, the activate.sh script should be executed. i.e.

	. activate.sh

This sets the environment varilables needed to find the OpenCV shared object libraries.

## Reports

The report written for class is contained in a markdown file anmed camera_calibration.md.


Have fun!

~Luke Newmeyer
