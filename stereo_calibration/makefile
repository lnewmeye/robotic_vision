# This is a makefile for testing OpenCV

TARGET = stereo_calibration

CXX=g++
CFLAGS = -std=c++11 -Wall
LFLAGS = /usr/local/lib
LIBS = -lopencv_core -lopencv_imgcodecs -lopencv_videoio -lopencv_highgui -lopencv_imgproc -lopencv_calib3d

all: $(TARGET).cpp calibration.h calibration.cpp
	$(CXX) calibration.cpp $(TARGET).cpp $(CFLAGS) -o $(TARGET) -L$(LFLAGS) $(LIBS)

clean:
	rm -rf $(TARGET)
