# This is a makefile for testing OpenCV

TARGET = visiual_odometry
MAIN = main.cpp
SRC = src/
BIN = bin/
CCLASS = $(SRC)VisualOdometer.cpp $(SRC)MotionTracker.cpp
HCLASS = $(SRC)VisualOdometer.hpp $(SRC)MotionTracker.hpp

CXX=g++
CFLAGS = -std=c++11 -Wall
LFLAGS = /usr/local/lib
LIBS = -lopencv_core -lopencv_imgcodecs -lopencv_videoio -lopencv_highgui -lopencv_imgproc -lopencv_calib3d -lopencv_video -lopencv_features2d

all: $(SRC)$(MAIN) $(CCLASS) $(HCLASS)
	$(CXX) $(CCLASS) $(SRC)$(MAIN) $(CFLAGS) -o $(BIN)$(TARGET) -L$(LFLAGS) $(LIBS)

clean:
	rm -rf $(TARGET)
