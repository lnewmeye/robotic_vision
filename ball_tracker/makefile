# This is a makefile for testing OpenCV

TARGET = ball_tracker
SRC = src/
BIN = bin/
CCLASS = $(SRC)calibration.cpp $(SRC)stereo.cpp $(SRC)ball.cpp
HCLASS = $(SRC)calibration.h $(SRC)stereo.h $(SRC)ball.h

CXX=g++
CFLAGS = -std=c++11 -Wall
LFLAGS = /usr/local/lib
LIBS = -lopencv_core -lopencv_imgcodecs -lopencv_videoio -lopencv_highgui -lopencv_imgproc -lopencv_calib3d

all: $(SRC)$(TARGET).cpp $(CCLASS) $(HCLASS)
	$(CXX) $(CCLASS) $(SRC)$(TARGET).cpp $(CFLAGS) -o $(BIN)$(TARGET) -L$(LFLAGS) $(LIBS)

clean:
	rm -rf $(TARGET)
