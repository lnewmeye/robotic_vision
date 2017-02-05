# This is a makefile for testing OpenCV

TARGET = camera_calibration

CXX=g++
CFLAGS = -std=c++11
LFLAGS = /usr/local/lib
LIBS = -lopencv_core -lopencv_imgcodecs -lopencv_videoio -lopencv_highgui -lopencv_imgproc -lopencv_calib3d

all: $(TARGET).cpp
	$(CXX) $(TARGET).cpp $(CFLAGS) -o $(TARGET) -L$(LFLAGS) $(LIBS)

clean:
	rm -rf $(TARGET)
