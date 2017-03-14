# This is a makefile for testing OpenCV

TARGET = motion_field
SRC = src/
BIN = bin/
CCLASS = 
CLASS = 

CXX=g++
CFLAGS = -std=c++11 -Wall
LFLAGS = /usr/local/lib
LIBS = -lopencv_core -lopencv_imgcodecs -lopencv_videoio -lopencv_highgui -lopencv_imgproc -lopencv_calib3d -lopencv_video

all: $(SRC)$(TARGET).cpp $(CCLASS) $(HCLASS)
	$(CXX) $(CCLASS) $(SRC)$(TARGET).cpp $(CFLAGS) -o $(BIN)$(TARGET) -L$(LFLAGS) $(LIBS)

clean:
	rm -rf $(TARGET)
