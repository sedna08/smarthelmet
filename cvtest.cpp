#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <time.h>
#include <stdlib.h>
#include <pigpio.h>
#include "cv.h"
#include <unistd.h>

using namespace cv;
using namespace std;

int prevVal = 0, currVal = 0;
int elapsedS = 0, elapsedMS = 0;
int startS = 0, startMS = 0;
int currentS = 0, currentMS = 0;
double timeMS = 0.0f;
int startIMUFlag = 0;

int main() {
	CV CV;
	if (gpioInitialise()<0) //initialises pigpio.h
	{
		//if pigpio initialisation failed
		cout<<"pigpio.h initialisation failed\n";
		return -1;
	}

    CascadeClassifier eyeCascade;
    if (!eyeCascade.load("/home/pi/Desktop/opencv-test/res/haarcascade/haarcascade_eye_tree_eyeglasses.xml")) {
        cerr << "Could not load eye detector." << endl;
        return 0;
    }

    VideoCapture cap(0);
    Mat frame;

    /* Detect Eyes */
    Rect eyeRIO = Rect(0, 0, 0, 0);
    while (eyeRIO.empty()) {
        cap.read(frame);
        //imshow("frame", frame);
        cout << "Detecting eyes..." << endl; 
        frame = CV.rotate(frame, 180);
        eyeRIO = CV.detectEyes(frame, eyeCascade);

        if (waitKey(1) == 27) {
            cout << "Program terminated." << endl;
            break;
        }
    }

	// getting start time
	gpioTime(0,&startS,&startMS);
	

    /* Detect Blinks */
    while (true) {
		
		// getting current time
		gpioTime(0,&currentS,&currentMS);
		// getting elapsed time from start to current
		elapsedS = currentS - startS;
		elapsedMS = currentMS - startMS;
		// getting integer elapsed Microsecond and converting it to float for an actual float value of microsecond
		timeMS = elapsedMS/1000000.0f;
		// getting elapsed millisecond from elapsed microsecond
		currVal = elapsedMS/1000;
		
		printf("Detecting Eye Detections");
		
		
        cap.read(frame);
        if (frame.empty()) break;
        if (eyeRIO.empty()) break;
        frame = CV.rotate(frame, 180);
        frame = frame(eyeRIO);
        CV.detectBlink(frame);
         printf("Time: %lf seconds ----  ", (double) ((double) elapsedS + timeMS));
         cout << "PERCLOS=" << CV.PERCLOS << endl;

        if (waitKey(1) == 27) {
            cout << "Program terminated." << endl;
            break;
        }
    }

    return 0;
}

