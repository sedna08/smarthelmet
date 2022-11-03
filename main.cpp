#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "eeg.h"
#include "Parser_Filter.h"
#include <string.h>
#include <string>
#include <fstream>
#include "imu.h"
#include "kalmanFilter.h"
#include <errno.h>
#include <csignal>
#include <unistd.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>
#include <pigpio.h>
#include <iostream>
#include "cv.h"

using namespace cv;
using namespace std;

EEG EEG;
IMU IMU;
CV CV;
FILE* Data_text;
FILE* data;
extern double dataArray[15];
int prevVal = 0, currVal = 0;
int elapsedS = 0, elapsedMS = 0;
int startS = 0, startMS = 0;
int currentS = 0, currentMS = 0;
double timeMS = 0.0f;
int startIMUFlag = 0;
int startCVFlag = 0;

void signalHandler (int signum) {
    cout << "\nInterrupt Handling #" << signum << endl;
	serClose(EEG.serial_port);
    free(EEG.queue);
    free(EEG.queue1);
    fclose(data);
	// turning back gpio 4 and 17 to input
	gpioWrite(17,0);
	gpioWrite(4,0);
	gpioSetMode(4,PI_INPUT);
	gpioSetMode(17,PI_INPUT);
	gpioTerminate();
	fclose(Data_text);

    // cleanup and close up stuff here  
    // terminate program  

    exit(signum);
}


int main( int argc, char **argv ) {
	
	long double fittedRoll;
	long double fittedPitch;
	
	if (gpioInitialise()<0) //initialises pigpio.h
    {
        //if pigpio initialisation failed
        cout<<"pigpio.h initialisation failed\n";
        return -1;
    }

    // register SIGINT and signal handler
    signal(SIGINT, signalHandler); 

	// gpio 4 is set to output for LED indicator for poor signal quality
	gpioSetMode(4,PI_OUTPUT);
	gpioSetMode(4,PI_OUTPUT);

	
	CascadeClassifier eyeCascade;
    if (!eyeCascade.load("haarcascade_eye_tree_eyeglasses.xml")) {
        cout << "xml not found! Check path and try again." << endl;
        return 0;
    }

    VideoCapture cap(0); // cap(0) (webcam), cap(1) (video)
    Mat frame;

    // Detect Eyes
    Rect eyeRIO = Rect(0, 0, 0, 0); // eye region of interest (ROI)
    while (eyeRIO.empty()) {
        cap.read(frame);
        cout << "Detecting eyes..." << endl; 
        if (frame.empty()) break;
        frame = CV.rotate(frame, 180);
	imshow("frame", frame);
	if (waitKey(1) == 27) {
		cout << "Program terminated" << endl;
		break;
	}
        eyeRIO = CV.detectEyes(frame, eyeCascade);
    }


	// Initialize 1st IMU
    printf("Initializing IMU MPU6050\n");
    initDevice(&IMU, 1);

    printf("Initializing NeuroSky\n");
    initEEG(&EEG);

	unsigned char streamByte;
    Data_text = fopen("Text_Data.txt","w");
	
	data = fopen("data.bin","wb");
	
	dataArray[0]=0;		// index 0 for time
	dataArray[1]=0;		// index 1 for EEG Raw Values
	dataArray[2]=0;		// index 2 for EEG Values with MAV Filter
	dataArray[3]=0;		// index 3 for Delta Band
	dataArray[4]=0;		// index 4 for Theta Band
	dataArray[5]=0;		// index 5 for Low-alpha Band
	dataArray[6]=0;		// index 6 for High-alpha Band
	dataArray[7]=0;		// index 7 for Low-beta Band
	dataArray[8]=0;		// index 8 for High-beta Band
	dataArray[9]=0;		// index 9 for Low-gamma Band
	dataArray[10]=0;	// index 10 for Mid-gamma Band
	dataArray[11]=0;	// index 11 for Poor-Signal Quality
	dataArray[12]=0;	// index 12 for fitterRoll IMU values
	dataArray[13]=0;	// index 13 for fittedPitch IMU values
	dataArray[14]=0;	// index 14 for PERCLOS

	
	// getting start time
	gpioTime(0,&startS,&startMS);

	while(1){
		// getting current time
		gpioTime(0,&currentS,&currentMS);
		// getting elapsed time from start to current
		elapsedS = currentS - startS;
		elapsedMS = currentMS - startMS;
		// getting integer elapsed Microsecond and converting it to float for an actual float value of microsecond
		timeMS = (double) elapsedMS/1000000.0f;
		// getting elapsed millisecond from elapsed microsecond
		currVal = elapsedMS/1000;
		
		
		if(serDataAvailable (EEG.serial_port)){
			// printf("new ser data avail: %lf\n",(double) ((double) elapsedS + timeMS) );
			streamByte = eegRead(&EEG);
			// printing streamByte for debugging purposes
			// printf("\n streamByte: %d", streamByte);
			fflush(stdout);
			//startIMUFlag = 1;
		}
		
		else if(startCVFlag == 0 && startIMUFlag != 0) {
			readIMU(&IMU,1);
			dataArray[0] = (double) ((double) elapsedS + timeMS);

			// calculations for fitted Roll and fitted Pitch
			fittedRoll = (-0.000005857680127)*(powf(IMU.EulerAngles.Roll, 3)) + (-0.000059321347234)*(powf(IMU.EulerAngles.Roll, 2)) + 1.097026959773455*IMU.EulerAngles.Roll +  0.301807912947446;
			fittedPitch = (-0.000005959003219) *(powf(IMU.EulerAngles.Pitch, 3)) + 0.000202569646997*(powf(IMU.EulerAngles.Pitch, 2)) + 1.099149655304190*IMU.EulerAngles.Pitch +  (-0.953976026366887);

			// updating and writing data to bin file
			dataArray[12]=(double)fittedRoll;
			dataArray[13]=(double)fittedPitch;
			fwrite(dataArray,sizeof(double),15,data);
			startCVFlag = 1;
			startIMUFlag = 0;
		}
		else if(startCVFlag != 0) {
			// printf("Detecting Eye\n");
			cap.read(frame);
			if (frame.empty()) break;
			if (eyeRIO.empty()) break;
			frame = CV.rotate(frame, 180);
			frame = frame(eyeRIO);
			CV.detectBlink(frame);
			// cout << "Time: " << (double) ((double) elapsedS + timeMS) << " ------ PERCLOS = " << CV.PERCLOS << endl;
			if(elapsedS % 1 == 0) {
				dataArray[0] = (double) ((double) elapsedS + timeMS);
				dataArray[14] = (double) CV.PERCLOS;
			}
			if (waitKey(1) == 27) {
				cout << "Program terminated" << endl;
				break;
			}
			
			startCVFlag = 0;
		}
		/*
		if(currVal % 2 == 0){
			if(prevVal != currVal){
				prevVal = currVal;
				// storing elapsed time to dataArray buffer
				dataArray[0] = (double) ((double) elapsedS + timeMS);
				// for writing for binary
				fwrite(dataArray, sizeof(double), 15, data);
				
				//printf("%lf\n",dataArray[0]);
			}
		}
		*/
		printf("Time: %lf seconds\n",(double) ((double) elapsedS + timeMS));
		//printf("%lfs\t%.02lf\t%.02lf\t%.02lf\t%.02lf\t%.02lf\t%.02lf\t%.02lf\t%.02lf\t%.02lf\t%.02lf\t%.02lf\t%.02lf\t%.02lf\t%.02lf\n",dataArray[0],dataArray[1],dataArray[2],dataArray[3],dataArray[4],dataArray[5],dataArray[6],dataArray[7],dataArray[8],dataArray[9],dataArray[10],dataArray[11],dataArray[12],dataArray[13],dataArray[14]);
		
	}
   
    return 0;
}
