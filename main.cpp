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
#include <vector>
#include <sstream>
#include <numeric>
#include <array>
#include <functional>
#include <iterator>

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

vector<double> center_weighted_savitzky_golay_filter(vector<double> data, int window_size, int polynomial_order, int derivative_order);

int main( int argc, char **argv ) {

	int bufferCount = 0;
	vector<double> eegBuffer[8];
	vector<double> imuBufferTemp[2];
	vector<double> imuBuffer[2];
	vector<double> cvBuffer[1];
	
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

	// Start of CV - Initialization
    CascadeClassifier eyeCascade;
    eyeCascade.load("haarcascade_eye_tree_eyeglasses.xml");
    CascadeClassifier faceCascade;
    faceCascade.load("haarcascade_frontalface_alt2.xml");
    // Configure here: cap(0) for webcam, cap((string)path) for videos
    // string path = "24-Blinks-1-Minute.mp4";
    VideoCapture cap(0);
    // Start Detecting Eyes
    Mat frame;
    Rect eye = Rect(0, 0, 0, 0);
    Rect iris = Rect(0, 0, 0, 0);
    while (eye.empty())
    {
        cap.read(frame);
        cout << "Detecting eyes..." << endl;
        if (frame.empty()) break;
        frame = CV.rotate(frame, 180);
        eye = CV.detectEyes(frame, eyeCascade);
        iris = CV.detectIris(frame, eye);

        imshow("detected", frame);
        waitKey(1);
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

	/*
	int iDataArray = 0;
	for (iDataArray=0; i<=14; i++){
		iDataArray[i] = 0;
	}
	*/
	
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
		
		if(serDataAvailable (EEG.serial_port))
		{
			// printf("new ser data avail: %lf\n",(double) ((double) elapsedS + timeMS) );
			streamByte = eegRead(&EEG);
			// printing streamByte for debugging purposes
			// printf("\n streamByte: %d", streamByte);
			fflush(stdout);
			//startIMUFlag = 1;
		}
		else if(startCVFlag == 0 && startIMUFlag != 0) 
		{
			readIMU(&IMU,1);
			dataArray[0] = (double) ((double) elapsedS + timeMS);

			// calculations for fitted Roll and fitted Pitch
			fittedRoll = (-0.000005857680127)*(powf(IMU.EulerAngles.Roll, 3)) + (-0.000059321347234)*(powf(IMU.EulerAngles.Roll, 2)) + 1.097026959773455*IMU.EulerAngles.Roll +  0.301807912947446;
			fittedPitch = (-0.000005959003219) *(powf(IMU.EulerAngles.Pitch, 3)) + 0.000202569646997*(powf(IMU.EulerAngles.Pitch, 2)) + 1.099149655304190*IMU.EulerAngles.Pitch +  (-0.953976026366887);

			// updating and writing data to bin file
			dataArray[12]=(double)fittedRoll;
			dataArray[13]=(double)fittedPitch;
			fwrite(dataArray,sizeof(double),15,data);

			//	creating vector array of data
			if(bufferCount != 900) {

				// inserting EEG data to eegBuffer for Model Prediction
				eegBuffer[0].push_back(dataArray[3]); // insert Delta Band to data vector
				eegBuffer[1].push_back(dataArray[4]); // insert Theta Band to data vector
				eegBuffer[2].push_back(dataArray[5]); // insert Low-alpha Band to data vector
				eegBuffer[3].push_back(dataArray[6]); // insert High-alpha Band to data vector
				eegBuffer[4].push_back(dataArray[7]); // insert Low-beta Band to data vector
				eegBuffer[5].push_back(dataArray[8]); // insert High-beta Band to data vector
				eegBuffer[6].push_back(dataArray[9]); // insert Low-gamma Band to data vector
				eegBuffer[7].push_back(dataArray[10]); // insert Mid-gamma Band to data vector

				// inserting IMU data to imuBufferTemp to be processed later before proceeding to model prediction
				imuBufferTemp[0].push_back(dataArray[12]); // insert fittedRoll to data vector
				imuBufferTemp[1].push_back(dataArray[13]); // insert fittedPitch Band to data vector

				// inserting PERCLOS data to cvBuffer for Model Prediction
				cvBuffer[0].push_back(dataArray[14]); // insert Perclos to data vector



				bufferCount++;


			}
			else if(bufferCount == 900) {
				
				int window_size = 51;
				int polynomial_order = 2;
				int derivative_order = 0;

				// SGOLAY FILTER for ROLL
				vector<double> sgolayRoll = center_weighted_savitzky_golay_filter(imuBufferTemp[0], window_size, polynomial_order, derivative_order);
				// SGOLAY FILTER for PITCH
				vector<double> sgolayPitch = center_weighted_savitzky_golay_filter(imuBufferTemp[1], window_size, polynomial_order, derivative_order);

				// getting Derivative of sgolayRoll and sgolayPitch
				vector<double> dysgolayRoll(sgolayRoll.size(), 0.0);
				vector<double> dysgolayPitch(sgolayPitch.size(), 0.0);

				// diff() function in matlab
    			std::adjacent_difference( std::begin(sgolayRoll), std::end(sgolayRoll), std::begin(dysgolayRoll) );
				std::adjacent_difference( std::begin(sgolayPitch), std::end(sgolayPitch), std::begin(dysgolayPitch) );

				imuBuffer[0] = dysgolayRoll;
				imuBuffer[1] = dysgolayPitch;
				// replacing first element of imuBuffer to 0 since pre-differentiated value at index 0 is retained
				imuBuffer[0][0] = 0.0;
    			imuBuffer[1][0] = 0.0;

				/* code here for evaluating prediction
				*	Use eegBuffer vector array for eeg classifier
				*	Use imuBuffer vector array for imu classifier
				*	Use cvBuffer vector array for cv classifier
				*/

				// clearing buffer after prediction
				eegBuffer.clear();
				imuBufferTemp.clear();
				imuBuffer.clear();
				cvBuffer.clear();
				bufferCount = 0;
			}

			startCVFlag = 1;
			startIMUFlag = 0;
		}
		else if(startCVFlag != 0) 
		{
			cap.read(frame);
			if (frame.empty()) break;
			frame = CV.rotate(frame, 180.0);
			frame = frame(eye);
			CV.detectBlink(frame, eye, iris);
			waitKey(1);
				
			// cout << "Time: " << (double) ((double) elapsedS + timeMS) << " ------ PERCLOS = " << CV.PERCLOS << endl;
			if(elapsedS % 1 == 0) 
			{
				dataArray[0] = (double) ((double) elapsedS + timeMS);
				dataArray[14] = (double) CV.PERCLOS;
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


vector<double> center_weighted_savitzky_golay_filter(vector<double> data, int window_size, int polynomial_order, int derivative_order) {
    vector<double> filtered_data;
    int half_window = (window_size - 1) / 2;
    int n = data.size();
    double sum_weights = 0;

    // Calculate weights for the filter
    vector<double> weights(window_size);
    for (int i = -half_window; i <= half_window; i++) {
        double weight = 0;
        for (int j = 0; j <= polynomial_order; j++) {
            weight += pow(i, j);
        }
        weight = abs(weight);
        weights[i + half_window] = weight;
        sum_weights += weight;
    }

    // Apply the filter
    for (int i = 0; i < n; i++) {
        double filtered_value = 0;
        for (int j = -half_window; j <= half_window; j++) {
            int index = i + j;
            if (index < 0) {
                index = 0;
            }
            if (index >= n) {
                index = n - 1;
            }
            filtered_value += weights[j + half_window] * data[index];
        }
        filtered_value /= sum_weights;
        filtered_data.push_back(filtered_value);
    }

    return filtered_data;
}
