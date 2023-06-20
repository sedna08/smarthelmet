#include <stdio.h>
#include <opencv2/opencv.hpp>

#include "eeg.h"
#include "Parser_Filter.h"
#include "cv.h"
#include "imu.h"
#include "kalmanFilter.h"

#include <string.h>
#include <string>
#include <fstream>
#include <errno.h>
#include <csignal>
#include <unistd.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>
#include <pigpio.h>
#include <iostream>
#include <vector>
#include <sstream>
#include <numeric>
#include <array>
#include <functional>
#include <iterator>
#include <algorithm>

#include <mutex>
#include <thread>
#include <chrono>
// #include <fdeep/fdeep.hpp>
#include </usr/local/include/fdeep/fdeep.hpp>
#include "alarm.h"

using namespace cv;
using namespace std;

Alarm alarm1;
EEG EEG;
IMU IMU;
CV CV;
FILE *Data_text;
FILE *data;
FILE *downdata;
extern double dataArray[15];
int prevVal = 0, currVal = 0;
int elapsedS = 0, elapsedMS = 0;
int startS = 0, startMS = 0;
int currentS = 0, currentMS = 0;
double timeMS = 0.0f;
int startIMUFlag = 0;
int startCVFlag = 0;

/** For Classification **/

int clf_result = -1;
int eegclf_state = -1;
int cvclf_state = -1;
int imuclf_state = -1;

vector<double> eegBuffer[8];
vector<double> imuBufferTemp[2];
vector<double> imuBuffer[2];
vector<double> cvBuffer[1];

// Queued data for accessing only in threaded process
vector<double> q_eegBuffer[8];
vector<double> q_imuBufferTemp[2];
vector<double> q_imuBuffer[2];
vector<double> q_cvBuffer[1];

// Flags
int f_t1 = 1; // thread 1 flag
// mutex data_lock;
thread processing_thread;

void signalHandler(int signum)
{
	cout << "\nInterrupt Handling #" << signum << endl;
	serClose(EEG.serial_port);

	// Set flags
	f_t1 = 0;

	cout << "End of process." << endl;
	processing_thread.join();
 
   gpioPWM(PIN_PWM, 0); // Sets GPIO23 full off.
   cout << "dutycycle (0-255): " << gpioGetPWMdutycycle(PIN_PWM) << endl;

	free(EEG.queue);
	free(EEG.queue1);
	fclose(data);
	// turning back gpio 4 and 17 to input
	gpioWrite(17, 0);
	gpioWrite(4, 0);
	gpioSetMode(4, PI_INPUT);
	gpioSetMode(17, PI_INPUT);
	gpioTerminate();
	fclose(Data_text);
	fclose(downdata);

	// cleanup and close up stuff here
	// terminate program

	exit(signum);
}


int curr_lvl = 1;
int prev_lvl = 0;

void buttonPressCallback(int gpio, int level, uint32_t tick);

void buttonReleaseCallback(int gpio, int level, uint32_t tick);

vector<double> center_weighted_savitzky_golay_filter(vector<double> data, int window_size, int polynomial_order, int derivative_order);
void predict_fdeep_nn(const fdeep::model &cvclf_model, const fdeep::model &imuclf_model, const fdeep::model &eegclf_model, const fdeep::model &seclvlclf_model);
const std::vector<float> *vdouble_to_vfloat(const std::vector<double> *arr, int size);

void aFunction(int gpio, int level, uint32_t tick);

int main(int argc, char **argv)
{
	int downsampleCounter = 0;
	// Load converted model (JSON)
	const auto cvclf_model = fdeep::load_model("best_model(perclosv2).json");
	const auto imuclf_model = fdeep::load_model("best_model(imuv3).json");
	const auto eegclf_model = fdeep::load_model("best_model(eegv2).json");
	const auto seclvlclf_model = fdeep::load_model("best_model(2ndlevelv2).json");

	long double fittedRoll;
	long double fittedPitch;

  /*
	if (gpioInitialise() < 0) // initialises pigpio.h
	{
		// if pigpio initialisation failed
		cout << "pigpio.h initialisation failed\n";
		return -1;
	}
  */


	// register SIGINT and signal handler
   alarm1.init();
	signal(SIGINT, signalHandler);
 
   
   
   gpioSetISRFunc(PIN_BTN, RISING_EDGE, 0, buttonPressCallback);
    alarm1.state = 0;

	// gpio 4 is set to output for LED indicator for poor signal quality
	gpioSetMode(4, PI_OUTPUT);
	gpioSetMode(4, PI_OUTPUT);

	// Initialize CV
	printf("Initializing CV\n");
	initCV(&CV, 2);

	// Initialize 1st IMU
	printf("Initializing IMU MPU6050\n");
	initDevice(&IMU, 1);

	printf("Initializing NeuroSky\n");
	initEEG(&EEG);

	unsigned char streamByte;
	Data_text = fopen("Text_Data.txt", "w");

	data = fopen("data.bin", "wb");
	downdata = fopen("downdata.bin", "wb");

	double downdataArray[12];

	downdataArray[0] = 0;
	downdataArray[1] = 0;
	downdataArray[2] = 0;
	downdataArray[3] = 0;
	downdataArray[4] = 0;
	downdataArray[5] = 0;
	downdataArray[6] = 0;
	downdataArray[7] = 0;
	downdataArray[8] = 0;
	downdataArray[9] = 0;
	downdataArray[10] = 0;
	downdataArray[11] = 0;

	dataArray[0] = 0;  // index 0 for time
	dataArray[1] = 0;  // index 1 for EEG Raw Values
	dataArray[2] = 0;  // index 2 for EEG Values with MAV Filter
	dataArray[3] = 0;  // index 3 for Delta Band
	dataArray[4] = 0;  // index 4 for Theta Band
	dataArray[5] = 0;  // index 5 for Low-alpha Band
	dataArray[6] = 0;  // index 6 for High-alpha Band
	dataArray[7] = 0;  // index 7 for Low-beta Band
	dataArray[8] = 0;  // index 8 for High-beta Band
	dataArray[9] = 0;  // index 9 for Low-gamma Band
	dataArray[10] = 0; // index 10 for Mid-gamma Band
	dataArray[11] = 0; // index 11 for Poor-Signal Quality
	dataArray[12] = 0; // index 12 for fitterRoll IMU values
	dataArray[13] = 0; // index 13 for fittedPitch IMU values
	dataArray[14] = 0; // index 14 for PERCLOS

	// getting start time
	gpioTime(0, &startS, &startMS);

	processing_thread = thread(predict_fdeep_nn, ref(cvclf_model), ref(imuclf_model), ref(eegclf_model), ref(seclvlclf_model));

	while (true)
	{
		// Getting current time
		gpioTime(0, &currentS, &currentMS);
		// Getting elapsed time from start to current
		elapsedS = currentS - startS;
		elapsedMS = currentMS - startMS;
		// Getting integer elapsed Microsecond and converting it to float for an actual float value of microsecond
		timeMS = (double)elapsedMS / 1000000.0f;
		// Getting elapsed millisecond from elapsed microsecond
		currVal = elapsedMS / 1000;

		if (serDataAvailable(EEG.serial_port))
		{
			streamByte = eegRead(&EEG);
			// printing streamByte for debugging purposes
			// printf("\n streamByte: %d", streamByte);
			fflush(stdout);
			// startIMUFlag = 1;
			//printf("new ser data avail: %lf\n", (double)((double)elapsedS + timeMS));
		}
		else if (startCVFlag == 0 && startIMUFlag != 0)
		{
			readIMU(&IMU, 1);
			dataArray[0] = (double)((double)elapsedS + timeMS);

			// calculations for fitted Roll and fitted Pitch
			fittedRoll = (-0.000005857680127) * (powf(IMU.EulerAngles.Roll, 3)) + (-0.000059321347234) * (powf(IMU.EulerAngles.Roll, 2)) + 1.097026959773455 * IMU.EulerAngles.Roll + 0.301807912947446;
			fittedPitch = (-0.000005959003219) * (powf(IMU.EulerAngles.Pitch, 3)) + 0.000202569646997 * (powf(IMU.EulerAngles.Pitch, 2)) + 1.099149655304190 * IMU.EulerAngles.Pitch + (-0.953976026366887);

			// updating and writing data to bin file
			dataArray[12] = (double)fittedRoll;
			dataArray[13] = (double)fittedPitch;
			fwrite(dataArray,sizeof(double),15,data);
      

			// Set flags
			startCVFlag = 1;
			startIMUFlag = 0;
		}
		else if (startCVFlag != 0)
		{
			readCV(&CV);

			// cout << "Time: " << (double) ((double) elapsedS + timeMS) << " ------ PERCLOS = " << CV.PERCLOS << endl;
			if (elapsedS % 1 == 0)
			{
				dataArray[0] = (double)((double)elapsedS + timeMS);
				dataArray[14] = (double)CV.perclos;
			}
      
      
      downsampleCounter++;

			//	creating vector array of data
			// std::lock_guard<std::mutex> lock(data_lock);
			if (cvBuffer[0].size() != 900 && downsampleCounter == 5)
			{

				// inserting EEG data to eegBuffer for Model Prediction
				eegBuffer[0].push_back(dataArray[3]);  // insert Delta Band to data vector
				eegBuffer[1].push_back(dataArray[4]);  // insert Theta Band to data vector
				eegBuffer[2].push_back(dataArray[5]);  // insert Low-alpha Band to data vector
				eegBuffer[3].push_back(dataArray[6]);  // insert High-alpha Band to data vector
				eegBuffer[4].push_back(dataArray[7]);  // insert Low-beta Band to data vector
				eegBuffer[5].push_back(dataArray[8]);  // insert High-beta Band to data vector
				eegBuffer[6].push_back(dataArray[9]);  // insert Low-gamma Band to data vector
				eegBuffer[7].push_back(dataArray[10]); // insert Mid-gamma Band to data vector

				// inserting IMU data to imuBufferTemp to be processed later before proceeding to model prediction
				imuBufferTemp[0].push_back(dataArray[12]); // insert fittedRoll to data vector
				imuBufferTemp[1].push_back(dataArray[13]); // insert fittedPitch Band to data vector

				// inserting PERCLOS data to cvBuffer for Model Prediction
				cvBuffer[0].push_back(dataArray[14]); // insert Perclos to data vector

				downdataArray[0] = dataArray[0];
				downdataArray[1] = dataArray[3];
				downdataArray[2] = dataArray[4];
				downdataArray[3] = dataArray[5];
				downdataArray[4] = dataArray[6];
				downdataArray[5] = dataArray[7];
				downdataArray[6] = dataArray[8];
				downdataArray[7] = dataArray[9];
				downdataArray[8] = dataArray[10];
				downdataArray[9] = dataArray[12];
				downdataArray[10] = dataArray[13];
				downdataArray[11] = dataArray[14];
				fwrite(downdataArray, sizeof(double), 12, downdata);

				downsampleCounter = 0;
			}
			else if (cvBuffer[0].size() == 900 && downsampleCounter == 5)
			{
				// std::lock_guard<std::mutex> lock(data_lock);
				if (q_cvBuffer[0].size() == 0)
				{
					copy(eegBuffer, eegBuffer + 8, q_eegBuffer);
					copy(imuBufferTemp, imuBufferTemp + 2, q_imuBufferTemp);
					copy(cvBuffer, cvBuffer + 1, q_cvBuffer);
				}
				else
				{
					q_eegBuffer[0].push_back(eegBuffer[0][899]);
					q_eegBuffer[1].push_back(eegBuffer[1][899]);
					q_eegBuffer[2].push_back(eegBuffer[2][899]);
					q_eegBuffer[3].push_back(eegBuffer[3][899]);
					q_eegBuffer[4].push_back(eegBuffer[4][899]);
					q_eegBuffer[5].push_back(eegBuffer[5][899]);
					q_eegBuffer[6].push_back(eegBuffer[6][899]);
					q_eegBuffer[7].push_back(eegBuffer[7][899]);
					q_imuBufferTemp[0].push_back(imuBufferTemp[0][899]);
					q_imuBufferTemp[1].push_back(imuBufferTemp[1][899]);
					q_cvBuffer[0].push_back(cvBuffer[0][899]);
				}

				eegBuffer[0].erase(eegBuffer[0].begin());
				eegBuffer[1].erase(eegBuffer[1].begin());
				eegBuffer[2].erase(eegBuffer[2].begin());
				eegBuffer[3].erase(eegBuffer[3].begin());
				eegBuffer[4].erase(eegBuffer[4].begin());
				eegBuffer[5].erase(eegBuffer[5].begin());
				eegBuffer[6].erase(eegBuffer[6].begin());
				eegBuffer[7].erase(eegBuffer[7].begin());
				imuBufferTemp[0].erase(imuBufferTemp[0].begin());
				imuBufferTemp[1].erase(imuBufferTemp[1].begin());
				cvBuffer[0].erase(cvBuffer[0].begin());

				downsampleCounter = 0;
			}
      
      printf("Time: %lf seconds",(double) ((double) elapsedS + timeMS));
     printf(" Buffer %d ", cvBuffer[0].size());
     
      switch (eegclf_state)
			{
				case 0:
					printf(" eeg: Awake");
					break;
				case 1:
					printf(" eeg: Drowsy");
					break;
				case -1:
					printf(" eeg: Loading");
					break;
			}
      switch (cvclf_state)
			{
				case 0:
					printf(" cv: Awake");
					break;
				case 1:
					printf(" cv: Drowsy");
					break;
				case -1:
					printf(" cv: Loading");
					break;
			}
      switch (imuclf_state)
			{
				case 0:
					printf(" imu: Awake");
					break;
				case 1:
					printf(" imu: Drowsy");
					break;
				case -1:
					printf(" imu: Loading");
					break;
			}
			switch (clf_result)
			{
				case 0:
					printf(" 2ndlevel: Awake\n");
					break;
				case 1:
					printf(" 2ndlevel: Drowsy\n");
          alarm1.state = 1;
          alarm1.sound1();
					break;
				case -1:
					printf(" 2ndlevel: Loading\n");
					break;
			}

			// Set flags
			startCVFlag = 0;
      
			
		}

		// printf("%lfs\t%.02lf\t%.02lf\t%.02lf\t%.02lf\t%.02lf\t%.02lf\t%.02lf\t%.02lf\t%.02lf\t%.02lf\t%.02lf\t%.02lf\t%.02lf\t%.02lf\n",dataArray[0],dataArray[1],dataArray[2],dataArray[3],dataArray[4],dataArray[5],dataArray[6],dataArray[7],dataArray[8],dataArray[9],dataArray[10],dataArray[11],dataArray[12],dataArray[13],dataArray[14]);
	}

	// Set flags
	f_t1 = 0;
	processing_thread.join();

	return 0;
}

vector<double> center_weighted_savitzky_golay_filter(vector<double> data, int window_size, int polynomial_order, int derivative_order)
{
	vector<double> filtered_data;
	int half_window = (window_size - 1) / 2;
	int n = data.size();
	double sum_weights = 0;

	// Calculate weights for the filter
	vector<double> weights(window_size);
	for (int i = -half_window; i <= half_window; i++)
	{
		double weight = 0;
		for (int j = 0; j <= polynomial_order; j++)
		{
			weight += pow(i, j);
		}
		weight = abs(weight);
		weights[i + half_window] = weight;
		sum_weights += weight;
	}

	// Apply the filter
	for (int i = 0; i < n; i++)
	{
		double filtered_value = 0;
		for (int j = -half_window; j <= half_window; j++)
		{
			int index = i + j;
			if (index < 0)
			{
				index = 0;
			}
			if (index >= n)
			{
				index = n - 1;
			}
			filtered_value += weights[j + half_window] * data[index];
		}
		filtered_value /= sum_weights;
		filtered_data.push_back(filtered_value);
	}

	return filtered_data;
}

const std::vector<float> *vdouble_to_vfloat(const std::vector<double> *arr, int size)
{
	std::vector<float> *floatArr = new std::vector<float>[size];

	for (int i = 0; i < size; i++)
	{
		floatArr[i].resize(arr[i].size());
		std::transform(arr[i].begin(), arr[i].end(), floatArr[i].begin(), [](double d)
					   { return static_cast<float>(d); });
	}

	return floatArr;
}

/**
 * @brief 
 * 
 * @param cvclf_model 
 * @param imuclf_model 
 * @param eegclf_model 
 * @param seclvlclf_model 
 */
void predict_fdeep_nn(const fdeep::model& cvclf_model, const fdeep::model& imuclf_model, const fdeep::model& eegclf_model, const fdeep::model& seclvlclf_model)
{
    while (f_t1)
    {
        // std::lock_guard<std::mutex> lock(data_lock);
        // Process data on queue
        if (q_cvBuffer[0].size() >= 900)
        {
            vector<double> t_eegBuffer[8];
            vector<double> t_imuBufferTemp[2];
            vector<double> t_imuBuffer[2];
            vector<double> t_cvBuffer[1];

            copy(q_eegBuffer, q_eegBuffer + 8, t_eegBuffer);
            copy(q_imuBufferTemp, q_imuBufferTemp + 2, t_imuBufferTemp);
            copy(q_cvBuffer, q_cvBuffer + 1, t_cvBuffer);


            for (int i = 0; i < 8; i++)
            {
                if (i < 1) t_cvBuffer[i].resize(900);
                if (i < 2) t_imuBufferTemp[i].resize(900);
                if (i < 8) t_eegBuffer[i].resize(900);
            }

            q_eegBuffer[0].erase(q_eegBuffer[0].begin());
            q_eegBuffer[1].erase(q_eegBuffer[1].begin());
            q_eegBuffer[2].erase(q_eegBuffer[2].begin());
            q_eegBuffer[3].erase(q_eegBuffer[3].begin());
            q_eegBuffer[4].erase(q_eegBuffer[4].begin());
            q_eegBuffer[5].erase(q_eegBuffer[5].begin());
            q_eegBuffer[6].erase(q_eegBuffer[6].begin());
            q_eegBuffer[7].erase(q_eegBuffer[7].begin());
            q_imuBufferTemp[0].erase(q_imuBufferTemp[0].begin());
            q_imuBufferTemp[1].erase(q_imuBufferTemp[1].begin());
            q_cvBuffer[0].erase(q_cvBuffer[0].begin());

            // TODO - IMU data pre-processing
            // SGolay filter for roll and pitch
            vector<double> sgolayRoll = center_weighted_savitzky_golay_filter(t_imuBufferTemp[0], 51, 2, 0);
            vector<double> sgolayPitch = center_weighted_savitzky_golay_filter(t_imuBufferTemp[1], 51, 2, 0);

            // getting Derivative of sgolayRoll and sgolayPitch
            vector<double> dysgolayRoll(sgolayRoll.size(), 0.0);
            vector<double> dysgolayPitch(sgolayPitch.size(), 0.0);

            // diff() function in matlab
            std::adjacent_difference(std::begin(sgolayRoll), std::end(sgolayRoll), std::begin(dysgolayRoll));
            std::adjacent_difference(std::begin(sgolayPitch), std::end(sgolayPitch), std::begin(dysgolayPitch));

            t_imuBuffer[0] = dysgolayRoll;
            t_imuBuffer[1] = dysgolayPitch;
            t_imuBuffer[0][0] = 0.0;
            t_imuBuffer[1][0] = 0.0;

            // TODO - Classifier
            // Fill an fdeep::tensor with values, e.g., from an std::vector<float>
            // Convert double to float
            const vector<float>* c_eegBuffer = vdouble_to_vfloat(t_eegBuffer, 8);
            const vector<float>* c_imuBuffer = vdouble_to_vfloat(t_imuBuffer, 2);
            const vector<float>* c_cvBuffer = vdouble_to_vfloat(t_cvBuffer, 1);

            // cout << c_cvBuffer[0][0] << endl;

            // Const 900x8 tensor for EEG
            const int eeg_tensor_rows = 900;
            const int eeg_tensor_cols = 8;
            fdeep::tensor_shape eeg_tensor_shape(eeg_tensor_rows, eeg_tensor_cols);
            fdeep::tensor eeg_t(eeg_tensor_shape, 0.0f);

            for (int i = 0; i < eeg_tensor_rows; i++)
            {
                for (int j = 0; j < eeg_tensor_cols; j++)
                {
                    eeg_t.set(fdeep::tensor_pos(i, j), c_eegBuffer[j][i]);
                }
            }


            // Const 900x2 tensor for IMU
            const int imu_tensor_rows = 900;
            const int imu_tensor_cols = 2;
            fdeep::tensor_shape imu_tensor_shape(imu_tensor_rows, imu_tensor_cols);
            fdeep::tensor imu_t(imu_tensor_shape, 0.0f);

            for (int i = 0; i < imu_tensor_rows; i++)
            {
                for (int j = 0; j < imu_tensor_cols; j++)
                {
                    imu_t.set(fdeep::tensor_pos(i, j), c_imuBuffer[j][i]);
                }
            }

            // Const 900x1 tensor for CV
            const fdeep::tensor cv_t(fdeep::tensor_shape(900, 1), c_cvBuffer[0]); // 900x1 tensor
            // cout << fdeep::show_tensor(cv_t) << endl;

            try
            {
                // Step X.x: Uncomment this section to show probability of classes
                // auto result = cvmodel.predict({ t }); // predict class
                // std::cout << fdeep::show_tensors(result) << std::endl; // print the tensor

                // --- Step X.x (Optional): Uncomment this section to show predicted class
                //   - This section outputs 0 and 1
                //       - 0 = awake, 1 = drowsy
                const auto cvclf_result = cvclf_model.predict_class({ cv_t }); // predict class
                if (cvclf_result == 1) cvclf_state = 1;
                else cvclf_state = 0;
                
                const auto eegclf_result = eegclf_model.predict_class({ eeg_t }); // predict class
                if (eegclf_result == 1) eegclf_state = 1;
                else eegclf_state = 0;
                
                const auto imuclf_result = imuclf_model.predict_class({ imu_t }); // predict class
                if (imuclf_result == 1) imuclf_state = 1;
                else imuclf_state = 0;
                
                

                /*cout << (float)cvclf_result << "\t";
                cout << (float)eegclf_result << "\t";
                cout << (float)imuclf_result << endl;*/

                // Const 1x3 vector for second level clf
                const vector<float> c_seclvlBuffer = {
                    (float)cvclf_result,
                    (float)eegclf_result,
                    (float)imuclf_result,
                };


                // --- Step 2: Second level clf
                const fdeep::tensor seclvl_t(fdeep::tensor_shape(3), c_seclvlBuffer); // 1x3 tensor for second level clf
                const auto seclvlclf_result = seclvlclf_model.predict({ seclvl_t }); // predict class probability (should contain 1 class)
                const vector<float> vec = seclvlclf_result[0].to_vector(); // convert tensor result to vector of type float
                int result = static_cast<int>(std::round(vec[0])); // convert float to int
                
                if (result == 1) clf_result = 1;
                else clf_result = 0;
                /*
                if (clf_result == 1) cout << "Drowsy" << endl;
                else if (clf_result == 0) cout << "Awake" << endl;
                else cout << "Loading..." << endl;
                */
            }
            catch (const std::exception& e)
            {
                std::cout << e.what() << '\n';
            }
        }
    }
}

void buttonPressCallback(int gpio, int level, uint32_t tick)
{
    static uint32_t lastTick = 0;
    uint32_t currentTick = gpioTick();
    if ((currentTick - lastTick) < 200000) {
        // Debounce - ignore button press if less than 200ms since last press
        return;
    }

    alarm1.state = 0;
    alarm1.sound1();
    //printf("GPIO %d became %d at %d and state became %d\n", gpio, level, tick, alarm.state);
    lastTick = currentTick;
}
void buttonReleaseCallback(int gpio, int level, uint32_t tick) {
    // Do nothing
}