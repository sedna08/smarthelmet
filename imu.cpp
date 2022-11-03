#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "imu.h"
#include <iostream>
#include <fstream>
#include <time.h>
#include <pigpio.h>
#include <queue>
#include <thread>
using namespace std;

void pushNewData(queue<float> *q, float newData) {
	q->pop();
    q->push(newData);
}

void populateQueue(queue<float> *q, int size) {
    for (int i=0; i<size; i++) {
        q->push(0);
    }
}

IMU::IMU() {
	populateQueue(&(IMU::AccX), 7);
	populateQueue(&(IMU::AccY), 7);
	populateQueue(&(IMU::AccZ), 7);
}

void IMU::setIdentifiers(int ID, int MAG_ID) {
	IMU::identifier = ID;
	IMU::mag_identifier = MAG_ID;
}

void IMU::getIdentifier(int *ID) {
	*ID = IMU::identifier;
}

void IMU::getMagIdentifier(int *MAG_ID) {
	*MAG_ID = IMU::mag_identifier;
}

void MPU9250_Init(int ID){
	bool errorFlag = false;
	// wake up device
	if (i2cWriteByteData(ID, PWR_MGMT_1, 0x00) != 0) errorFlag = true;  // Clear sleep mode bit (6), enable all sensors 
	gpioDelay(100000); // Wait for all registers to reset 

 	// get stable time source
	if (i2cWriteByteData(ID, PWR_MGMT_1, 0x01) != 0) errorFlag = true;  // Auto select clock source to be PLL gyroscope reference if ready else
	gpioDelay(200000);

	// Configure Gyro and Thermometer
	// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
	// minimum gpioDelay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
	// be higher than 1 / 0.0059 = 170 Hz
	// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
	// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
	if (i2cWriteByteData(ID, CONFIG, 0x03) != 0) errorFlag = true;

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	if (i2cWriteByteData(ID, SMPLRT_DIV, 0x04) != 0) errorFlag = true;  // Use a 200 Hz rate; a rate consistent with the filter update rate 
									  			 // determined inset in CONFIG above

	// Set gyroscope full scale range
	// Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	u_int8_t c = i2cReadByteData(ID, GYRO_CONFIG); // get current GYRO_CONFIG register value
	// c = c & ~0xE0; // Clear self-test bits [7:5] 
	c = c & ~0x03; // Clear Fchoice bits [1:0] 
	c = c & ~0x18; // Clear GFS bits [4:3]
	c = c | (0 << 3); // Set full scale range for the gyro
	// c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
	if (i2cWriteByteData(ID, GYRO_CONFIG, c) != 0) errorFlag = true; // Write new GYRO_CONFIG value to register

	// Set accelerometer full-scale range configuration
	c = i2cReadByteData(ID, ACCEL_CONFIG);							// get current ACCEL_CONFIG register value
	// c = c & ~0xE0; // Clear self-test bits [7:5] 
	c = c & ~0x18;  // Clear AFS bits [4:3]
	c = c | (0 << 3); // Set full scale range for the accelerometer 
	if (i2cWriteByteData(ID, ACCEL_CONFIG, c) != 0) errorFlag = true; // Write new ACCEL_CONFIG register value

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	c = i2cReadByteData(ID, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
	c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
	c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	if (i2cWriteByteData(ID, ACCEL_CONFIG2, c) != 0) errorFlag = true; // Write new ACCEL_CONFIG2 register value
	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
	// clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
	// can join the I2C bus and all can be controlled by the Arduino as master

	if (i2cWriteByteData(ID, INT_PIN_CFG, 0x12) != 0) errorFlag = true; // INT is 50 microsecond pulse and any read to clear   
	if (i2cWriteByteData(ID, INT_ENABLE, 0x01) != 0) errorFlag = true;  // Enable data ready (bit 0) interrupt
	gpioDelay(100000);

	if (errorFlag) {
		cout << "Error Initializing MPU-9250" << endl;
	}
}

// initializing the magnetometer and calibration
void IMU::initCalibMagnetometer(int IMU_NUM) {
	IMU::initMagnetometer();
	// IMU::calibrateMagnetometer();
	IMU::calibrateMagnetometer_SET(IMU_NUM);
}

void IMU::initMagnetometer() {
	int MAG_ID = IMU::mag_identifier;

	bool errorFlag = false;
	// First extract the factory calibration for each magnetometer axis
	u_int8_t rawData[3];  // x/y/z gyro calibration data stored here
	float MagValues[3];
	if (i2cWriteByteData(MAG_ID, AK8963_CNTL, 0x00) != 0) errorFlag = true;  // Power down magnetometer  
	gpioDelay(10000);
	if (i2cWriteByteData(MAG_ID, AK8963_CNTL, 0x0F)!= 0) errorFlag = true; // Enter Fuse ROM access mode
	gpioDelay(10000);

	rawData[0] = i2cReadByteData(MAG_ID, AK8963_ASAX);
	rawData[1] = i2cReadByteData(MAG_ID, AK8963_ASAY);
	rawData[2] = i2cReadByteData(MAG_ID, AK8963_ASAZ);

	IMU::MagCalib.X =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
	IMU::MagCalib.Y =  (float)(rawData[1] - 128)/256. + 1.;  
	IMU::MagCalib.Z =  (float)(rawData[2] - 128)/256. + 1.;

	printf("Magnetometer Calibration Values:\nX: %f \nY: %f \nZ: %f \n",
			IMU::MagCalib.X, IMU::MagCalib.Y, IMU::MagCalib.Z
	);

	if (i2cWriteByteData(MAG_ID, AK8963_CNTL, 0x00) != 0) errorFlag = true; // Power down magnetometer  
	gpioDelay(10000);
	// Configure the magnetometer for continuous read and highest resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	int Mmode = 0x06; // 100 Hz
	// int Mmode = 0x02; // 8 Hz
	//Set to 8 Hz @ 16bit
	if (i2cWriteByteData(MAG_ID, AK8963_CNTL, 0x00 | Mmode) != 0) errorFlag = true; // Set magnetometer data resolution and sample ODR
	gpioDelay(10000);

	if (errorFlag) {
		cout << "Error Initializing Magnetometer Data" << endl;
	}
}

void IMU::calibrateMagnetometer_SET(int IMU_NUM) {
	switch(IMU_NUM) {
		case 1: {
			printf("Case 1\n");
			IMU::MagBias.X = 62.957651;
			IMU::MagBias.Y = 84.985630; 
			IMU::MagBias.Z = -72.524249;

			IMU::MagCorrectionVal[0][0] = 0.967571;
			IMU::MagCorrectionVal[0][1] = 0.010724;
			IMU::MagCorrectionVal[0][2] = 0.017179;
			IMU::MagCorrectionVal[1][0] = 0.010724;
			IMU::MagCorrectionVal[1][1] = 0.927992;
			IMU::MagCorrectionVal[1][2] = -0.015795;
			IMU::MagCorrectionVal[2][0] = 0.017179;
			IMU::MagCorrectionVal[2][1] = -0.015795;
			IMU::MagCorrectionVal[2][2] = 0.905527;

			// IMU::MagBias.X = 134.4296;
			// IMU::MagBias.Y = 657.9977;
			// IMU::MagBias.Z = -478.8675;

			// IMU::MagScale.X = 0.9920635;
			// IMU::MagScale.Y = 0.9469697;
			// IMU::MagScale.Z = 1.068376;
			break;
		}
		case 2: {
			printf("Case 2\n");
			IMU::MagBias.X = -34.072534;
			IMU::MagBias.Y = 45.019629;
			IMU::MagBias.Z = -52.878930;

			IMU::MagCorrectionVal[0][0] = 0.927127;
			IMU::MagCorrectionVal[0][1] = 0.021618;
			IMU::MagCorrectionVal[0][2] = -0.015469;
			IMU::MagCorrectionVal[1][0] = 0.021618;
			IMU::MagCorrectionVal[1][1] = 0.961346;
			IMU::MagCorrectionVal[1][2] = 0.020737;
			IMU::MagCorrectionVal[2][0] = -0.015469;
			IMU::MagCorrectionVal[2][1] = 0.020737;
			IMU::MagCorrectionVal[2][2] = 0.956699;

			// IMU::MagBias.X = -63.46635;
			// IMU::MagBias.Y = 326.5389;
			// IMU::MagBias.Z = 6.817537;

			// IMU::MagScale.X = 1.047619;
			// IMU::MagScale.Y = 0.9565217;
			// IMU::MagScale.Z = 1.000000;
			break;
		}
		default: {
			printf("No Case\n");
			IMU::MagBias.X = 0.0;
			IMU::MagBias.Y = 0.0;
			IMU::MagBias.Z = 0.0;

			IMU::MagScale.X = 0.0;
			IMU::MagScale.Y = 0.0;
			IMU::MagScale.Z = 0.0;
			break;
		}
	}
	// IMU::MagBias.X = 36.4375060;
	// IMU::MagBias.Y = 67.9223460;
	// IMU::MagBias.Z = -61.2266340;

	// IMU::MagScale.X = 1.0085483;
	// IMU::MagScale.Y = 0.9821303;
	// IMU::MagScale.Z = 1.0165571;
}

void IMU::calibrateMagnetometer(){
	u_int16_t ii = 0, sample_count = 0;
	int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
	int16_t mag_max[3] = {-8190, -8190, -8190}, mag_min[3] = {8190, 8190, 8190}, mag_temp[3] = {0, 0, 0};
	// int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

	printf("Magnetometer Calibration: Wave device in a figure eight until done!\n");
	gpioDelay(4000000);

	sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
	
	ofstream data;
    data.open("data/magdata/mag2_data.dat");
    char str[100];
	for(ii = 0; ii < sample_count; ii++) {
		IMU::readRawIMU();
		mag_temp[0] = IMU::MagData.X;
		mag_temp[1] = IMU::MagData.Y;
		mag_temp[2] = IMU::MagData.Z;

		sprintf(str, "%e\t%e\t%e",
                IMU::MagData.X, IMU::MagData.Y, IMU::MagData.Z
		);
		data << str << endl;
		printf("%d\t%e\t%e\t%e\n",ii,IMU::MagData.X, IMU::MagData.Y, IMU::MagData.Z);

		for (int jj = 0; jj < 3; jj++) {
			if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
			if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
		}

		gpioDelay(12000);  // at 100 Hz ODR, new mag data is available every 10 ms
    }
	data.close();

	// Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

	float res_14bit = 8190.0;
	float mRes = 10.*4912./res_14bit;

	IMU::MagBias.X = (float) mag_bias[0] * mRes * IMU::MagCalib.X;
	IMU::MagBias.Y = (float) mag_bias[1] * mRes * IMU::MagCalib.Y;
	IMU::MagBias.Z = (float) mag_bias[2] * mRes * IMU::MagCalib.Z;

	// Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

	IMU::MagScale.X = avg_rad/((float)mag_scale[0]);
	IMU::MagScale.Y = avg_rad/((float)mag_scale[1]);
	IMU::MagScale.Z = avg_rad/((float)mag_scale[2]);

	printf("Magnetometer Bias Values:\nX: %e mG\nY: %e mG\nZ: %e mG\n",
			IMU::MagBias.X, IMU::MagBias.Y, IMU::MagBias.Z
	);
	printf("Magnetometer Scale Values:\nX: %e mG\nY: %e mG\nZ: %e mG\n",
			IMU::MagScale.X, IMU::MagScale.Y, IMU::MagScale.Z
	);
	printf("Mag Calibration done! Press Enter to continue to next IMU\n");
	printf("Note: Lay IMU Flat");
    char test;
	cin.get(test);
}

short read_raw_data(int IMUIdentifier, int addr) {
	int low_addr;
	short high_byte, low_byte, value;

	low_addr = (addr == AK8963_XOUT_H || addr == AK8963_YOUT_H || addr == AK8963_ZOUT_H) ? (addr-1) : (addr+1);

	high_byte = i2cReadByteData(IMUIdentifier, addr);
	low_byte  = i2cReadByteData(IMUIdentifier, low_addr);
	
	value = (high_byte << 8) | low_byte;
	return value;
}

void IMU::calibrateMPU9250() {
	u_int8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  	int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
	
	int ID = IMU::identifier;
	bool errorFlag = false;

	// reset device
	if (i2cWriteByteData(ID, PWR_MGMT_1, 0x80) != 0) errorFlag = true; // Write a one to bit 7 reset bit; toggle reset device
	gpioDelay(100000);

	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
	// else use the internal oscillator, bits 2:0 = 001
	if (i2cWriteByteData(ID, PWR_MGMT_1, 0x01) != 0) errorFlag = true; 
	if (i2cWriteByteData(ID, PWR_MGMT_2, 0x00) != 0) errorFlag = true;
	gpioDelay(200000);

	// Configure device for bias calculation
	if (i2cWriteByteData(ID, INT_ENABLE, 0x00) != 0)   errorFlag = true; // Disable all interrupts
	if (i2cWriteByteData(ID, FIFO_EN, 0x00) != 0)      errorFlag = true; // Disable FIFO
	if (i2cWriteByteData(ID, PWR_MGMT_1, 0x00) != 0)   errorFlag = true; // Turn on internal clock source
	if (i2cWriteByteData(ID, I2C_MST_CTRL, 0x00) != 0) errorFlag = true; // Disable I2C master
	if (i2cWriteByteData(ID, USER_CTRL, 0x00) != 0)    errorFlag = true; // Disable FIFO and I2C master modes
	if (i2cWriteByteData(ID, USER_CTRL, 0x0C) != 0)    errorFlag = true; // Reset FIFO and DMP
	gpioDelay(15000);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	if (i2cWriteByteData(ID, CONFIG, 0x01) != 0)      errorFlag = true;  // Set low-pass filter to 188 Hz
	if (i2cWriteByteData(ID, SMPLRT_DIV, 0x00) != 0)  errorFlag = true;  // Set sample rate to 1 kHz
	if (i2cWriteByteData(ID, GYRO_CONFIG, 0x00) != 0) errorFlag = true;  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	if (i2cWriteByteData(ID, ACCEL_CONFIG, 0x00) != 0) errorFlag = true; // Set accelerometer full-scale to 2 g, maximum sensitivity

	u_int16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  	u_int16_t  accelsensitivity = 16384;  // = 16384 LSB/g

	for (int ii=0; ii<100; ii++) {
		IMU::readRawIMU();
		accel_bias[0] += (int32_t) IMU::AccelData.X;
		accel_bias[1] += (int32_t) IMU::AccelData.Y;
		accel_bias[2] += (int32_t) IMU::AccelData.Z;

		gyro_bias[0]  += (int32_t) IMU::GyroData.X;
		gyro_bias[1]  += (int32_t) IMU::GyroData.Y;
		gyro_bias[2]  += (int32_t) IMU::GyroData.Z;
	}

	accel_bias[0] /= (int32_t) 100;
	accel_bias[1] /= (int32_t) 100;
	accel_bias[2] /= (int32_t) 100;
	gyro_bias[0]  /= (int32_t) 100;
	gyro_bias[1]  /= (int32_t) 100;
	gyro_bias[2]  /= (int32_t) 100;

	if(accel_bias[2] > 0L) {
		accel_bias[2] -= (int32_t) accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
	} else {
		accel_bias[2] += (int32_t) accelsensitivity;
	}

	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1]/4)       & 0xFF;
	data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2]/4)       & 0xFF;

	// Push gyro biases to hardware registers
	if (i2cWriteByteData(ID, XG_OFFSET_H, data[0]) != 0) errorFlag = true;
	if (i2cWriteByteData(ID, XG_OFFSET_L, data[1]) != 0) errorFlag = true;
	if (i2cWriteByteData(ID, YG_OFFSET_H, data[2]) != 0) errorFlag = true;
	if (i2cWriteByteData(ID, YG_OFFSET_L, data[3]) != 0) errorFlag = true;
	if (i2cWriteByteData(ID, ZG_OFFSET_H, data[4]) != 0) errorFlag = true;
	if (i2cWriteByteData(ID, ZG_OFFSET_L, data[5]) != 0) errorFlag = true;

	if (errorFlag) {
		cout << "Error during IMU Calibration" << endl;
	}

	printf("GyroBiases:\nX: %f °/s\nY: %f °/s\nZ: %f °/s\n",
			(float) gyro_bias[0]/(float) gyrosensitivity,
			(float) gyro_bias[1]/(float) gyrosensitivity,
			(float) gyro_bias[2]/(float) gyrosensitivity
	);

	int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
	accel_bias_reg[0] = read_raw_data(ID, XA_OFFSET_H); // Read factory accelerometer trim values
	accel_bias_reg[1] = read_raw_data(ID, YA_OFFSET_H);
	accel_bias_reg[2] = read_raw_data(ID, ZA_OFFSET_H);

	u_int32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  	u_int8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  
	for(int ii = 0; ii < 3; ii++) {
		if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0])      & 0xFF;
	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1])      & 0xFF;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2])      & 0xFF;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	// Push accelerometer biases to hardware registers this isn't working
	// i2cWriteByteData(ID, XA_OFFSET_H, data[0]);
	// i2cWriteByteData(ID, XA_OFFSET_L, data[1]);
	// i2cWriteByteData(ID, YA_OFFSET_H, data[2]);
	// i2cWriteByteData(ID, YA_OFFSET_L, data[3]);
	// i2cWriteByteData(ID, ZA_OFFSET_H, data[4]);
	// i2cWriteByteData(ID, ZA_OFFSET_L, data[5]);

	IMU::AccelBias.X = ((float) accel_bias[0]/(float) accelsensitivity);
	IMU::AccelBias.Y = ((float) accel_bias[1]/(float) accelsensitivity);
	IMU::AccelBias.Z = ((float) accel_bias[2]/(float) accelsensitivity);

	printf("AccelBiases:\nX: %f mg\nY: %f mg\nZ: %f mg\n",
			((float) accel_bias[0]/(float) accelsensitivity)*1000,
			((float) accel_bias[1]/(float) accelsensitivity)*1000,
			((float) accel_bias[2]/(float) accelsensitivity)*1000
	);
	gpioDelay(1000000); // short pause to display Biases
}

void IMU::readRawIMU() {
	/*Read raw value of Accelerometer and gyroscope from MPU9250*/
	int ID = IMU::identifier;
	int MAG_ID = IMU::mag_identifier;
	float MagData[3];

	IMU::AccelData.X = (float) read_raw_data(ID, ACCEL_XOUT_H);
	IMU::AccelData.Y = (float) read_raw_data(ID, ACCEL_YOUT_H);
	IMU::AccelData.Z = (float) read_raw_data(ID, ACCEL_ZOUT_H);
	
	IMU::GyroData.X  = (float) read_raw_data(ID, GYRO_XOUT_H);
	IMU::GyroData.Y  = (float) read_raw_data(ID, GYRO_YOUT_H);
	IMU::GyroData.Z  = (float) read_raw_data(ID, GYRO_ZOUT_H);

	// Read when data ready bit is set for magnetometer
	// remove block comment if using magnetometer
	/*
	if (i2cReadByteData(MAG_ID, AK8963_ST1) && 0x01) {
		MagData[0] = (float) read_raw_data(MAG_ID, AK8963_XOUT_H);
		MagData[1] = (float) read_raw_data(MAG_ID, AK8963_YOUT_H);
		MagData[2] = (float) read_raw_data(MAG_ID, AK8963_ZOUT_H);

		// If data overflow set then data is wrong		
		if (i2cReadByteData(MAG_ID, AK8963_ST2) && 0x08) {
			// printf("Wrong Magnetometer Data\n"); 
		} else {
			// printf("Correct Magnetometer Data\n");
			IMU::MagData.X = MagData[0];
			IMU::MagData.Y = MagData[1];
			IMU::MagData.Z = MagData[2];
		}
	}
	*/
}   

void IMU::applySensitivityFactor() {
	IMU::AccelData.X /= 16384.0f;
    IMU::AccelData.Y /= 16384.0f;
    IMU::AccelData.Z /= 16384.0f;

	pushNewData(&(IMU::AccX), IMU::AccelData.X);
	pushNewData(&(IMU::AccY), IMU::AccelData.Y);
	pushNewData(&(IMU::AccZ), IMU::AccelData.Z);

	//Remove AccelerometerBias
	// IMU::AccelData.X -= IMU::AccelBias.X;
	// IMU::AccelData.Y -= IMU::AccelBias.Y;
	// IMU::AccelData.Z -= IMU::AccelBias.Z;
        
    IMU::GyroData.X  /= 131.0f;
    IMU::GyroData.Y  /= 131.0f;
    IMU::GyroData.Z  /= 131.0f;

	//Magnetometer Calibration
	// remvoe comment block if using magnetometer
	/*
	float OffX, OffY, OffZ;
	OffX = IMU::MagData.X - IMU::MagBias.X;
	OffY = IMU::MagData.Y - IMU::MagBias.Y;
	OffZ = IMU::MagData.Z - IMU::MagBias.Z;

	IMU::MagData.X = IMU::MagCorrectionVal[0][0] * OffX + 
					 IMU::MagCorrectionVal[1][0] * OffY + 
					 IMU::MagCorrectionVal[2][0] * OffZ;

	IMU::MagData.Y = IMU::MagCorrectionVal[0][1] * OffX + 
					 IMU::MagCorrectionVal[1][1] * OffY + 
					 IMU::MagCorrectionVal[2][1] * OffZ;

	IMU::MagData.Z = IMU::MagCorrectionVal[0][2] * OffX + 
					 IMU::MagCorrectionVal[1][2] * OffY + 
					 IMU::MagCorrectionVal[2][2] * OffZ;

	float magNorm = sqrtf(powf(IMU::MagData.X,2) + powf(IMU::MagData.Y,2) + powf(IMU::MagData.Z,2));
	IMU::MagData.X /= magNorm;
	IMU::MagData.Y /= magNorm;
	IMU::MagData.Z /= magNorm;
	*/

}

/*--------------------------------------Functions for IMU-----------------------------------*/

// Setup all the AD0 pins to output
void setupMultiplexer() {
	gpioSetMode(5, PI_OUTPUT);
	gpioSetMode(6, PI_OUTPUT);

	gpioWrite(5, PI_LOW);
	gpioWrite(6, PI_LOW);
}
/*
// Selects which IMU to use
void selectIMU(unsigned char byte) {
	// unsigned char AD0Select = 1;
	switch((int) byte){
		case 1: {
			gpioWrite(5, PI_LOW);
			gpioWrite(6, PI_HIGH);
			break;
		}
		case 2: {
			gpioWrite(5, PI_HIGH);
			gpioWrite(6, PI_LOW);
			break;
		}
		default: {
			gpioWrite(5, PI_LOW);
			gpioWrite(6, PI_LOW);
			break;
		}
	}
	// gpioWrite(5, (int) (AD0Select ^ byte)); //0
	// gpioWrite(6, (int) ((AD0Select<<1) ^ byte));//2
}
*/

void getIMUData(IMU *IMU) {
    IMU->readRawIMU();
    IMU->applySensitivityFactor();
}
/*
float getElapsedTime(struct timespec *prev) {
    enum {NS_PER_SECOND = 1000000000};
    struct timespec now, td;

    clock_gettime(CLOCK_MONOTONIC, &now);
    td.tv_sec  = now.tv_sec - prev->tv_sec;
    td.tv_nsec = now.tv_nsec - prev->tv_nsec; 
    
    if (td.tv_sec > 0 && td.tv_nsec < 0) {
        td.tv_nsec += NS_PER_SECOND;
        td.tv_sec--;
    } 

    float elapsedTime = (float) td.tv_sec + (float) (td.tv_nsec * 1e-9);

    prev->tv_sec = now.tv_sec;
    prev->tv_nsec = now.tv_nsec;

    return elapsedTime;
}
*/

void computeYaw(IMU *IMU) {
	float roll = atan2(IMU->AccelData.Y, IMU->AccelData.Z);
	float az2 = IMU->AccelData.Y * sin(roll) +
			    IMU->AccelData.Z * cos(roll);
	float pitch = atan2(-IMU->AccelData.X, az2);

	// printf("Roll: %.2f,Pitch: %.2f\n",
	// 		roll*(180.0f/M_PI), 
	// 		pitch*(180.0f/M_PI)
	// );
	// remove comment block below if using magnetometer
	/*
	float my2 = IMU->MagData.Z * sin(roll) - 
				IMU->MagData.Y * cos(roll);
	float mz2 = IMU->MagData.Y * sin(roll) +
				IMU->MagData.Z * cos(roll);
	float mx3 = IMU->MagData.X * cos(pitch) +
				mz2 * sin(pitch);
	IMU->EulerAngles.Yaw = atan2(my2, mx3);
	*/
}

void computeEulerAngles(IMU *IMU) {
	float vector[3] = {IMU->AccelData.X, IMU->AccelData.Y, IMU->AccelData.Z};
	normalizeVectors(vector);

	float Ax2 = powf(IMU->AccelData.X, 2);
	float Ay2 = powf(IMU->AccelData.Y, 2);
	float Az2 = powf(IMU->AccelData.Z, 2);

	// IMU->EulerAngles.Roll  = atan2(IMU->AccelData.Y, sqrtf(Ax2 + Az2));
	// IMU->EulerAngles.Pitch = atan2(IMU->AccelData.X, sqrtf(Ay2 + Az2));

	
	
	IMU->EulerAngles.Roll  = atan2(IMU->AccelData.Y, IMU->AccelData.Z);
	IMU->EulerAngles.Pitch = atan2(IMU->AccelData.X, IMU->AccelData.Z);
		
	computeYaw(IMU);
	IMU->EulerAngles.Roll *= (180.0f / M_PI);
	IMU->EulerAngles.Pitch *= (180.0f / M_PI);
	IMU->EulerAngles.Yaw *= (180.0f / M_PI);
	// printf("Roll: %.2f,Pitch: %.2f\n",
	// 		IMU->EulerAngles.Roll, 
	// 		IMU->EulerAngles.Pitch 
	// );
}

void applyKalmanFiltering(IMU *IMU) {
	IMU->EulerAngles.Roll = IMU->Kalman.Roll.getAngle(
							IMU->EulerAngles.Roll,
							IMU->GyroData.X,
							IMU->deltat
	);
	IMU->EulerAngles.Pitch = IMU->Kalman.Pitch.getAngle(
							 IMU->EulerAngles.Pitch,
							 -IMU->GyroData.Y,
							 IMU->deltat
	);
	// IMU->EulerAngles.Yaw = IMU->Kalman.Yaw.getAngle(
	// 						 IMU->EulerAngles.Yaw,
	// 						 -IMU->GyroData.Z,
	// 						 IMU->deltat
	// );
}

void readIMU(IMU *IMU, int IMU_NUM) {
	extern int prevVal, currVal;
	extern int elapsedS, elapsedMS;
	extern int startS, startMS;
	extern int currentS, currentMS;
	extern double timeMS;
    // selectIMU(IMU_NUM);
    int ID;
    IMU->getIdentifier(&ID);
    if(i2cReadByteData(ID, INT_STATUS) && 0x01) {
            getIMUData(IMU);
    }
	IMU->lowPassFilter();
    //IMU->deltat = getElapsedTime(&IMU->now);   // set integration time by time elapsed since last filter update
	IMU->deltat = (float) ((double) elapsedS + timeMS);
	// Apply Kalman Filtering
	// Compute current Angle
	computeEulerAngles(IMU);
	applyKalmanFiltering(IMU);

	// for magnetometer
	/*
	switch(IMU_NUM) {
		case 1: IMU->EulerAngles.Yaw = -40.0f; break;
		case 2: IMU->EulerAngles.Yaw = 0.0f; break;
		default: IMU->EulerAngles.Yaw = 0.0f; break;
	}*/
}

// initKalmanValues initializes the kalman values of the IMU
void initKalmanValues(IMU *IMU, int IMU_NUM) {
	//Read values 200 times
	float Ax = 0.0f, Ay = 0.0f, Az = 0.0f;
	float Roll, Pitch;

	for (int i=0; i<200; i++) {
		readIMU(IMU, IMU_NUM);
		Ax += IMU->AccelData.X;
		Ay += IMU->AccelData.Y;
		Az += IMU->AccelData.Z;
	}
	// Get Average Values
	Ax /= 200.0f;	Ay /= 200.0f;	Az /= 200.0f;

	// Initial Roll and Pitch Values
	// Roll  = atan2(Ay, sqrtf(powf(Ax,2) + powf(Az,2)));
	// Pitch = atan2(Ax, sqrtf(powf(Ay,2) + powf(Az,2)));

	Roll  = atan2(Ay, sqrtf(powf(Ax,2) + powf(Az,2)));
	// Roll = atan2(Ay, Az);
	Pitch = atan2(Ax, Az);

	// Convert radians to degrees
	Roll  *= (180.0f/M_PI);		
	Pitch *= (180.0f/M_PI);

	printf("Initial Euler IMU[%d] Roll, Pitch: %.2f, %.2f\n", IMU_NUM, Roll, Pitch);

	// Set initial Roll and Pitch to Kalman Filter
	IMU->Kalman.Roll.setAngle(Roll);
	IMU->Kalman.Pitch.setAngle(Pitch);

	// Initial GVectors
	Ax = 0.0f; Ay = 0.0f; Az = 1.0f;
	float vector[3] = {Ax, Ay, Az};
	normalizeVectors(vector);
	IMU->InitialGVector.X = vector[0];
	IMU->InitialGVector.Y = vector[1];
	IMU->InitialGVector.Z = vector[2];
	printf("Initial GVector IMU[%d] X, Y, Z: %lf, %lf, %lff\n", 
			IMU_NUM, 
			vector[0], vector[1], vector[2]
	);
}

void initDevice(IMU *IMU, int IMU_NUM) {
    // selectIMU(IMU_NUM);
    
    
	int ID = i2cOpen(I2C_DEVICE, Device_Address, I2C_FLAGS); /*Initializes I2C with device Address*/
	if (ID < 0) {
		cout << "Failed to open i2c communication to IMU" << endl;
	}

	int MAG_ID = i2cOpen(I2C_DEVICE,AK8963_ADDRESS,I2C_FLAGS); /*Initializes Magnetometer I2C with device Address*/

	if (MAG_ID < 0) {
		cout << "Failed to open i2c communication to Magnetometer" << endl;
	}           

    IMU->setIdentifiers(ID, MAG_ID);
    IMU->calibrateMPU9250();                                    /* calibrates MPU-9250 gryo and accelerometer*/
    MPU9250_Init(ID);                                           /* initializes MPU9250 */
    
    // remove comment if using magnetometer
    // IMU->initCalibMagnetometer(IMU_NUM);                        /* initialize and calibrate magnetometer */

	//Get Kalman Initial Values
	initKalmanValues(IMU, IMU_NUM);
    clock_gettime(CLOCK_MONOTONIC, &IMU->now);                           
}

/*----------------------------------Operation Functions for IMU-----------------------------------*/
void normalizeVectors(float rV[3]) {
    float norm = sqrtf(pow(rV[0],2) +
                       pow(rV[1],2) +
                       pow(rV[2],2)
                 );
    rV[0] /= norm;
    rV[1] /= norm;
    rV[2] /= norm;
}

void vectorCross(float v[2][3], float rV[3]) {
    rV[0] = v[0][1] * v[1][2] - v[0][2] * v[1][1];
    rV[1] = v[0][2] * v[1][0] - v[0][0] * v[1][2];
    rV[2] = v[0][0] * v[1][1] - v[0][1] * v[1][0];
}

float vectorDotProd(float v1[3], float v2[3]) {
    float mag[2], vectorDotProd;
    
    vectorDotProd = v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
                          
    mag[0] = abs(sqrtf(pow(v1[0],2) + pow(v1[1],2) + pow(v1[2],2)));
    mag[1] = abs(sqrtf(pow(v2[0],2) + pow(v2[1],2) + pow(v2[2],2)));

    vectorDotProd = vectorDotProd / (mag[0] * mag[1]);
    return vectorDotProd;
}

// Get vector applying rotation matrix - roll
void rotMatrixRoll(IMU *IMU) {
	float angle = IMU->EulerAngles.Roll * (M_PI / 180.0f);

	IMU->AccelData.X = IMU->InitialGVector.X;
	IMU->AccelData.Y = ((IMU->InitialGVector.Y * cos(angle)) + 
					    (IMU->InitialGVector.Z * sin(angle)));
	IMU->AccelData.Z = ((IMU->InitialGVector.Y * -sin(angle)) + 
					    (IMU->InitialGVector.Z * cos(angle)));
}

// Get vector applying rotation matrix - pitch
void rotMatrixPitch(IMU *IMU) {
	float angle = IMU->EulerAngles.Pitch * (M_PI / 180.0f);
	float Ax = IMU->AccelData.X;

	IMU->AccelData.X = ((Ax * cos(angle)) + 
					   (IMU->AccelData.Z * sin(angle)));
	IMU->AccelData.Z = ((Ax * -sin(angle)) + 
					   (IMU->AccelData.Z * cos(angle)));
}

// Get vector applying rotation matrix - yaw
void rotMatrixYaw(IMU *IMU) {
	float angle = IMU->EulerAngles.Yaw * (M_PI / 180.0f);
	float Ax = IMU->AccelData.X;

	IMU->AccelData.X = (Ax * cos(angle)) +
					   (IMU->AccelData.Y * sin(angle));
	IMU->AccelData.Y = (Ax * -sin(angle)) +
					   (IMU->AccelData.Y * cos(angle));
	// printf("Yaw Angle: %.2f\n",angle * (180.0f/M_PI));
	// printf("rotYaw Ax, Ay, Az: %f, %f, %f\n", IMU->AccelData.X, IMU->AccelData.Y, IMU->AccelData.Z); 
}

void applyRotMatrix(IMU *IMU){
	float rV[3] = {IMU->AccelData.X, IMU->AccelData.Y, IMU->AccelData.Z};
	normalizeVectors(rV);
	rotMatrixRoll(IMU);
	rotMatrixPitch(IMU);
	rotMatrixYaw(IMU);
	normalizeVectors(rV);
}

float computeAngle(IMU* IMU) {
    float resultVector[2][3];
    float dotProd;

	applyRotMatrix(&IMU[0]);
	applyRotMatrix(&IMU[1]);

    float vector_IMU1[2][3] = {
        {IMU[0].AccelData.X, IMU[0].AccelData.Y, IMU[0].AccelData.Z},
        {0.0f, 1.0f, 0.0f}
    };

    float vector_IMU2[2][3] = {
        {IMU[1].AccelData.X, IMU[1].AccelData.Y, IMU[1].AccelData.Z},
        {0.0f, 1.0f, 0.0f}
    };
    vectorCross(vector_IMU1, resultVector[0]);
    vectorCross(vector_IMU2, resultVector[1]);
    normalizeVectors(resultVector[0]);
    normalizeVectors(resultVector[1]);
    
    dotProd = vectorDotProd(resultVector[0], resultVector[1]);

    return acos(dotProd);
}

float conv(queue<float> Acc, float* h) {
	float y = 0;
	int i = 6;
	
	while(!Acc.empty()) {
		y += Acc.front() * h[i];
		Acc.pop();
		i--;
	}

	return y;
}

void IMU::lowPassFilter() {
	float h[7] = {
		2.363158109348752e-02,
		9.279382700281774e-02,
		2.323198039224572e-01,
		3.025095759624751e-01,
		2.323198039224572e-01,     
		9.279382700281774e-02,
		2.363158109348752e-02
	};

	IMU::AccelData.X = conv(IMU::AccX, h);
	IMU::AccelData.Y = conv(IMU::AccY, h);
	IMU::AccelData.Z = conv(IMU::AccZ, h);

	// float rV[3] = {IMU::AccelData.X, IMU::AccelData.Y, IMU::AccelData.Z};
	// normalizeVectors(rV);
}	

