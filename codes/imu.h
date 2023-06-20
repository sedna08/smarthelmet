#ifndef IMU_H_
#define IMU_H_

#define Device_Address 0x68	/*Device Address/Identifier for MPU9250*/

#define PWR_MGMT_1   0x6B
#define PWR_MGMT_2   0x6C
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define INT_ENABLE   0x38
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H  0x43
#define GYRO_YOUT_H  0x45
#define GYRO_ZOUT_H  0x47
#define FIFO_EN      0x23
#define I2C_MST_CTRL 0x24
#define USER_CTRL    0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define FIFO_COUNTH  0x72
#define XG_OFFSET_H  0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L  0x14
#define YG_OFFSET_H  0x15
#define YG_OFFSET_L  0x16
#define ZG_OFFSET_H  0x17
#define ZG_OFFSET_L  0x18
#define XA_OFFSET_H  0x77
#define XA_OFFSET_L  0x78
#define YA_OFFSET_H  0x7A
#define YA_OFFSET_L  0x7B
#define ZA_OFFSET_H  0x7D
#define ZA_OFFSET_L  0x7E
#define INT_PIN_CFG  0x37
#define ACCEL_CONFIG2 0x1D

//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L	 0x03  // data
#define AK8963_XOUT_H	 0x04
#define AK8963_YOUT_L	 0x05
#define AK8963_YOUT_H	 0x06
#define AK8963_ZOUT_L	 0x07
#define AK8963_ZOUT_H	 0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value
#define INT_STATUS       0x3A

#define I2C_DEVICE 1
#define I2C_FLAGS 0

#include <math.h>
#include <time.h>
#include <queue>
#include "kalmanFilter.h"
using namespace std;

//Class IMU
class IMU {
    private:
        int  identifier, mag_identifier;
        void initMagnetometer();
        void calibrateMagnetometer();
        void calibrateMagnetometer_SET(int IMU_NUM);

    public:
        IMU();
        struct Data {
            float X, Y, Z;
        };

        struct EuAngles {
            float Roll, Pitch, Yaw;
        };

        struct KalmanData {
            Kalman Roll, Pitch, Yaw;     //Roll - rot around the x-axis //Pitch - rot around y-axis
        };

        queue<float> AccX, AccY, AccZ;
        struct timespec now;
        float MagCorrectionVal[3][3];
        float deltat;
        void setIdentifiers(int ID, int MAG_ID);
        void getIdentifier(int *ID);
        void getMagIdentifier(int *MAG_ID);
        void calibrateMPU9250();
        void readRawIMU();
        void applySensitivityFactor();
        void initCalibMagnetometer(int IMU_NUM);
        void lowPassFilter();
        // void setOffsetQuaternions();

    Data AccelData;                 //Accelerometer Data
    Data InitialGVector;            //Initial GVector
    Data AccelBias;                 //Accelerometer Bias
    Data GyroData;                  //Gyroscope Data
    Data Angles;                    //Angle Computations
    Data MagData;                   //Magnetometer Data
    Data MagCalib;                  //Magnetometer Calibration
    Data MagBias, MagScale;         //Magnetometer Bias and Scale
    KalmanData Kalman;              //Kalman Data

    EuAngles EulerAngles;           //Initial Euler Angles
};

void pushNewData(queue<float> *q, float newData);
void populateQueue(queue<float> *q, int size);

//Function Prototypes
void MPU9250_Init(int ID);
void calculateUsingAccAngles(IMU *IMU);

short read_raw_data(int IMUIdentifier, int addr);


// Functions for IMU
void setupMultiplexer();
void selectIMU(unsigned char byte);
void getIMUData(IMU *IMU);
// float getElapsedTime(struct timespec *prev);
void readIMU(IMU *IMU, int IMU_NUM);
void initDevice(IMU *IMU, int IMU_NUM);


// Operation Functions
void normalizeVectors(float rV[3]);
float computeAngle(IMU* IMU);
float vectorDotProd(float v1[3], float v2[3]);
void vectorCross(float v[2][3], float rV[3]);
void readIMU(IMU *IMU, int IMU_NUM);
void initKalmanValues(IMU *IMU, int IMU_NUM);
void rotMatrixRoll(IMU *IMU);
void rotMatrixYaw(IMU *IMU);
void rotMatrixPitch(IMU *IMU);
void applyRotMatrix(IMU *IMU);
float computeAngle(IMU* IMU);
float conv(queue<float> Acc, float* h);

// Filter
void computeEulerAngles(IMU *IMU);
void applyKalmanFiltering(IMU *IMU);
void computeYaw(IMU *IMU);

#endif