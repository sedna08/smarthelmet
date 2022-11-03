#ifndef CV_H_
#define CV_H_

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <time.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

class CV {
    
    public: 
        float PERCLOS;
        struct EyeData {
            int radius;
        };
        struct FrameData {
            static const int numOfFrames = 960; // num of frames = 1 minute @ avg 16fps
            float numOfPixels[numOfFrames]; // num of black/white pixels per frame
            int state[numOfFrames]; // frameData.state per frame
            int counter;
            float prevFrameWhitePixelNo;
            float currFrameWhitePixelNo;
            bool close;
        };

        EyeData eyeData;
        FrameData frameData;

        
        CV();
        Mat rotate(Mat src, double angle);
        void gammaCorrection(const Mat &src, Mat &dst, const float gamma);
        Rect getLeftmostEye(vector<Rect> &eyes);
        Vec3f getEyeball(Mat &eye, vector<Vec3f> &circles);
        Point stabilize( vector<Point> &points, int windowSize);
        Rect detectEyes(Mat &frame, CascadeClassifier &eyeCascade);
        void detectBlink(Mat &frame);
    
};
#endif
