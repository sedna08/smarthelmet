#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <time.h>
#include <stdlib.h>
#include "cv.h"

using namespace cv;
using namespace std;

CV::CV() {
    CV::PERCLOS = 0.0;

    CV::eyeData.radius = 0;

    CV::frameData.numOfPixels[CV::frameData.numOfFrames] = { 0 }; // num of black/white pixels per frame
    CV::frameData.state[CV::frameData.numOfFrames] = { 0 }; // CV::frameData.state per frame
    CV::frameData.counter = 0;
    CV::frameData.prevFrameWhitePixelNo = 0.0;
    CV::frameData.currFrameWhitePixelNo = 0.0;
    CV::frameData.close = false;
}

Mat CV::rotate(Mat src, double angle)   //rotate function returning mat object with parametres imagefile and angle    
{
    Mat dst;      //Mat object for output image file
    Point2f pt(src.cols/2., src.rows/2.);          //point from where to rotate    
    Mat r = getRotationMatrix2D(pt, angle, 1.0);      //Mat object for storing after rotation
    warpAffine(src, dst, r, Size(src.cols, src.rows));  ///applie an affine transforation to image.
    return dst;         //returning Mat object for output image file
}

void CV::gammaCorrection(const Mat &src, Mat &dst, const float gamma) {
    Mat gray;
    cvtColor(src, gray, COLOR_BGR2GRAY);

    float invGamma = 1 / gamma;
    Mat table(1, 256, CV_8U);
    uchar *p = table.ptr();
    for (int i = 0; i < 256; ++i) {
        p[i] = (uchar) (pow(i / 255.0, invGamma) * 255);
    }
    dst = src.clone();
    LUT(src, table, dst);
}

Rect CV::getLeftmostEye(vector<Rect> &eyes) {
    int leftmost = 99999999;
    int leftmostIndex = -1;
    for (int i = 0; i < eyes.size(); i++) {
        if (eyes[i].tl().x < leftmost) {
            leftmost = eyes[i].tl().x;
            leftmostIndex = i;
        }
    }
    return eyes[leftmostIndex];
}

Vec3f CV::getEyeball(Mat &eye, vector<Vec3f> &circles)
    {
    vector<int> sums(circles.size(), 0);
    for (int y = 0; y < eye.rows; y++)
    {
        uchar *ptr = eye.ptr<uchar>(y);
        for (int x = 0; x < eye.cols; x++)
        {
            int value = static_cast<int>(*ptr);
            for (int i = 0; i < circles.size(); i++)
            {
                Point center((int)round(circles[i][0]), (int)round(circles[i][1]));
                int radius = (int)round(circles[i][2]);
                if (pow(x - center.x, 2) + pow(y - center.y, 2) < pow(CV::eyeData.radius, 2))
                {
                    sums[i] += value;
                }
            }
            ++ptr;
        }
    }
    int smallestSum = 9999999;
    int smallestSumIndex = -1;
    for (int i = 0; i < circles.size(); i++)
    {
        if (sums[i] < smallestSum)
        {
            smallestSum = sums[i];
            smallestSumIndex = i;
        }
    }
    return circles[smallestSumIndex];
}

Point CV::stabilize( vector<Point> &points, int windowSize)
{
    float sumX = 0;
    float sumY = 0;
    int count = 0;
    for (int i = max(0, (int)(points.size() - windowSize)); i < points.size(); i++)
    {
        sumX += points[i].x;
        sumY += points[i].y;
        ++count;
    }
    if (count > 0)
    {
        sumX /= count;
        sumY /= count;
    }
    return Point(sumX, sumY);
}

Rect CV::detectEyes(Mat &frame, CascadeClassifier &eyeCascade) {
    Mat gray;
    gammaCorrection(frame, frame, 5); // brighten image
    cvtColor(frame, gray, COLOR_BGR2GRAY); // convert image to grayscale
    equalizeHist(gray, gray); // enchance image contrast
    // Detect Both Eyes
    vector<Rect> eyes;
    eyeCascade.detectMultiScale(gray, eyes, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(90, 90)); // eye size (Size(90,90)) is determined emperically based on eye distance
    if (eyes.size() != 2) { // if both eyes not detected
        cout << "Error: Both eyes not detected" << endl;
        return Rect(0, 0, 0, 0); // return empty rectangle
    }
    for (Rect &eye : eyes) {
        rectangle(frame, eye.tl(), eye.br(), Scalar(0, 255, 0), 2); // draw rectangle around both eyes
    }
    // Get Left-Most Eyes and Detect Iris
    vector<Point> centers;
    vector<Vec3f> circles;
    Mat eye = gray(getLeftmostEye(eyes));
    HoughCircles(eye, circles, HOUGH_GRADIENT, 1, eye.cols / 8, 250, 15, eye.rows / 8, eye.rows / 3);
    if (circles.size() > 0) {
        Vec3f eyeball = getEyeball(eye, circles);
        Point center(eyeball[0], eyeball[1]);
        centers.push_back(center);
        center = stabilize(centers, 5); // we are using the last 5
        CV::CV::eyeData.radius = (int)eyeball[2]; // get iris radius
        circle(eye, center, CV::eyeData.radius, Scalar(255, 255, 255), 2); // draw circle around the iris
    }
    else if (circles.size() <= 0) { // if eyeball not detected
        cout << "Error: Eyeball not detected" << endl;
        return Rect(0, 0, 0, 0); // return empty rectangle
    }

    line(eye, Point(0, getEyeball(eye, circles)[1]), Point(eye.cols, getEyeball(eye, circles)[1]), Scalar(255, 255, 255), 2, 8, 0);
    imshow("frame", frame);
    imshow("eye", eye);
    return getLeftmostEye(eyes);
}

void CV::detectBlink(Mat &frame) {
    // BGR to Binary
    Mat gray;
    cvtColor(frame, gray, COLOR_BGR2GRAY); // convert image to grayscale
    equalizeHist(gray, gray); // enchance image contrast
    Mat blur;
    GaussianBlur(gray, blur, Size(9, 9), 0); // blur image
    Mat thresh;
    threshold(blur, thresh, 20, 255, THRESH_BINARY_INV); // convert to binary image
    
    // Crop Sides to Remove Eyebrows etc.
    double crop_percent = 0.2;
    int x = thresh.cols * crop_percent;
    int y = thresh.rows * crop_percent;
    int src_w = thresh.cols * (1 - (crop_percent * 2));
    int src_h = thresh.rows * (1 - (crop_percent * 2));
    Mat crop = thresh(Rect(x, y, src_w, src_h)); // crop side to remove eyebrows etc.

    // Get Upper Half of Cropped Frame
    int upper_w = crop.cols;
    int upper_h = (int)((double)crop.rows * 0.50) + (int)((double)CV::CV::eyeData.radius * 0.3); // upper half and additional 3% of iris rad from the center should approximately include 80% of eyes.
    Mat upper = crop(Rect(0, 0, upper_w, upper_h)); // get upper half of image

    // Calculate Histogram
    int histSize = 256;
    float range[] = { 0, 256 }; // the upper boundary is exclusive
    const float* histRange[] = { range };
    bool uniform = true, accumulate = false;
    Mat hist;
    calcHist(&upper, 1, 0, Mat(), hist, 1, &histSize, histRange, uniform, accumulate); // get histogram
    
    // Compare Current and Previous Frames
    CV::frameData.prevFrameWhitePixelNo = CV::frameData.currFrameWhitePixelNo;
    CV::frameData.currFrameWhitePixelNo = hist.at<float>(255);
    float percentDiff = ((CV::frameData.prevFrameWhitePixelNo - CV::frameData.currFrameWhitePixelNo) / ((CV::frameData.prevFrameWhitePixelNo + CV::frameData.currFrameWhitePixelNo) / 2)) * 100;
    if (percentDiff >= 80.0) {
        CV::frameData.close = true;
    } 
    else if (percentDiff <= -20.0) {
        CV::frameData.close = false;
    }
    
    // Calculate CV::PERCLOS: P80
    CV::frameData.numOfPixels[CV::frameData.counter] = hist.at<float>(255);
    if (CV::frameData.close) { 
        CV::frameData.state[CV::frameData.counter] = 1;
    }
    CV::frameData.counter += 1;
    if (CV::frameData.counter == CV::frameData.numOfFrames) {
        // Get Number of Open and Closed States
        int closedStates = 0;
        for (int i = 0; i < CV::frameData.numOfFrames; i++) {
            closedStates += CV::frameData.state[i];
        }

        // CV::PERCLOS P:80
        CV::PERCLOS = (float)closedStates / CV::frameData.numOfFrames * 100;

        // SHL
        for (int i = 0; i < CV::frameData.numOfFrames; i++) {
            CV::frameData.numOfPixels[i] = CV::frameData.numOfPixels[i + 1];
            CV::frameData.state[i] = CV::frameData.state[i + 1];
        }
        CV::frameData.numOfPixels[CV::frameData.numOfFrames - 1] = 0;
        CV::frameData.state[CV::frameData.numOfFrames - 1] = 0;
        CV::frameData.counter = CV::frameData.numOfFrames - 1;
    }

    Point p1(x, y);
    Point p2(x+src_h, y+src_w);
    rectangle(gray, p1, p2, Scalar(0, 255, 0), 2);
    line(gray, Point(x, y+upper_h), Point(x+src_w, y+upper_h), Scalar(0, 0, 255), 2, 8, 0);
    imshow("gray", gray);
    imshow("crop", crop);
    imshow("upper", upper);
}
