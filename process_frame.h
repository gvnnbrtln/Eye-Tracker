#pragma once

#include <iostream>
#include "ofxOpenCv.h"
#include "corneal_reflection.h"
#include "ransac.h"

using namespace std;
using namespace cv;

#define FIX_PIXEL(x) ( (x)<0 ? 0 : ((x)>255 ? 255:(x)) )

const int FRAMEW = 640;
const int FRAMEH = 480;
const int FRAMERATE = 60;
const double beta = 0.2;
const int CROSS_LEN = 5; 
const Scalar WHITE(255, 255, 255);
const Scalar YELLOW(255, 255, 0);
const Scalar RED(255, 0, 0);
const Scalar BLUE(0, 0, 255);
const Scalar GREEN(0, 255, 0);
const int NUM_LED = 2;

extern unsigned char* pixel;
extern IplImage* pixel_ellipse;
extern vector<double> intensity_factor_horizontal;
extern vector<double> avg_intensity_horizontal;
extern Point corneal_reflection;
extern ofxCvGrayscaleImage frame_threshold;
extern int edge_threshold;
extern int rays;
extern int minimum_features;
extern int cr_windows_size;
extern int number_inliers;
extern Point pupil;

void Process(ofxCvGrayscaleImage& frame, Point& start, ofxCvColorImage& image);
void ReduceLineNoise(unsigned char* pixel);
void CalculateAvgIntenistyHorizontal(const unsigned char* pixel);
void DrawCross(IplImage* image, Point center, int square_len, Scalar color);
