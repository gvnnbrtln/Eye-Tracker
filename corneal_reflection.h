#pragma once

#include <iostream>
#include "ofxOpenCv.h"

using namespace std;
using namespace cv;

#ifndef PI
	#define PI 3.141592653589
#endif

#define MAX(x, y) ( x>y ? x : y )
#define MIN(x, y) ( x<y ? x : y )

extern int crar; // corneal reflection approximate radius

void RemoveCornealReflection(ofxCvGrayscaleImage& src, ofxCvGrayscaleImage& dest, Point sp, int window_size,
	int biggest_crr, Point& cr, int& crr);

void LocateCornealReflection(ofxCvGrayscaleImage& src, ofxCvGrayscaleImage& dest, Point sp, int window_size, 
	int biggest_crar, Point& cr, int& crr);

int FitCircleRadiusToCornealReflection(ofxCvGrayscaleImage& src, const Point& cr, int crar, int biggest_crar,
	const vector<double>& sin_array, const vector<double>& cos_array);

void InterpolateCornealReflection(ofxCvGrayscaleImage& src, const Point& cr, int crr, const vector<double>& sin_array,
	const vector<double>& cos_array);
