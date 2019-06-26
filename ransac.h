#pragma once

#include <iostream>
#include "ofxOpenCv.h"

using namespace std;
using namespace cv;

#ifndef PI
#define PI 3.141592653589
#endif

extern vector<double> pupil_parameters;
extern vector<Point_<double>*> edge_point;

void GetFiveRandomNum(int max, vector<int>& random_num);

bool SolveEllipse(const vector<double>& conic_parameters, vector<double>& ellipse_parameters);

int* PupilFittingInliers(unsigned char* image, int width, int height, int& return_max_inliers);

Point_<double>* NormalizeEdgePoint(double& dis_scale, Point_<double>& normalized_center);

void DenormalizeEllipseParameters(vector<double>& parameters, const vector<double>& normalized_parameters, double dis_scale,
	Point_<double> normalized_center);

void DestroyEdgePoint();

void StarburstPupilContourDetection(unsigned char* image, int width, int height, int threshold, int N, 
	int minimum_features, Point start_point);

void LocateEdgePoints(unsigned char* image, int width, int height, Point_<double> center, int dis, double angle_step,
	double angle_normal, double angle_spread, int threshold);

Point_<double> GetEdgeMean();
