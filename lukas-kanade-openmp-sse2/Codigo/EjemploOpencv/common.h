#pragma once

#include <stdio.h>
#include <cv.h>
#include <highgui.h>
#include <math.h>
#include "contadorciclos.h"

#include <cstdlib>
#include <list>
#include <mpi.h>
//#include <omp.h>
using namespace std;

#define REPEAT_ALGORITHM_FOR_CLOCK 3

struct LKPiramidResults
{
	#define MAX_TRACK_FEATURES 400
	CvPoint2D32f frameA_features[MAX_TRACK_FEATURES];
	CvPoint2D32f frameB_features[MAX_TRACK_FEATURES];
	float optical_flow_feature_error[MAX_TRACK_FEATURES];
	char optical_flow_found_feature[MAX_TRACK_FEATURES];
	int count;
};




static const double pi = 3.14159265358979323846;


enum algoritmo {LKpyramidalPAA,LKpyramidalPAA_OpenMP,LKpyramidalClassic};
enum executionMode{noDisplay,Display};
