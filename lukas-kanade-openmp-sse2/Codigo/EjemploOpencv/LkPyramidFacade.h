#pragma once

#include "common.h"
#include "memoryUtils.h"

#define REPEAT_ALGORITHM_FOR_CLOCK 1

void CalcularLKPiramid(IplImage& frameA, IplImage& frameB,CvPoint2D32f frameA_features[],CvPoint2D32f frameB_features[],
					   int number_of_features, int windowsSize,int level ,float optical_flow_feature_error[],
					   char optical_flow_found_feature[],algoritmo a);