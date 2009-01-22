#pragma once

#include "common.h"
#include "memoryUtils.h"

void CalcularLKPiramid(IplImage& frameA, IplImage& frameB,int windowsSize,int level,int number_of_features ,algoritmo a,LKPiramidResults &outData);