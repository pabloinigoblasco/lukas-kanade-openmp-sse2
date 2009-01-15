#pragma once

#include <stdio.h>
#include <cv.h>
#include <highgui.h>
#include <math.h>
#include "contadorciclos.h"

void PintarPiramide(int number_of_features,CvPoint2D32f frame1_features[],CvPoint2D32f frame2_features[],IplImage& windowBackground,float optical_flow_feature_error[],char optical_flow_found_feature[]);
void PaintPoint(CvPoint p,CvPoint q,IplImage& frame,int line_thickness,CvScalar line_color,int arrowSize);
void PintarLK(IplImage& vx,IplImage& vy,IplImage& windowBackground);
void PintarFeatures(IplImage&,IplImage&,IplImage& ,CvPoint2D32f [],int);


static const double pi = 3.14159265358979323846;


enum algoritmo {LKpyramidalPAA,LKpyramidalPAA_OpenMP,LKpyramidalClassic};
enum executionMode{noDisplay,Display};
