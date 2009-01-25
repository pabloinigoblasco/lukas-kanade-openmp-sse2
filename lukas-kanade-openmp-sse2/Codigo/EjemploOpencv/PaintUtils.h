#include "common.h"

bool PintarPiramide(IplImage& windowBackground,LKPiramidResults& data,int minDistance=2,int maxDistance=8);
void PaintPoint(CvPoint p,CvPoint q,IplImage& frame,int line_thickness,CvScalar line_color,int arrowSize);
void PintarLK(IplImage& vx,IplImage& vy,IplImage& windowBackground);
void PintarFeatures(IplImage&,IplImage&,IplImage& ,CvPoint2D32f [],int);