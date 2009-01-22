#include "common.h"

void PintarPiramide(IplImage& windowBackground,LKPiramidResults& LkResultData);
void PaintPoint(CvPoint p,CvPoint q,IplImage& frame,int line_thickness,CvScalar line_color,int arrowSize);
void PintarLK(IplImage& vx,IplImage& vy,IplImage& windowBackground);
void PintarFeatures(IplImage&,IplImage&,IplImage& ,CvPoint2D32f [],int);