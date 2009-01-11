#pragma once

#include <stdio.h>
#include <cv.h>
#include <highgui.h>
#include <math.h>

/* This is just an inline that allocates images.  I did this to reduce clutter in the
* actual computer vision algorithmic code.  Basically it allocates the requested image
* unless that image is already non-NULL.  It always leaves a non-NULL image as-is even
* if that image's size, depth, and/or channels are different than the request.
*/
inline static void allocateOnDemand( IplImage **img, CvSize size, int depth, int channels )
{
	if ( *img != NULL )	return;

	*img = cvCreateImage( size, depth, channels );
	if ( *img == NULL )
	{
		fprintf(stderr, "Error: Couldn't allocate image.  Out of memory?\n");
		exit(-1);
	}
}


void PintarPiramide(int number_of_features,CvPoint2D32f frame1_features[],CvPoint2D32f frame2_features[],IplImage& windowBackground);
void PaintPoint(CvPoint p,CvPoint q,IplImage& frame,int line_thickness,CvScalar line_color);
void PintarLK(IplImage& vx,IplImage& vy,IplImage& windowBackground);
void PintarFeatures(IplImage&,IplImage&,IplImage& ,CvPoint2D32f [],int);

static const double pi = 3.14159265358979323846;

inline static double square(int a)
{
	return a * a;
}
