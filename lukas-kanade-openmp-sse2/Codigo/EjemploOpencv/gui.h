#pragma once
#include "common.h"
#include "memoryUtils.h"

class GUI
{
private:
	IplImage* windowBackground;
public:
	GUI()
	{
		windowBackground=NULL;
	}
	~GUI()
	{
		if(windowBackground!=NULL)
			cvReleaseImage(&windowBackground);
	}
	IplImage GetWindowBackground()
	{
		return *windowBackground;
	}
	void Initialize(CvSize size)
	{
		windowBackground=NULL;
		allocateOnDemand( &windowBackground, size, IPL_DEPTH_8U, 3 );
		cvNamedWindow("Optical Flow", CV_WINDOW_AUTOSIZE);
	}
	void Refresh()
	{
		cvShowImage("Optical Flow", windowBackground);     
		
	}

	void Refresh(IplImage& background)
	{
		cvConvertImage(&background,windowBackground,CV_GRAY2RGB);
		cvShowImage("Optical Flow", windowBackground);     
	}	
};
