#include "video.h"
#include "gui.h"
#include "memoryUtils.h"
#include "strategies.h"
#include "PaintUtils.h"
#include "lkPyramidFacade.h"

int EjemploSimple(algoritmo a)
{
	GUI g;
	float optical_flow_feature_error[400];
	char optical_flow_found_feature[400];
	IplImage *imgA = NULL;
	IplImage *imgB = NULL;
	IplImage *grayA = NULL;
	IplImage *grayB = NULL;
	IplImage *velx = NULL;
	IplImage *vely = NULL;

	imgA = cvLoadImage("imageA.bmp", true);
	imgB = cvLoadImage("imageB.bmp", true);

	CvSize size = cvGetSize(imgA);
	g.Initialize(size);

	grayA = cvCreateImage(cvGetSize(imgA), IPL_DEPTH_8U, 1);
	grayB = cvCreateImage(cvGetSize(imgB), IPL_DEPTH_8U, 1);


	velx = cvCreateImage(size, IPL_DEPTH_32F, 1);   // cvCreateImage(cvGetSize(imgA),32,1);
	vely = cvCreateImage(size, IPL_DEPTH_32F, 1);

	cvCvtColor(imgA, grayA, CV_BGR2GRAY);
	cvCvtColor(imgB, grayB, CV_BGR2GRAY);

	printf("pulse una tecla\n");
	getchar();

	CvSize tamanyoVentana={3,3};

	int number_of_features=400;
	

	while(true)
	{
		cvCalcOpticalFlowLK( grayA,grayB, tamanyoVentana, velx, vely);
		
		LKPiramidResults lkData;
		CalcularLKPiramid(*grayA,*grayB,3,5,number_of_features, a,lkData);

		g.Refresh(*grayA);      
		cvCopyImage(imgA,&g.GetWindowBackground());

		PintarLK(*velx,*vely,g.GetWindowBackground());
		PintarPiramide(g.GetWindowBackground(),lkData);
		g.Refresh();
		int key_pressed = cvWaitKey(0);

		/* If the users pushes "b" or "B" go back one frame.
		* Otherwise go forward one frame.
		*/
		if(key_pressed=='l'&& tamanyoVentana.height<=13)
		{
			tamanyoVentana.height+=2;
			tamanyoVentana.width+=2;
		}
		else if (key_pressed=='k'&& tamanyoVentana.height>=5)
		{
			tamanyoVentana.height-=2;
			tamanyoVentana.width-=2;
		}

	}
	return 0;	
}
