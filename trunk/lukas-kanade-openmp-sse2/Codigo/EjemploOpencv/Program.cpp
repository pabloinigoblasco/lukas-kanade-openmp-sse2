#include "common.h"
#include "video.h"
#include "gui.h"
#include "lkPyramidFacade.h"
#include "memoryUtils.h"
#include "LKMpi.h"
#include "PaintUtils.h"

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


int EjemploVideo(Video& v,algoritmo a,executionMode mode)
{
	/* Create an object that decodes the input video stream. */
	GUI g;
	
	float optical_flow_feature_error[400];
	char optical_flow_found_feature[400];

	if(mode==executionMode::Display)
	{
		g.Initialize(v.GetSize());
	}

	CvSize tamanyoVentana={3,3};
	int number_of_features=400;
	CvPoint2D32f frame1_features[400];
	/* This array will contain the locations of the points from frame 1 in frame 2. */
	CvPoint2D32f frame2_features[400];
	IplImage *frame1_1C = NULL, *frame2_1C = NULL;
	allocateOnDemand( &frame1_1C, v.GetSize(), IPL_DEPTH_8U, 1 );
	allocateOnDemand( &frame2_1C, v.GetSize(), IPL_DEPTH_8U, 1 );

	bool continuar=true;

	while(continuar)
	{
		v.GoToCurrentFrame();		
		v.GetFrameSnapshot(*frame1_1C);
		v.GetFrameSnapshot(*frame2_1C,1);
		LKPiramidResults lkData;

		CalcularLKPiramid(*frame1_1C,*frame2_1C,9,1,number_of_features,a,lkData);

		if(mode==executionMode::Display)
		{
			v.GetFrameSnapshot(g.GetWindowBackground());
			PintarPiramide(g.GetWindowBackground(),lkData);
			g.Refresh();      
			int key_pressed = cvWaitKey(1);
		}

		continuar=v.NextFrame();
	}
	cvReleaseImage(&frame1_1C);
	cvReleaseImage(&frame2_1C);
	return 1;
}



void TrackVideo()
{
	char filename[]="tree.avi";
	//char filename[]="Movie2B.avi";
	Video v;
	v.Initialize(filename);

	printf("pulse una tecla para comenzar\n");
	//getchar();
	Cronometro cClassic,cPaa,cPaaOmp;
	
	printf("Piramide clásico\n\n");
	cClassic.Start();
	EjemploVideo(v,algoritmo::LKpyramidalClassic,executionMode::noDisplay);
	cClassic.Stop();
	cClassic.PrintTime("Tiempo total:\n");

	v.Restart();
	printf("\n\nPiramide PAA optimizado sse2\n\n");
	cPaa.Start();
	EjemploVideo(v,algoritmo::LKpyramidalPAA,executionMode::noDisplay);
	cPaa.Stop();
	cPaa.PrintTime("Tiempo total:\n");
	
	v.Restart();
	printf("\n\nPiramide PAA optimizado sse2+omp\n\n");
	cPaaOmp.Start();
	EjemploVideo(v,algoritmo::LKpyramidalPAA_OpenMP,executionMode::noDisplay);
	cPaaOmp.Stop();
	cPaaOmp.PrintTime("Tiempo total:\n");
}

int main(int argc, char **argv)
{
	//char filename[]="tree.avi";
	/*char filename[]="Movie2B.avi";
	Video v;

	int temporal_window=200;

	v.Initialize(filename);
	lk_mpi (argc, argv,v,temporal_window,9,4,algoritmo::LKpyramidalPAA,400);*/
	TrackVideo();
}




