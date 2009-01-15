#include "common.h"
#include "video.h"
#include "gui.h"
#include "lkPyramidFacade.h"
#include "memoryUtils.h"

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
	CvPoint2D32f frame1_features[400];
	/* This array will contain the locations of the points from frame 1 in frame 2. */
	CvPoint2D32f frame2_features[400];

	while(true)
	{

		cvCalcOpticalFlowLK( grayA,grayB, tamanyoVentana, velx, vely);
		CalcularLKPiramid(*grayA,*grayB,frame1_features,frame2_features,number_of_features,3,5,optical_flow_feature_error,optical_flow_found_feature,a);

		g.Refresh(*grayA);      
		cvCopyImage(imgA,&g.GetWindowBackground());

		PintarLK(*velx,*vely,g.GetWindowBackground());
		PintarPiramide(number_of_features,frame1_features,frame2_features,g.GetWindowBackground(),optical_flow_feature_error,optical_flow_found_feature);
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


int EjemploVideo(const char* nombreFichero,algoritmo a,executionMode mode)
{
	/* Create an object that decodes the input video stream. */
	GUI g;
	Video v;

	float optical_flow_feature_error[400];
	char optical_flow_found_feature[400];

	v.Initialize(nombreFichero);

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
		v.GetCurrentFrameCopy(*frame1_1C);
		v.GetFrameCopy(*frame2_1C,1);
		CalcularLKPiramid(*frame1_1C,*frame2_1C,frame1_features,frame2_features,number_of_features,9,1,optical_flow_feature_error,optical_flow_found_feature,a);

		if(mode==executionMode::Display)
		{
			v.GetCurrentFrameCopy(g.GetWindowBackground());
			PintarPiramide(number_of_features,frame1_features,frame2_features,g.GetWindowBackground(),optical_flow_feature_error,optical_flow_found_feature);
			g.Refresh();      
			int key_pressed = cvWaitKey(100);
		}

		continuar=v.NextFrame();
	}
	cvReleaseImage(&frame1_1C);
	cvReleaseImage(&frame2_1C);
	return 1;
}





int main(int argc, char **argv)
{
	
	//char filename[]="tree.avi";
	char filename[]="Movie2B.avi";

	printf("pulse una tecla para comenzar\n");
	getchar();
	Cronometro cClassic,cPaa;
	
	printf("Piramide clásico\n\n");
	cClassic.Start();
	EjemploVideo(filename,algoritmo::LKpyramidalClassic,executionMode::Display);
	cClassic.Stop();
	cClassic.PrintTime("Tiempo total:\n");


	printf("\n\nPiramide PAA optimizado\n\n");
	cPaa.Start();
	EjemploVideo(filename,algoritmo::LKpyramidalPAA,executionMode::Display);
	cPaa.Stop();
	cPaa.PrintTime("Tiempo total:\n");
	
	getchar();
	printf("Pulse una tecla para finalizar");
	//EjemploSimple();
	//Prueba();
}




