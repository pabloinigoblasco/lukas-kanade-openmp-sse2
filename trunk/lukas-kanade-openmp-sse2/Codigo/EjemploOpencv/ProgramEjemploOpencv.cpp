#include "common.h"
#include "video.h"
#include "gui.h"
#include <stdlib.h>
#define REPEAT_ALGORITHM_FOR_CLOCK 1


enum algoritmo {LKpyramidalPAA,LKpyramidalClassic};
enum executionMode{noDisplay,Display};

void CalcularLKPiramid(IplImage& frameA, IplImage& frameB,CvPoint2D32f frameA_features[],CvPoint2D32f frameB_features[],
					   int number_of_features, int windowsSize,int level ,float optical_flow_feature_error[],
					   char optical_flow_found_feature[],algoritmo a)
{
	IplImage *eig_image = NULL, *temp_image = NULL, *pyramid1 = NULL, *pyramid2 = NULL;
	allocateOnDemand( &eig_image,cvGetSize(&frameA), IPL_DEPTH_32F, 1 );
	allocateOnDemand( &temp_image, cvGetSize(&frameB), IPL_DEPTH_32F, 1 );

	cvGoodFeaturesToTrack(&frameA, eig_image, temp_image, frameA_features, &number_of_features, .01, .01, NULL);
	CvSize optical_flow_window = cvSize(windowsSize,windowsSize);
	CvTermCriteria optical_flow_termination_criteria
		= cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 );

	allocateOnDemand( &pyramid1,cvGetSize(&frameA), IPL_DEPTH_8U, 1 );
	allocateOnDemand( &pyramid2,cvGetSize(&frameA), IPL_DEPTH_8U, 1 );

	Cronometro c1,c2;
	int numeroRepeticiones=REPEAT_ALGORITHM_FOR_CLOCK;

	if(a==algoritmo::LKpyramidalClassic)
	{
		for(int i=0;i<numeroRepeticiones;i++)
		{
			c1.Start();
			cvCalcOpticalFlowPyrLK(&frameA, &frameB, pyramid1, pyramid2, frameA_features, frameB_features, 
				number_of_features, optical_flow_window, level, optical_flow_found_feature, optical_flow_feature_error, 
				optical_flow_termination_criteria, 0 );
			c1.Stop();
			c1.Reset();
		}
		c1.PrintMinimumTime("");
	}

	else if(a==algoritmo::LKpyramidalPAA)
	{
		for(int i=0;i<numeroRepeticiones;i++)
		{
			c2.Start();
			cvCalcOpticalFlowPyrLK_paa(&frameA, &frameB, pyramid1, pyramid2, frameA_features, frameB_features, 
				number_of_features, optical_flow_window, level, optical_flow_found_feature, optical_flow_feature_error, 
				optical_flow_termination_criteria, 0 );

			c2.Stop();
			c2.Reset();
		}
		c2.PrintMinimumTime("");
	}

	cvReleaseImage(&pyramid1);
	cvReleaseImage(&pyramid2);
}



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



struct ImageInfo
{
	int depth;
	int nchannels;
	int step;
	int width;
	int height;
	float* data;
};

void getRawImage(IplImage* inputImage,ImageInfo& outImageInfo)
{
	CvSize size;
	cvGetRawData( inputImage, (uchar**)&outImageInfo.data, &outImageInfo.step,&size); 

	outImageInfo.width= size.width;
	outImageInfo.height=size.height;
	outImageInfo.nchannels=inputImage->nChannels;
	outImageInfo.depth=inputImage->depth;
}

IplImage* ReCreateImageFromRaw(ImageInfo& info)
{
	IplImage* imgB =cvCreateImageHeader(cvSize(info.width,info.height),info.depth,info.nchannels);
	cvSetData(imgB,info.data,info.step);
	return imgB;
}


void Prueba()
{
	ImageInfo info;
	IplImage* imgA = cvLoadImage("imageA.bmp", true);

	getRawImage(imgA,info);
	IplImage* imgB=ReCreateImageFromRaw(info);


	GUI g;
	g.Initialize(cvGetSize(imgA));
	g.Refresh(*imgB);
	cvWaitKey(0);
	getchar();	
}
int main(void)
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




