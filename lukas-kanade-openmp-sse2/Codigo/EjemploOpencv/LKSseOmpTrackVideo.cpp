#include "strategies.h"
#include "lkPyramidFacade.h"
#include "video.h"
#include "gui.h"
#include "memoryUtils.h"
#include "PaintUtils.h"

int EjemploVideo(Video& v,algoritmo a,executionMode mode)
{
	/* Create an object that decodes the input video stream. */
	GUI g;
	

	if(mode==executionMode::Display)
	{
		g.Initialize(v.GetSize());
	}

	CvSize tamanyoVentana={3,3};
	int number_of_features=400;

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
	//char filename[]="tree.avi";
	char filename[]="Movie2B.avi";
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