#include "strategies.h"
#include "lkPyramidFacade.h"
#include "video.h"
#include "gui.h"
#include "memoryUtils.h"
#include "PaintUtils.h"

int EjemploVideo(Video& v,algoritmo a,executionMode mode,int endFrame)
{
	/* Create an object that decodes the input video stream. */
	GUI g;
	if(mode==executionMode::Display)
	{
		g.Initialize(v.GetSize());
	}

	int number_of_features=400;

	IplImage *frame1_1C = NULL, *frame2_1C = NULL;
	allocateOnDemand( &frame1_1C, v.GetSize(), IPL_DEPTH_8U, 1 );
	allocateOnDemand( &frame2_1C, v.GetSize(), IPL_DEPTH_8U, 1 );

	bool continuar=true;

	LKPiramidResults last;
	while(continuar)
	{
		v.GoToCurrentFrame();		
		v.GetFrameSnapshot(*frame1_1C);
		v.GetFrameSnapshot(*frame2_1C,1);
		LKPiramidResults lkData;

		//quality 0.01,distance 4
		CalcularLKPiramid(*frame1_1C,*frame2_1C,15,7,number_of_features,a,0.01,0.4,lkData);
		

		if(mode==executionMode::Display)
		{
			v.GetFrameSnapshot(g.GetWindowBackground());
			if(!PintarPiramide(g.GetWindowBackground(),lkData,1,50))
				PintarPiramide(g.GetWindowBackground(),last,1,50);
			g.Refresh();      
			int key_pressed = cvWaitKey(1);
		}
		last=lkData;

		continuar=v.NextFrame();

		if(endFrame!=-1)
			continuar=continuar & endFrame>v.GetCurrentFrame();
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
	int endFrame=1000;

	//printf("pulse una tecla para comenzar\n");
	//getchar();
	Cronometro cClassic,cPaa,cPaaOmp;
	
	printf("Piramide clásico\n\n");
	cClassic.Start();
	EjemploVideo(v,algoritmo::LKpyramidalClassic,executionMode::Display,endFrame);
	cClassic.Stop();
	cClassic.PrintTime("Tiempo total:\n");
/*
	v.Restart();
	printf("\n\nPiramide PAA optimizado sse2\n\n");
	cPaa.Start();
	EjemploVideo(v,algoritmo::LKpyramidalPAA,executionMode::noDisplay,endFrame);
	cPaa.Stop();
	cPaa.PrintTime("Tiempo total:\n");
	
	v.Restart();
	printf("\n\nPiramide PAA optimizado sse2+omp\n\n");
	cPaaOmp.Start();
	EjemploVideo(v,algoritmo::LKpyramidalPAA_OpenMP,executionMode::noDisplay,endFrame);
	cPaaOmp.Stop();
	cPaaOmp.PrintTime("Tiempo total:\n");
	*/
}


//		float pj[200];
//		float pi[200];
//		for(int i=0;i<200;i++)
//		{
//			pj[i]=pi[i]=i+1;
//			pj[i]=pj[i]*0.2;
//		}
//		float err=0;
//
//		int size=10;
//		float* pjp=pj;
//		float* pip=pi;
//		float* endpi=pi+((size/4)*4);
//		float* realendpi=pi+size;
//		float zeros[]={0,0,0,0};
//		float values[4];
//								
//							__asm
//							{
//								
//
//									mov eax, [pip]
//									mov ebx, [pjp]
//									mov ecx, [endpi]
//
//									movups xmm3,[zeros]
//_loop:
//								    cmp eax,ecx
//									jge _end
//									
//									movups xmm1, [eax]
//									movups xmm2, [ebx]
//									subps xmm1, xmm2
//									mulps xmm1, xmm1 //t[x+1]*t[x+1]
//									add eax,16
//									add ebx,16
//									addps xmm3, xmm1
//									jmp _loop
//_end:
//
//									movups [values],xmm3								
//									mov ecx,[realendpi]
//
//_loop2:								cmp eax,ecx
//									jge _end2
//									movss xmm1,[eax]
//									movss xmm2,[ebx]
//									subss xmm1,xmm2
//									movss xmm2,err
//									addss xmm1,xmm2
//									movss [err],xmm1
//
//									add eax,4
//									add ebx,4
//									jmp _loop2
//_end2:
//									
//
//
//								}
//	err+=values[0]+values[1]+values[2]+values[3];
							