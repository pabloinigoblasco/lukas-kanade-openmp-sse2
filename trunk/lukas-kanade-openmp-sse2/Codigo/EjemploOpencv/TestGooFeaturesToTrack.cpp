#include "common.h"
#include "video.h"
#include "gui.h"
#include "LKPyramidFacade.h"
#include "PaintUtils.h"

void TestGoodFeatures(Video& v)
{
	GUI g;

	int frameA=200;

	g.Initialize(v.GetSize());

	int tamanyoVentana=3;
	int niveles=1;
	int number_of_features=400;
	int frame_offset=4;

	IplImage *frame1_1C = NULL, *frame2_1C = NULL;
	allocateOnDemand( &frame1_1C, v.GetSize(), IPL_DEPTH_8U, 1 );
	allocateOnDemand( &frame2_1C, v.GetSize(), IPL_DEPTH_8U, 1 );


	LKPiramidResults lkData;

	char k='c';
	while(k!='f')
	{
		v.GetFrameSnapshot(*frame1_1C,frameA);
		v.GetFrameSnapshot(*frame2_1C,frameA+frame_offset);
		
		CalcularLKPiramid(*frame1_1C,*frame2_1C,tamanyoVentana,niveles,number_of_features,algoritmo::LKpyramidalPAA,lkData);
		PintarPiramide(*frame1_1C,lkData,0.1,20);

		g.Refresh(*frame1_1C);      
		printf("introduzca una tecla:\n");
		k=cvWaitKey(0);

		if(k=='p')
			tamanyoVentana+=2;
		else if(k=='ñ')
			tamanyoVentana-=2;
		else if(k=='o')
			niveles+=1;
		else if(k=='l')
			niveles-=1;
		else if(k=='i')
			frame_offset++;
		else if(k=='k')
			frame_offset--;
		else if(k=='u')
			frameA++;
		else if(k='j')
			frameA--;

		if(niveles<0)
			niveles=0;

		if(frame_offset<1)
			frame_offset=1;

		printf("------------------------------------\n");
		printf("tamaño de ventana: %d\n",tamanyoVentana);
		printf("niveles de piramide: %d\n",niveles);
		printf("Intervalo de frames[%d %d]: %d\n",frameA,frameA+frame_offset);
		printf("------------------------------------\n");
		printf("p mayor tamaño de ventana\n");
		printf("ñ menor tamaño de ventana\n");
		printf("o mayor numero de niveles de la piramide\n");
		printf("l menor numero de niveles de la piramide\n");
		printf("i mayor frame offset\n");
		printf("k menor frame offset\n");
		printf("u frame++\n");
		printf("j frame--\n");
		printf("------------------------------------\n");
	}

	cvReleaseImage(&frame1_1C);
	cvReleaseImage(&frame2_1C);
}