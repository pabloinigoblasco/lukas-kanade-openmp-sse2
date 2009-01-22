#include "common.h"
#include "video.h"
#include "gui.h"
#include "LKPyramidFacade.h"
#include "PaintUtils.h"

void TestGoodFeatures(Video& v)
{
	GUI g;
	
	int frameA=100;
	int frameB=101;

	g.Initialize(v.GetSize());
	
	CvSize tamanyoVentana={3,3};
	int number_of_features=400;

	IplImage *frame1_1C = NULL, *frame2_1C = NULL;
	allocateOnDemand( &frame1_1C, v.GetSize(), IPL_DEPTH_8U, 1 );
	allocateOnDemand( &frame2_1C, v.GetSize(), IPL_DEPTH_8U, 1 );

	bool continuar=true;

	while(continuar)
	{
		v.GoToCurrentFrame();		
		v.GetFrameSnapshot(*frame1_1C,frameA);
		v.GetFrameSnapshot(*frame2_1C,frameB);
		LKPiramidResults lkData;

		for(int i=3;i<17;i+=2)
		{
			tamanyoVentana.height=i;
			tamanyoVentana.width=i;
			CalcularLKPiramid(*frame1_1C,*frame2_1C,9,1,number_of_features,algoritmo::LKpyramidalPAA,lkData);
		}

		v.GetFrameSnapshot(g.GetWindowBackground());
		PintarPiramide(g.GetWindowBackground(),lkData);
		g.Refresh();      
		int key_pressed = cvWaitKey(500);
	}
	cvReleaseImage(&frame1_1C);
	cvReleaseImage(&frame2_1C);
}