#include "LkPyramidFacade.h"

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

	Cronometro c1,c2,c3;
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
	else if(a==algoritmo::LKpyramidalPAA_OpenMP)
	{
		for(int i=0;i<numeroRepeticiones;i++)
		{
			c3.Start();
			cvCalcOpticalFlowPyrLK_paa_omp(&frameA, &frameB, pyramid1, pyramid2, frameA_features, frameB_features, 
				number_of_features, optical_flow_window, level, optical_flow_found_feature, optical_flow_feature_error, 
				optical_flow_termination_criteria, 0 );

			c3.Stop();
			c3.Reset();
		}
		c3.PrintMinimumTime("");
	}

	cvReleaseImage(&pyramid1);
	cvReleaseImage(&pyramid2);
}
