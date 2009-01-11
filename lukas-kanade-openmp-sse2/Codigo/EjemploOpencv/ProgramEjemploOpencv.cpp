#include "common.h"
#include "video.h"
#include "gui.h"

void PyramidLK(IplImage& frameA, IplImage& frameB,CvPoint2D32f frameA_features[],CvPoint2D32f frameB_features[],
			   int number_of_features, int windowsSize,int level )
{
	IplImage *eig_image = NULL, *temp_image = NULL, *pyramid1 = NULL, *pyramid2 = NULL;
	/* Shi and Tomasi Feature Tracking! */
	/* Preparation: Allocate the necessary storage. */
	allocateOnDemand( &eig_image,cvGetSize(&frameA), IPL_DEPTH_32F, 1 );
	allocateOnDemand( &temp_image, cvGetSize(&frameB), IPL_DEPTH_32F, 1 );

	/* The i-th element of this array is the error in the optical flow for the i-th feature
	* of windowBackground as found in frame 2.  If the i-th feature was not found (see the array above)
	* I think the i-th entry in this array is undefined.*/
	float optical_flow_feature_error[400];

	/* The i-th element of this array will be non-zero if and only if the i-th feature of
	* frame 1 was found in frame 2.*/
	char optical_flow_found_feature[400];


	/* Actually run the Shi and Tomasi algorithm!!
	* "frame1_1C" is the input image.
	* "eig_image" and "temp_image" are just workspace for the algorithm.
	* The first ".01" specifies the minimum quality of the features (based on the eigenvalues).
	* The second ".01" specifies the minimum Euclidean distance between features.
	* "NULL" means use the entire input image.  You could point to a part of the image.
	* WHEN THE ALGORITHM RETURNS:
	* "frame1_features" will contain the feature points.
	* "number_of_features" will be set to a value <= 400 indicating the number of feature points found.
	*/
	cvGoodFeaturesToTrack(&frameA, eig_image, temp_image, frameA_features, &number_of_features, .01, .01, NULL);


	/* This is the window size to use to avoid the aperture problem (see slide "Optical Flow: Overview"). */
	CvSize optical_flow_window = cvSize(windowsSize,windowsSize);

	/* This termination criteria tells the algorithm to stop when it has either done 20 iterations or when
	* epsilon is better than .3.  You can play with these parameters for speed vs. accuracy but these values
	* work pretty well in many situations.
	*/
	CvTermCriteria optical_flow_termination_criteria
		= cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 );

	/* This is some workspace for the algorithm.
	* (The algorithm actually carves the image into pyramids of different resolutions.)
	*/
	allocateOnDemand( &pyramid1,cvGetSize(&frameA), IPL_DEPTH_8U, 1 );
	allocateOnDemand( &pyramid2,cvGetSize(&frameA), IPL_DEPTH_8U, 1 );

	/* Actually run Pyramidal Lucas Kanade Optical Flow!!
	* "frame1_1C" is the first frame with the known features.
	* "frame2_1C" is the second frame where we want to find the first frame's features.
	* "pyramid1" and "pyramid2" are workspace for the algorithm.
	* "frame1_features" are the features from the first frame.
	* "frame2_features" is the (outputted) locations of those features in the second frame.
	* "number_of_features" is the number of features in the frame1_features array.
	* "optical_flow_window" is the size of the window to use to avoid the aperture problem.
	* "5" is the maximum number of pyramids to use.  0 would be just one level.
	* "optical_flow_found_feature" is as described above (non-zero iff feature found by the flow).
	* "optical_flow_feature_error" is as described above (error in the flow for this feature).
	* "optical_flow_termination_criteria" is as described above (how long the algorithm should look).
	* "0" means disable enhancements.  (For example, the second array isn't pre-initialized with guesses.)
	*/


	Cronometro c1,c2;
	
	c1.Start();
	cvCalcOpticalFlowPyrLK(&frameA, &frameB, pyramid1, pyramid2, frameA_features, frameB_features, 
		number_of_features, optical_flow_window, level, optical_flow_found_feature, optical_flow_feature_error, 
		optical_flow_termination_criteria, 0 );
	c1.Stop();
	c1.PrintTime("Tiempo que ha tardado en ejecutarse el algoritmo piramidal de opencv para esta imagen");

	c2.Start();
	cvCalcOpticalFlowPyrLK_paa(&frameA, &frameB, pyramid1, pyramid2, frameA_features, frameB_features, 
		number_of_features, optical_flow_window, level, optical_flow_found_feature, optical_flow_feature_error, 
		optical_flow_termination_criteria, 0 );
	c2.Stop();
	c1.PrintTime("Tiempo que ha tardado en ejecutarse el algoritmo piramidal mejorado para esta imagen");	

	cvReleaseImage(&pyramid1);
	cvReleaseImage(&pyramid2);
}


int EjemploDosFrames(void)
{
	GUI g;
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

	

	CvSize tamanyoVentana={3,3};

	int number_of_features=400;
		CvPoint2D32f frame1_features[400];
		/* This array will contain the locations of the points from frame 1 in frame 2. */
		CvPoint2D32f frame2_features[400];

	while(true)
	{
		
		cvCalcOpticalFlowLK( grayA,grayB, tamanyoVentana, velx, vely);
		PyramidLK(*grayA,*grayB,frame1_features,frame2_features,number_of_features,3,5);

		g.Refresh(*grayA);      
		//cvCopyImage(imgA,&g.GetWindowBackground());

		cvOr(imgA,imgB,&g.GetWindowBackground());

		PintarLK(*velx,*vely,g.GetWindowBackground());
		PintarPiramide(number_of_features,frame1_features,frame2_features,g.GetWindowBackground());
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
int EjemploVideo(void)
{
	/* Create an object that decodes the input video stream. */
	GUI g;
	Video v;
	v.Initialize("tree.avi");
	g.Initialize(v.GetSize());
	
	CvSize tamanyoVentana={3,3};
	int number_of_features=400;
		CvPoint2D32f frame1_features[400];
		/* This array will contain the locations of the points from frame 1 in frame 2. */
		CvPoint2D32f frame2_features[400];

	while(true)
	{
		static  IplImage *frame1_1C = NULL, *frame2_1C = NULL;

		v.GoToCurrentFrame();		
		/* Allocate another image if not already allocated.
		* Image has ONE channel of color (ie: monochrome) with 8-bit "color" depth.
		* This is the image format OpenCV algorithms actually operate on (mostly).
		*/
		allocateOnDemand( &frame1_1C, v.GetSize(), IPL_DEPTH_8U, 1 );
		allocateOnDemand( &frame2_1C, v.GetSize(), IPL_DEPTH_8U, 1 );

		v.GetCurrentFrameCopy(*frame1_1C);
		v.GetCurrentFrameCopy(g.GetWindowBackground());
		v.GetFrameCopy(*frame2_1C,1);
		
		
		PyramidLK(*frame1_1C,*frame2_1C,frame1_features,frame2_features,number_of_features,3,5);


		IplImage* vx,*vy;
		CvSize size = cvGetSize(frame1_1C);

		vx = cvCreateImage(size, IPL_DEPTH_32F, 1);
		vy = cvCreateImage(size, IPL_DEPTH_32F, 1);

		cvCalcOpticalFlowLK(frame1_1C,frame2_1C,tamanyoVentana,vx,vy);

		PintarLK(*vx,*vy,g.GetWindowBackground());
		PintarPiramide(number_of_features,frame1_features,frame2_features,g.GetWindowBackground());
		PintarFeatures(*vx,*vy,g.GetWindowBackground(),frame1_features,number_of_features);

		cvReleaseImage(&vx);
		cvReleaseImage(&vy);

		g.Refresh();      
		int key_pressed = cvWaitKey(0);

		/* If the users pushes "b" or "B" go back one frame.
		* Otherwise go forward one frame.
		*/
		if (key_pressed == 'b' || key_pressed == 'B')	
		{
			v.PreviousFrame();
		}
		else if (key_pressed=='n')
		{
			v.NextFrame();
		}
		else if(key_pressed=='l'&& tamanyoVentana.height<=13)
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
}


int main(void)
{
	EjemploDosFrames();
}




