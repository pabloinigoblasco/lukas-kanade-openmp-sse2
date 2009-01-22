#pragma once
#include "common.h"

class Video
{
private:
	string s;
	long current_frame ;
	long number_of_frames;
	CvCapture *input_video;
	CvSize frame_size;
	IplImage *Image ;

	IplImage* LoadFrame(int frame)
	{
		/* Go to the frame we want.  Important if multiple frames are queried in
		* the loop which they of course are for optical flow.  Note that the very
		* first call to this is actually not needed. (Because the correct position
		* is set outsite the for() loop.)
		*/
		
		if(frame>=number_of_frames)
			frame=number_of_frames-2;
		else if( frame<0)
			frame=0;

		cvSetCaptureProperty( input_video, CV_CAP_PROP_POS_FRAMES, frame );

		/* Get the next frame of the video.
		* IMPORTANT!  cvQueryFrame() always returns a pointer to the _same_
		* memory location.  So successive calls:
		* windowBackground = cvQueryFrame();
		* frame2 = cvQueryFrame();
		* frame3 = cvQueryFrame();
		* will result in (windowBackground == frame2 && frame2 == frame3) being true.
		* The solution is to make a copy of the cvQueryFrame() output.
		*/
		Image = cvQueryFrame( input_video );
		if (Image == NULL)
		{
			/* Why did we get a NULL frame?  We shouldn't be at the end. */
			fprintf(stderr, "Error: Hmm. The end came sooner than we thought.\n");
		}

		return Image;
	}

public:
	
	long GetFrameCount() const
	{
		return number_of_frames;
	}
	long GetCurrentFrame() const
	{
		return current_frame;
	}
	CvSize GetSize()const
	{
		return frame_size;
	}

	void Initialize(const char filename[])
	{
		s=filename;
		input_video= cvCaptureFromFile(filename);
		//input_video= cvCaptureFromAVI(filename);
		
		Image=NULL;
		current_frame=0;

		if (input_video == NULL)
		{
			/* Either the video didn't exist OR it uses a codec OpenCV
			* doesn't support.
			*/
			fprintf(stderr, "Error: Can't open video.\n");
		}

		/* Read the video's frame size out of the AVI. */

		this->frame_size.height =
			(int) cvGetCaptureProperty( input_video, CV_CAP_PROP_FRAME_HEIGHT );
		this->frame_size.width =
			(int) cvGetCaptureProperty( input_video, CV_CAP_PROP_FRAME_WIDTH );


		/* Go to the end of the AVI (ie: the fraction is "1") */
		cvSetCaptureProperty( input_video, CV_CAP_PROP_POS_AVI_RATIO, 1. );
		/* Now that we're at the end, read the AVI position in frames */
		number_of_frames = (int) cvGetCaptureProperty( input_video, CV_CAP_PROP_POS_FRAMES );
		/* Return to the beginning */
		cvSetCaptureProperty( input_video, CV_CAP_PROP_POS_FRAMES, 0. );
	}

	void GoToCurrentFrame()
	{
		LoadFrame(current_frame);
	}

	
	void GetFrameSnapshot(IplImage& copy)
	{
		this->GetFrameSnapshot(copy,0);
	}
	void GetFrameSnapshot(IplImage& copy,int offset)
	{
		this->LoadFrame(GetCurrentFrame()+offset);
		cvConvertImage(Image, &copy);
	}

	void Restart()
	{
		current_frame=0;
		GoToCurrentFrame();
	}
	bool PreviousFrame()
	{
		current_frame--;
		GoToCurrentFrame();
		if(current_frame<=0)
		{
			return false;
			current_frame=0;
		}
		return true;
	}

	bool NextFrame()
	{
		current_frame++;
		GoToCurrentFrame();
		if(current_frame>=number_of_frames)
		{
			current_frame=number_of_frames;
			return false;
		}
		return true;
	}



};

