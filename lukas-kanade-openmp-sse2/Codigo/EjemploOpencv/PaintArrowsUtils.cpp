#include "common.h"

void PintarLK(IplImage& vx,IplImage& vy,IplImage& windowBackground)
{
	int step=5;
	for(int i = 0; i < vx.height; i+=step)
	{
		for(int j=0;j<vx.width;j+=step)
		{
			int line_thickness = line_thickness = 1;
			CvScalar line_color = CV_RGB(255,0,0);

			CvPoint p,q;
			p.x = j;
			p.y = i;
			q.x =j + cvGetReal2D(&vx, i, j);
			q.y =i + cvGetReal2D(&vy, i, j);

			if(p.x!=q.x || p.y!=q.y)
				PaintPoint(p,q,windowBackground,line_thickness,line_color);

		}
	}
}

void PintarFeatures(IplImage& vx,IplImage& vy,IplImage& windowBackground,CvPoint2D32f frame1_features[],int number_of_features)
{
	for(int i = 0; i < number_of_features; i++)
	{
		int line_thickness=1;			
		CvScalar line_color = CV_RGB(0,255,0);

		CvPoint p,q;
		p.x = (int) frame1_features[i].x;
		p.y = (int) frame1_features[i].y;
		q.x =p.x+ cvGetReal2D(&vx, p.y,p.x);
		q.y =p.y + cvGetReal2D(&vy, p.y,p.x);

		PaintPoint(p,q,windowBackground,line_thickness,line_color);
	}
}

void PintarPiramide(int number_of_features,CvPoint2D32f frame1_features[],CvPoint2D32f frame2_features[],IplImage& windowBackground)
{
	/* For fun (and debugging :)), let's draw the flow field. */
	for(int i = 0; i < number_of_features; i++)
	{
		int line_thickness;				line_thickness = 1;
		/* CV_RGB(red, green, blue) is the red, green, and blue components
		* of the color you want, each out of 255.
		*/	
		CvScalar line_color;			line_color = CV_RGB(0,0,255);

		/* Let's make the flow field look nice with arrows. */

		/* The arrows will be a bit too short for a nice visualization because of the high framerate
		* (ie: there's not much motion between the frames).  So let's lengthen them by a factor of 3.
		*/
		CvPoint p,q;
		p.x = (int) frame1_features[i].x;
		p.y = (int) frame1_features[i].y;
		q.x = (int) frame2_features[i].x;
		q.y = (int) frame2_features[i].y;

		PaintPoint(p,q,windowBackground,line_thickness,line_color);
	}

}
void PaintPoint(CvPoint p,CvPoint q,IplImage& windowBackground,int line_thickness,CvScalar line_color)
{
	double angle;		
	angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
	double hypotenuse;	hypotenuse = sqrt( square(p.y - q.y) + square(p.x - q.x) );

	/* Here we lengthen the arrow by a factor of three. */
	q.x = (int) (p.x -  hypotenuse * cos(angle));
	q.y = (int) (p.y -  hypotenuse * sin(angle));

	/* Now we draw the main line of the arrow. */
	/* "windowBackground" is the frame to draw on.
	* "p" is the point where the line begins.
	* "q" is the point where the line stops.
	* "CV_AA" means antialiased drawing.
	* "0" means no fractional bits in the center cooridinate or radius.
	*/
	cvLine( &windowBackground, p, q, line_color, line_thickness, CV_AA, 0 );
	/* Now draw the tips of the arrow.  I do some scaling so that the
	* tips look proportional to the main line of the arrow.
	*/			
	p.x = (int) (q.x + 2 * cos(angle + pi / 4));
	p.y = (int) (q.y + 2 * sin(angle + pi / 4));
	cvLine( &windowBackground, p, q, line_color, line_thickness, CV_AA, 0 );
	p.x = (int) (q.x + 2 * cos(angle - pi / 4));
	p.y = (int) (q.y + 2 * sin(angle - pi / 4));
	cvLine( &windowBackground, p, q, line_color, line_thickness, CV_AA, 0 );
}
