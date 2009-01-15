#include "common.h"
#include "gui.h"
#include <mpi.h>

struct ImageInfo
{
	int depth;
	int nchannels;
	int step;
	int width;
	int height;
	float* data;
};

//serializar imagen para enviarla por mpi
void getRawImage(IplImage* inputImage,ImageInfo& outImageInfo)
{
	CvSize size;
	cvGetRawData( inputImage, (uchar**)&outImageInfo.data, &outImageInfo.step,&size); 

	outImageInfo.width= size.width;
	outImageInfo.height=size.height;
	outImageInfo.nchannels=inputImage->nChannels;
	outImageInfo.depth=inputImage->depth;
}

//deserializar la imagen y recojerla por mpi
IplImage* ReCreateImageFromRaw(ImageInfo& info)
{
	IplImage* imgB =cvCreateImageHeader(cvSize(info.width,info.height),info.depth,info.nchannels);
	cvSetData(imgB,info.data,info.step);
	return imgB;
}



void PruebaSerializacion()
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

void lk_mpi (int argc, char **argv)
{
	double ancho, x, sum,tsum=0;
	int intervalos, i, nproc,iproc;
	if(MPI_Init(&argc, &argv)==MPI_SUCCESS) 
		exit(1);

	MPI_Comm_size(MPI_COMM_WORLD, &nproc);
	MPI_Comm_rank(MPI_COMM_WORLD, &iproc);


	/*
	intervalos = atoi(argv[1]);
	ancho = 1.0/(double) intervalos;
	for (i=iproc; i<intervalos;i+=nproc){
		x = (i+0.5)*ancho;
		sum=sum + 4.0/(1.0 + x*x);
	}

	sum*=ancho;
	*/

	MPI_Reduce(&sum,&tsum,1,MPI_DOUBLE,MPI_SUM,0,MPI_COMM_WORLD);
	MPI_Finalize();
}