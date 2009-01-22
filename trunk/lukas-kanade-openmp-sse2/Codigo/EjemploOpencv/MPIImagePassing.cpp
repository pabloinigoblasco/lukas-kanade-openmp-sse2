#include "strategies.h"
#include "gui.h"

struct ImageInfo
{
	int depth;
	int nchannels;
	int step;
	int width;
	int height;
	float* data;
	bool end;
};

//serializar imagen para enviarla por mpi
ImageInfo getRawImage(IplImage* inputImage)
{
	ImageInfo outImageInfo;
	CvSize size;
	cvGetRawData( inputImage, (uchar**)&outImageInfo.data, &outImageInfo.step,&size); 

	outImageInfo.width= size.width;
	outImageInfo.height=size.height;
	outImageInfo.nchannels=inputImage->nChannels;
	outImageInfo.depth=inputImage->depth;

	return outImageInfo;
}

//deserializar la imagen y recojerla por mpi
IplImage* ReCreateImageFromRaw(ImageInfo& info)
{
	IplImage* imgB =cvCreateImageHeader(cvSize(info.width,info.height),info.depth,info.nchannels);
	cvSetData(imgB,info.data,info.step);
	return imgB;
}


void SendImageThroughMPI(IplImage* img,int dest)
{
	ImageInfo info;
	MPI_Request r;

	if(img!=NULL)
	{
		info=getRawImage(img);
		info.end=false;
		MPI_Isend(&info,sizeof(info), MPI_CHAR, dest, 0, MPI_COMM_WORLD,&r);	
		int size=info.step*info.height;
		MPI_Send(info.data,size, MPI_CHAR, dest, 2, MPI_COMM_WORLD);
		//MPI_Isend(info.data,size, MPI_CHAR, dest, 2, MPI_COMM_WORLD,&r);
		cvReleaseImage(&img);
	}
	else
	{
		info.end=true;
		MPI_Isend(&info,sizeof(info), MPI_CHAR, dest, 0, MPI_COMM_WORLD,&r);	
	}
}

IplImage* ReceiveImageThroughMPI(int orig)
{
	ImageInfo info;
	MPI_Status stat;

	MPI_Recv(&info,sizeof(info),MPI_CHAR,orig,0,MPI_COMM_WORLD,&stat);
	if(!info.end)
	{
		int datosToReceive=info.height*info.step*4;
		info.data=(float*)malloc(datosToReceive*sizeof(float));
		MPI_Recv(info.data,datosToReceive,MPI_CHAR,orig,2,MPI_COMM_WORLD,&stat);
		IplImage* ret=ReCreateImageFromRaw(info);
		return  ret;
	}
	else
	{
		return NULL;
	}

}



void PruebaSerializacion()
{

	IplImage* imgA = cvLoadImage("imageA.bmp", true);

	ImageInfo info,info2;
	info=getRawImage(imgA);

	memcpy(&info2,&info,sizeof(ImageInfo));
	int size=info2.step*info.height;
	info2.data=(float*)malloc(size*sizeof(float));
	memcpy(info2.data,info.data,size);


	IplImage* imgB=ReCreateImageFromRaw(info2);
	GUI g;
	g.Initialize(cvGetSize(imgA));
	g.Refresh(*imgB);
	cvWaitKey(0);
	getchar();	
}
