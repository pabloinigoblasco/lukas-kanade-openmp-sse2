#include "LKMpi.h"

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

void SendImageThroughMPI(IplImage* img,int dest)
{
	ImageInfo info;
	getRawImage(img,info);

	MPI_Send(&info,sizeof(info), MPI_CHAR, dest, 0, MPI_COMM_WORLD);
	printf("Datos iniciales enviados, Enviando imagen..\n");
	
	int size=info.step*info.height;
	MPI_Send(info.data,size, MPI_CHAR, dest, 2, MPI_COMM_WORLD);
	printf("Imagen enviada..\n");

	cvReleaseImage(&img);
}
IplImage* ReceiveImageThroughMPI(int orig)
{
	ImageInfo info;
	MPI_Status stat;

	MPI_Recv(&info,sizeof(info),MPI_CHAR,orig,0,MPI_COMM_WORLD,&stat);

	int datosToReceive=info.height*info.step*4;
	info.data=(float*)malloc(datosToReceive*sizeof(float));
	MPI_Recv(info.data,datosToReceive,MPI_CHAR,orig,2,MPI_COMM_WORLD,&stat);
	printf("estado de la recepción cancelled=%d\n",stat.cancelled);
	printf("Imagen recibida.. %d\n",datosToReceive);
	return ReCreateImageFromRaw(info);
}

void PruebaSerializacion()
{

	IplImage* imgA = cvLoadImage("imageA.bmp", true);

	ImageInfo info,info2;
	getRawImage(imgA,info);

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
