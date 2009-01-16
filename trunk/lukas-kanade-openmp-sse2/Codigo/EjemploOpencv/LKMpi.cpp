#include "LKMpi.h"
#include "gui.h"

void lk_mpi (int argc, char **argv)
{
	double ancho, x, sum,tsum=0;
	int intervalos, i, nproc,iproc;
	if(MPI_Init(&argc, &argv)!=MPI_SUCCESS) 
		exit(1);

	MPI_Comm_size(MPI_COMM_WORLD, &nproc);
	MPI_Comm_rank(MPI_COMM_WORLD, &iproc);
	printf("hi from %d\n",iproc);

	if(iproc==0)
	{
		IplImage* imgA = cvLoadImage("imageA.bmp", true);
		int destino=1;
		SendImageThroughMPI(imgA,destino);
	}
	else if(iproc==1)
	{
		int fuente=0;
		IplImage* imgA=ReceiveImageThroughMPI(fuente);
		GUI g;
		g.Initialize(cvGetSize(imgA));
		g.Refresh(*imgA);
		cvWaitKey(0);
	}
	
	MPI_Barrier(MPI_COMM_WORLD);
	MPI_Finalize();
}