#include "strategies.h"
#include "gui.h"
#include "video.h"
#include "task.h"
#include "LkPyramidFacade.h"
#include "PaintUtils.h"

enum msg{data,protocol_control};

void lk_mpi (int argc, char **argv,Video& video,int frameBufferCount,int window_size,int level,algoritmo a,int number_of_features_to_track)
{
	int frames_per_process;
	int intervalos, nproc;

	if(MPI_Init(&argc, &argv)!=MPI_SUCCESS) 
		exit(1);

	Task currentTask;
	MPI_Comm_size(MPI_COMM_WORLD, &nproc);

	currentTask.println("Hi world!");
	frames_per_process=frameBufferCount/nproc;

	if(currentTask.GetProcessId()==0)
	{
		currentTask.println("frames: %d, numero procesos %d",frames_per_process,nproc);
		GUI g;
		g.Initialize(video.GetSize());
		int dispached_frame_index=0;

		int last_dispached=1;

		for(int i=0;i<video.GetFrameCount()-1;i++)
		{
			//cada frameBufferCount*nproc-1 bloques de frames hacer:
			if(i%frameBufferCount==0)
			{
				for(int j=0;j<frameBufferCount;j++)
				{	
					int destinyproc=last_dispached++;
					if(last_dispached>=nproc)
						last_dispached=1;

					IplImage* imgA=cvCreateImage(video.GetSize(),IPL_DEPTH_8U, 1);
					if(i+j<video.GetFrameCount())
					{
						video.GetFrameSnapshot(*imgA,j);
						currentTask.println("enviando imagen a proceso %d el frame %d+%d ..",destinyproc,i,j);
						SendImageThroughMPI(imgA,destinyproc);

						if(i+j+1<video.GetFrameCount())
						{
							imgA=cvCreateImage(video.GetSize(),IPL_DEPTH_8U, 1);
							video.GetFrameSnapshot(*imgA,j+1);
							currentTask.println("enviando imagen a proceso %d el frame %d+%d ..",destinyproc,i,j+1);
							SendImageThroughMPI(imgA,destinyproc);
						}
						else
							SendImageThroughMPI(NULL,destinyproc);
					}
					else
						SendImageThroughMPI(NULL,destinyproc);



				}
			}

			int source=(i%(nproc-1))+1;
			currentTask.println("esperando resultado del proceso %d",source);

			LKPiramidResults res;
			MPI_Status status;
			MPI_Request result;

			MPI_Recv(&res,sizeof(LKPiramidResults),MPI_CHAR,source,msg::data,MPI_COMM_WORLD,&status);

			video.GetFrameSnapshot(g.GetWindowBackground());
			PintarPiramide(g.GetWindowBackground(),res);
			g.Refresh();
			cvWaitKey(1);
			video.NextFrame();
			video.GoToCurrentFrame();
		}

		for(int i=1;i<nproc;i++)
		{
			SendImageThroughMPI(NULL,i);
		}
	}
	else
	{		
		MPI_Status status;
		MPI_Request result;

		bool end=false;
		int framereceivedCount=0;
		do
		{
			IplImage* first,*second;
			first=ReceiveImageThroughMPI(0);
			if(first==NULL)
				break;
			framereceivedCount++;

			second=ReceiveImageThroughMPI(0);
			if(second==NULL)
				break;
			framereceivedCount++;

			LKPiramidResults lkData;
			currentTask.println("esperando nueva imagen que procesar: %d",sizeof(LKPiramidResults));
			CalcularLKPiramid(*first,*second,window_size,level,number_of_features_to_track,a,lkData);
			currentTask.println("enviando resultado: %d",sizeof(LKPiramidResults));
			MPI_Isend(&lkData,sizeof(LKPiramidResults),MPI_CHAR,0,msg::data,MPI_COMM_WORLD,&result);
			cvReleaseImage(&first);


		}
		while(!end);
	}

	currentTask.println("esperando al resto de procesos para terminar");
	MPI_Barrier(MPI_COMM_WORLD);
	currentTask.println("Bye World!\n");
	MPI_Finalize();
}


