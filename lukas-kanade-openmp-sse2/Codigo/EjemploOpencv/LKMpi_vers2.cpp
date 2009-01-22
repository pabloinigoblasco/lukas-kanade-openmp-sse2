#include "LKMpi.h"
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

	/*if(MPI_Init(&argc, &argv)!=MPI_SUCCESS) 
	exit(1);*/	

	int l;
	if(MPI_Init_thread(&argc, &argv,1,&l)!=MPI_SUCCESS) 
		exit(1);

	Task currentTask;
	MPI_Comm_size(MPI_COMM_WORLD, &nproc);

	frames_per_process=frameBufferCount/nproc;

#pragma omp sections 
	{
#pragma omp section
		{
			currentTask.println("Hi world!");

			if(currentTask.GetProcessId()==0)
			{
				currentTask.println("master Activity");
				GUI g;
				g.Initialize(video.GetSize());

				std::list<IplImage*> l;
				int destinyproc=0;
				int dispached_frame_index=0;
				for(int destinyproc=0;destinyproc<nproc ;destinyproc++)
				{
					for(int i=0;i<frames_per_process+1 ;i++)
					{
						IplImage* imgA=cvCreateImage(video.GetSize(),IPL_DEPTH_8U, 1);

						video.GoToCurrentFrame();		
						video.GetFrameSnapshot(*imgA);
						l.push_back(imgA);
						currentTask.println("enviando imagen a proceso %d..",destinyproc);
						SendImageThroughMPI(imgA,destinyproc);
						dispached_frame_index++;

						video.GetFrameSnapshot(g.GetWindowBackground());
						g.Refresh();
						cvWaitKey(1);
						video.NextFrame();
					}
					video.PreviousFrame();
				}

				video.Restart();
				for(int source=1;source<nproc ;source++)
				{
					for(int i=0;i<frames_per_process ;i++)
					{
						LKPiramidResults res;
						MPI_Status result;
						currentTask.println("esperando resultado del proceso %d",source);
						MPI_Recv(&res,sizeof(LKPiramidResults),MPI_CHAR,source,msg::data,MPI_COMM_WORLD,&result);
						currentTask.println("resultado obtenido[cancelled:%d count:%d error:%d]",result.cancelled,result.count,result.MPI_ERROR);

						video.GetFrameSnapshot(g.GetWindowBackground());
						PintarPiramide(g.GetWindowBackground(),res);
						g.Refresh();
						cvWaitKey(1);
						video.NextFrame();
					}
				}

				bool endNotification=true;
				for(int destinyproc=1;destinyproc<nproc ;destinyproc++)
				{
					MPI_Request result;
					MPI_Isend(&endNotification,1,MPI_LOGICAL,destinyproc,msg::protocol_control,MPI_COMM_WORLD,&result);
					currentTask.println("enviando señal de terminado al proceso %d",destinyproc);
				}
			}
		}


#pragma omp section
		{
			currentTask.println("Hi world!");
			currentTask.println("slave Activity");
			IplImage* first,*second;

			MPI_Status status;
			MPI_Request result;

			second=ReceiveImageThroughMPI(0);

			bool end=false;
			MPI_Irecv(&end,1,MPI_LOGICAL,1,msg::protocol_control,MPI_COMM_WORLD,&result);

			do
			{
				first=second;
				second=ReceiveImageThroughMPI(0);

				LKPiramidResults lkData;
				currentTask.println("esperando nueva imagen que procesar: %d",sizeof(LKPiramidResults));
				CalcularLKPiramid(*first,*second,window_size,level,number_of_features_to_track,a,lkData);
				currentTask.println("enviando resultado: %d",sizeof(LKPiramidResults));
				MPI_Isend(&lkData,sizeof(LKPiramidResults),MPI_CHAR,0,msg::data,MPI_COMM_WORLD,&result);
				cvReleaseImage(&first);
				MPI_Irecv(&end,1,MPI_LOGICAL,1,msg::protocol_control,MPI_COMM_WORLD,&result);
			}
			while(!end);
		}

	}

	currentTask.println("esperando al resto de procesos para terminar");
	MPI_Barrier(MPI_COMM_WORLD);
	currentTask.println("Bye World!\n");
	MPI_Finalize();
}