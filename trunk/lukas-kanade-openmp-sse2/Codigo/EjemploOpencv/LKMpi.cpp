#include "LKMpi.h"
#include "gui.h"
#include "video.h"
#include "task.h"
#include "LkPyramidFacade.h"
#include "PaintUtils.h"

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
	currentTask.println("frames: %d, numero procesos %d",frames_per_process,nproc);

	if(currentTask.GetProcessId()==0)
	{
		std::list<IplImage*> l;
		int destinyproc=0;
		for(int destinyproc=1;destinyproc<nproc ;destinyproc++)
		{
			for(int i=0;i<frames_per_process+1 ;i++)
			{
				IplImage* imgA=cvCreateImage(video.GetSize(),IPL_DEPTH_8U, 1);

				video.GoToCurrentFrame();		
				video.GetFrameSnapshot(*imgA);
				video.NextFrame();
				l.push_back(imgA);
				currentTask.println("enviando imagen a proceso %d..",destinyproc);
				
				SendImageThroughMPI(imgA,destinyproc);
			}
			video.PreviousFrame();
		}

		
		GUI g;
		g.Initialize(video.GetSize());
		//list<LKPiramidResults> resultsLk;
		video.Restart();		
		

		for(int source=1;source<nproc ;source++)
		{
			for(int i=0;i<frames_per_process ;i++)
			{
				LKPiramidResults res;
				MPI_Status result;
				currentTask.println("esperando resultado del proceso %d",source);
				MPI_Recv(&res,sizeof(LKPiramidResults),MPI_CHAR,source,0,MPI_COMM_WORLD,&result);
				currentTask.println("resultado obtenido[cancelled:%d count:%d error:%d]",result.cancelled,result.count,result.MPI_ERROR);
				
				video.GoToCurrentFrame();
				//PintarPiramide(g.GetWindowBackground(),res);
				g.Refresh();
				cvWaitKey(100);
				video.NextFrame();
			}
		}

		
		
	}
	else
	{
		std::list<IplImage*> l;
		for(int i=0;i<frames_per_process+1 ;i++)
		{
			IplImage* imgA=ReceiveImageThroughMPI(0);
			l.push_back(imgA);
		}

		list<IplImage*>::const_iterator it = l.begin();
		IplImage* first,*second;
		first=*it;
		it++;
		while(it!=l.end())
		{
			second=first;
			first=*it;
			
			LKPiramidResults lkData;
			MPI_Request result;

			CalcularLKPiramid(*first,*second,window_size,level,number_of_features_to_track,a,lkData);
			currentTask.println("enviando resultado: %d",sizeof(LKPiramidResults));
			MPI_Send(&lkData,sizeof(LKPiramidResults),MPI_CHAR,0,0,MPI_COMM_WORLD);

			it++;
		}
	}
	
	currentTask.println("esperando al resto de procesos para terminar");
	MPI_Barrier(MPI_COMM_WORLD);
	currentTask.println("Bye World!\n");
	MPI_Finalize();
}