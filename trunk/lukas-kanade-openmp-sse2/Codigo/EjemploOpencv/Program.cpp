#include "common.h"
#include "strategies.h"
#include "video.h"


void TrackMPI(int argc, char **argv)
{
	char filename[]="Movie2B.avi";
	Video v;

	int temporal_window=10;
	v.Initialize(filename);
	lk_mpi (argc, argv,v,temporal_window,9,4,algoritmo::LKpyramidalPAA,400);
}
void TestGoodFeaturesToTrack()
{
	char filename[]="Movie2B.avi";
	Video v;
	v.Initialize(filename);

	TestGoodFeatures(v);
}
int main(int argc, char **argv)
{
	//TestGoodFeaturesToTrack();
	TrackMPI(argc,argv);
	//TrackVideo();
}




