#include "common.h"
#include "gui.h"

class Video;

IplImage* ReceiveImageThroughMPI(int sourceProcess);
void SendImageThroughMPI(IplImage* img,int destinyProcess);

void lk_mpi (int argc, char **argv,Video& video,int frameBufferCount,int window_size,int level,algoritmo a,int number_of_features_to_track);
void PruebaSerializacion();