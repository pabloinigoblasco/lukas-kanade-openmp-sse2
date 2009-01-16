#include "common.h"
#include "gui.h"
#include <mpi.h>


IplImage* ReceiveImageThroughMPI(int sourceProcess);
void SendImageThroughMPI(IplImage* img,int destinyProcess);

void lk_mpi (int argc, char **argv);
void PruebaSerializacion();