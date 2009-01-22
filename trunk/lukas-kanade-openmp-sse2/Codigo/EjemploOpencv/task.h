#include "common.h"

class Task
{
private:
	int currentProcId;
public:
	Task()
	{
		MPI_Comm_rank(MPI_COMM_WORLD, &currentProcId);
	   

	}
	void println(const char* message,...)
	{
		printf("[proc %d ]",currentProcId);
		va_list ap;
		va_start(ap,message);
		vprintf(message,ap);
		va_end(ap);		
		printf("\n");
	}
	int GetProcessId()
	{
		return this->currentProcId;
	}
};