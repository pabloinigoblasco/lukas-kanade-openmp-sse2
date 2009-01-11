#include <Windows.h>
#include <stdio.h>
#define FREC_GHZ (3.01)//Colocar la frecuencia de vuestro procesador

class Cronometro
{
	private:
		LARGE_INTEGER startTime;
		LARGE_INTEGER result;
		LARGE_INTEGER minimumTime;

		void LeerCiclos (LARGE_INTEGER *cic);
		void ImprimirCiclos (const char *pc, LARGE_INTEGER *cic);
		void ImprimirTiempo (const char *pc, LARGE_INTEGER *cic);
		void RestarCiclos (LARGE_INTEGER *c_dif, LARGE_INTEGER *c_final, LARGE_INTEGER *c_inic);
	public:
		Cronometro();
		void Start();
		void Stop();
		void Reset();
		void PrintTime(const char* message);
		void PrintCycles(const char* message);
		void PrintMinimumTime(const char* message);
		void PrintMinimumCycles(const char* message);
};