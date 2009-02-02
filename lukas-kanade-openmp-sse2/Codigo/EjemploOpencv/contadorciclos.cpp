#include "contadorciclos.h"

Cronometro::Cronometro()
{
	((unsigned int*)(&minimumTime))[0]= 9999999;
	((unsigned int*)(&minimumTime))[1]= 9999999;
	times=0;
	Reset();
}

void 
Cronometro::LeerCiclos (LARGE_INTEGER *cic)
{
	unsigned int t_alto, t_bajo;

	__asm { 
		//cpuid // instrucción serializing
		rdtsc
		mov t_bajo, eax
		mov t_alto, edx
	}
	cic->LowPart =t_bajo;
	cic->HighPart=t_alto;
}
void 
Cronometro::ImprimirCiclos (const char *pc, LARGE_INTEGER *cic)
{
	printf (pc);
	printf("%I64d\n",cic->QuadPart);
}	
void 
Cronometro::ImprimirTiempo (const char *pc, LARGE_INTEGER *cic)
{   
	if(pc!=NULL && strlen(pc)>0)
		printf("%s:",pc);

	if(cic->QuadPart>(FREC_GHZ*1E9))
		printf( "%.4f s\n", (float) cic->QuadPart/(FREC_GHZ*1E9));
	else
		if(cic->QuadPart>(FREC_GHZ*1E6))
			printf( "%.4f ms\n", (float) cic->QuadPart/(FREC_GHZ*1E6));
		else
			printf( "%.1f ns\n", (float) cic->QuadPart/FREC_GHZ );
	
}
void 
Cronometro::RestarCiclos (LARGE_INTEGER *c_dif, LARGE_INTEGER *c_final, LARGE_INTEGER *c_inic)
{
	unsigned int t0_alto=c_inic->HighPart, t0_bajo=c_inic->LowPart;
	unsigned int t1_alto=c_final->HighPart, t1_bajo=c_final->LowPart;
	unsigned int t_dif_alto, t_dif_bajo;

	__asm {
		mov eax, t1_bajo
		mov edx, t1_alto
		sub eax, t0_bajo
		sbb edx, t0_alto

		mov t_dif_bajo, eax
		mov t_dif_alto, edx
	}
	c_dif->HighPart=t_dif_alto;
	c_dif->LowPart =t_dif_bajo;
}

void Cronometro::SumarCiclos (LARGE_INTEGER *c_dif, LARGE_INTEGER *c_final, LARGE_INTEGER *c_inic)
{
	unsigned int t0_alto=c_inic->HighPart, t0_bajo=c_inic->LowPart;
	unsigned int t1_alto=c_final->HighPart, t1_bajo=c_final->LowPart;
	unsigned int t_dif_alto, t_dif_bajo;

	__asm {
		mov eax, t1_bajo
		mov edx, t1_alto
		add eax, t0_bajo
		adc edx, t0_alto

		mov t_dif_bajo, eax
		mov t_dif_alto, edx
	}
	c_dif->HighPart=t_dif_alto;
	c_dif->LowPart =t_dif_bajo;
}

void
Cronometro::Start()
{
	LeerCiclos(&startTime);
}

void
Cronometro::Stop()
{
	LARGE_INTEGER now;
	LeerCiclos(&now);

	LARGE_INTEGER r;
	RestarCiclos(&r,&now,&startTime);

	SumarCiclos(&result,&result,&r);

	
	if(result.HighPart<minimumTime.HighPart)
		minimumTime=result;
	if(result.HighPart==minimumTime.HighPart)
	{
		if(result.LowPart<minimumTime.LowPart)
			minimumTime=result;
	}
	times++;
}

void
Cronometro::Reset()
{
	((unsigned int*)(&startTime))[0]=0;
	((unsigned int*)(&startTime))[1]=0;

	((unsigned int*)(&result))[0]=0;
	((unsigned int*)(&result))[1]=0;
}

void 
Cronometro:: PrintTime(const char* message)
{
		if(!InhibeOutput)
	ImprimirTiempo(message,&result);
}

void 
Cronometro:: PrintCycles(const char* message)
{
	if(!InhibeOutput)
	ImprimirCiclos(message,&result);
}

void 
Cronometro::PrintMinimumTime(const char* message)
{
	if(!InhibeOutput)
	ImprimirTiempo(message,&minimumTime);
}

void 
Cronometro::PrintMinimumCycles(const char* message)
{
	if(!InhibeOutput)
	ImprimirCiclos(message,&minimumTime);
}