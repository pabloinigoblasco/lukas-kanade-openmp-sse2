#include "Contadorciclos.h"

inline void LeerCiclos (LARGE_INTEGER *cic)
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
inline void ImprimirCiclos (const char *pc, LARGE_INTEGER *cic)
{
	printf (pc);
	printf("%I64d\n",cic->QuadPart);
}	
inline void ImprimirTiempo (const char *pc, LARGE_INTEGER *cic)
{   
	if(cic->QuadPart>(FREC_GHZ*1E9))
		printf( "%.4f s\n", (float) cic->QuadPart/(FREC_GHZ*1E9));
	else
		if(cic->QuadPart>(FREC_GHZ*1E6))
			printf( "%.4f ms\n", (float) cic->QuadPart/(FREC_GHZ*1E6));
		else
			printf( "%.1f ns\n", (float) cic->QuadPart/FREC_GHZ );
	
}
inline void RestarCiclos (LARGE_INTEGER *c_dif, LARGE_INTEGER *c_final, LARGE_INTEGER *c_inic)
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

