#include <Windows.h>
#include <stdio.h>
#define FREC_GHZ (3.01)//Colocar la frecuencia de vuestro procesador


void inline LeerCiclos (LARGE_INTEGER *cic);
void inline ImprimirCiclos (const char *pc, LARGE_INTEGER *cic);
void inline ImprimirTiempo (const char *pc, LARGE_INTEGER *cic);
void inline RestarCiclos (LARGE_INTEGER *c_dif, LARGE_INTEGER *c_final, LARGE_INTEGER *c_inic);
