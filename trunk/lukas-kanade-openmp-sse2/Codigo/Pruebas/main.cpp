#include <stdio.h>
#include <stdlib.h>

int main(int argc, char * argcv[])
{
	float t[] = {0.1,0.2,0.3,0.4};
	double t2[] = {0.0,0.0,0.0,0.0};

	__asm {
		lea esi, t
		movups xmm0, [esi]
		movaps xmm1, xmm0
		movaps xmm2, xmm0
		shufps xmm1, xmm1, 04h//0000 0100 Coge arr[0] y arr [1]
		shufps xmm2, xmm2, 0eh//0000 1110 Coge arr[2] y arr[3]
		cvtps2pd xmm1, xmm1
		cvtps2pd xmm2, xmm2
		lea esi, t2
		movupd [esi], xmm1
		movupd [esi + 16], xmm2
	}

	printf("%d, %d, %d, %d\n", t2[0], t2[1], t2[2], t2[3]);
	printf("INTRO para salir");
	getchar();
}