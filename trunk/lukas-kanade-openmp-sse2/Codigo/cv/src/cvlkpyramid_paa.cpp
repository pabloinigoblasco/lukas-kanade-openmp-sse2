/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                        Intel License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of Intel Corporation may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/
#include "_cv.h"
#include <float.h>
#include <stdio.h>
#include "contadorciclos.h"

//For testing purposses we define this here
#ifndef MINGW
//#define EN_ASM_1
//#define EN_ASM_1b
//#define EN_ASM_2
//#define EN_ASM_3_B
//#define EN_ASM_3
#define EN_ASM_4
#endif



static void
intersect_paa( CvPoint2D32f pt, CvSize win_size, CvSize imgSize,
			  CvPoint* min_pt, CvPoint* max_pt )
{
	CvPoint ipt;

	ipt.x = cvFloor( pt.x );
	ipt.y = cvFloor( pt.y );

	ipt.x -= win_size.width;
	ipt.y -= win_size.height;

	win_size.width = win_size.width * 2 + 1;
	win_size.height = win_size.height * 2 + 1;

	min_pt->x = MAX( 0, -ipt.x );
	min_pt->y = MAX( 0, -ipt.y );
	max_pt->x = MIN( win_size.width, imgSize.width - ipt.x );
	max_pt->y = MIN( win_size.height, imgSize.height - ipt.y );
}



static int 
icvMinimalPyramidSize_paa( CvSize imgSize )
{
	return cvAlign(imgSize.width,8) * imgSize.height / 3;
}






static void
icvInitPyramidalAlgorithm_paa( const CvMat* imgA, const CvMat* imgB,
							  CvMat* pyrA, CvMat* pyrB,
							  int level, CvTermCriteria * criteria,
							  int max_iters, int flags,
							  uchar *** imgI, uchar *** imgJ,
							  int **step, CvSize** size,
							  double **scale, uchar ** buffer )
{
	CV_FUNCNAME( "icvInitPyramidalAlgorithm_paa" );
	__BEGIN__;

	const int ALIGN = 8;
	int pyrBytes, bufferBytes = 0, elem_size;
	int level1 = level + 1;

	int i;
	CvSize imgSize, levelSize;

	*buffer = 0;
	*imgI = *imgJ = 0;
	*step = 0;
	*scale = 0;
	*size = 0;

	/* check input arguments */
	if( ((flags & CV_LKFLOW_PYR_A_READY) != 0 && !pyrA) ||
		((flags & CV_LKFLOW_PYR_B_READY) != 0 && !pyrB) )
		CV_ERROR( CV_StsNullPtr, "Some of the precomputed pyramids are missing" );

	if( level < 0 )
		CV_ERROR( CV_StsOutOfRange, "The number of pyramid layers is negative" );

	switch( criteria->type )
	{
	case CV_TERMCRIT_ITER:
		criteria->epsilon = 0.f;
		break;
	case CV_TERMCRIT_EPS:
		criteria->max_iter = max_iters;
		break;
	case CV_TERMCRIT_ITER | CV_TERMCRIT_EPS:
		break;
	default:
		assert( 0 );
		CV_ERROR( CV_StsBadArg, "Invalid termination criteria" );
	}

	/* compare squared values */
	criteria->epsilon *= criteria->epsilon;

	/* set pointers and step for every level */
	pyrBytes = 0;

	imgSize = cvGetSize(imgA);
	elem_size = CV_ELEM_SIZE(imgA->type);
	levelSize = imgSize;

	for( i = 1; i < level1; i++ )
	{
		levelSize.width = (levelSize.width + 1) >> 1;
		levelSize.height = (levelSize.height + 1) >> 1;

		int tstep = cvAlign(levelSize.width,ALIGN) * elem_size;
		pyrBytes += tstep * levelSize.height;
	}

	assert( pyrBytes <= imgSize.width * imgSize.height * elem_size * 4 / 3 );

	/* buffer_size = <size for patches> + <size for pyramids> */
	bufferBytes = (int)((level1 >= 0) * ((pyrA->data.ptr == 0) +
		(pyrB->data.ptr == 0)) * pyrBytes +
		(sizeof(imgI[0][0]) * 2 + sizeof(step[0][0]) +
		sizeof(size[0][0]) + sizeof(scale[0][0])) * level1);

	CV_CALL( *buffer = (uchar *)cvAlloc( bufferBytes ));

	*imgI = (uchar **) buffer[0];
	*imgJ = *imgI + level1;
	*step = (int *) (*imgJ + level1);
	*scale = (double *) (*step + level1);
	*size = (CvSize *)(*scale + level1);

	imgI[0][0] = imgA->data.ptr;
	imgJ[0][0] = imgB->data.ptr;
	step[0][0] = imgA->step;
	scale[0][0] = 1;
	size[0][0] = imgSize;

	if( level > 0 )
	{
		uchar *bufPtr = (uchar *) (*size + level1);
		uchar *ptrA = pyrA->data.ptr;
		uchar *ptrB = pyrB->data.ptr;

		if( !ptrA )
		{
			ptrA = bufPtr;
			bufPtr += pyrBytes;
		}

		if( !ptrB )
			ptrB = bufPtr;

		levelSize = imgSize;

		/* build pyramids for both frames */
		for( i = 1; i <= level; i++ )
		{
			int levelBytes;
			CvMat prev_level, next_level;

			levelSize.width = (levelSize.width + 1) >> 1;
			levelSize.height = (levelSize.height + 1) >> 1;

			size[0][i] = levelSize;
			step[0][i] = cvAlign( levelSize.width, ALIGN ) * elem_size;
			scale[0][i] = scale[0][i - 1] * 0.5;

			levelBytes = step[0][i] * levelSize.height;
			imgI[0][i] = (uchar *) ptrA;
			ptrA += levelBytes;

			if( !(flags & CV_LKFLOW_PYR_A_READY) )
			{
				prev_level = cvMat( size[0][i-1].height, size[0][i-1].width, CV_8UC1 );
				next_level = cvMat( size[0][i].height, size[0][i].width, CV_8UC1 );
				cvSetData( &prev_level, imgI[0][i-1], step[0][i-1] );
				cvSetData( &next_level, imgI[0][i], step[0][i] );
				cvPyrDown( &prev_level, &next_level );
			}

			imgJ[0][i] = (uchar *) ptrB;
			ptrB += levelBytes;

			if( !(flags & CV_LKFLOW_PYR_B_READY) )
			{
				prev_level = cvMat( size[0][i-1].height, size[0][i-1].width, CV_8UC1 );
				next_level = cvMat( size[0][i].height, size[0][i].width, CV_8UC1 );
				cvSetData( &prev_level, imgJ[0][i-1], step[0][i-1] );
				cvSetData( &next_level, imgJ[0][i], step[0][i] );
				cvPyrDown( &prev_level, &next_level );
			}
		}
	}

	__END__;
}





/* compute dI/dx and dI/dy */
/*PAA version*/
static void
icvCalcIxIy_32f_paa( const float* src, int src_step, float* dstX, float* dstY, int dst_step,
					CvSize src_size, const float* smooth_k, float* buffer0 )
{
	int src_width = src_size.width, dst_width = src_size.width-2;
	int x, height = src_size.height - 2;
	float* buffer1 = buffer0 + src_width;

	src_step /= sizeof(src[0]);
	dst_step /= sizeof(dstX[0]);

	for( ; height--; src += src_step, dstX += dst_step, dstY += dst_step )
	{
		const float* src2 = src + src_step;
		const float* src3 = src + src_step*2;

#ifndef EN_ASM_4
		for( x = 0; x < src_width; x++ )
		{
			float t0 = (src3[x] + src[x])*smooth_k[0] + src2[x]*smooth_k[1];
			float t1 = src3[x] - src[x];
			buffer0[x] = t0; 
			buffer1[x] = t1;
		}
#else
		__asm
		{
			mov edi, smooth_k
			movss xmm7, [edi] //smooth_k[0]
			shufps xmm7, xmm7, 0h
			movss xmm6, [edi+4] //smooth_k[1]
			shufps xmm6, xmm6, 0h
		}
		for( x = 0; x < src_width; x+=4 )
		{
			__asm
			{
			mov eax, x
			shl eax, 2 //Numero de bytes a desplazar

			mov esi, src3
			add esi, eax
			mov edi, src
			add edi, eax
			
			movups xmm1, [esi]
			movups xmm5, [edi]
			addps xmm1, xmm5 //xmm1 is t0 but needs more calculation

			movups xmm2, [esi]
			movups xmm5, [edi]
			subps xmm2, xmm5 //xmm2 is t1 and is ready to use

			
			mulps xmm1, xmm7 //(src3[x] + src[x])*smooth_k[0]
			
			mov edi, src2
			add edi, eax
			movups xmm4, [edi]

			mulps xmm4, xmm6

			addps xmm1, xmm4 //t0 fully worked
			
			mov esi, buffer0
			add esi, eax
			movups [esi], xmm1

			mov esi, buffer1
			add esi, eax
			movups [esi], xmm2
			}			
		}

		for(; x < src_width; x++ )
		{
			float t0 = (src3[x] + src[x])*smooth_k[0] + src2[x]*smooth_k[1];
			float t1 = src3[x] - src[x];
			buffer0[x] = t0; 
			buffer1[x] = t1;
		}
#endif


#ifndef EN_ASM_4
		for( x = 0; x < dst_width; x++ )
		{
			float t0 = buffer0[x+2] - buffer0[x];
			float t1 = (buffer1[x] + buffer1[x+2])*smooth_k[0] + buffer1[x+1]*smooth_k[1];
			dstX[x] = t0; dstY[x] = t1;
		}
#else
		for( x = 0; x < dst_width; x+=4 )
		{
			__asm
			{
			mov eax, x
			shl eax, 2 //Numero de bytes a desplazar

			mov esi, buffer0
			add esi, eax
			movups xmm1, [esi+8]
			movups xmm5, [esi]
			subps xmm1, xmm5 //t0

			mov esi, buffer1
			add esi, eax
			movups xmm2, [esi]
			movups xmm5, [esi+8]
			addps xmm2, xmm5 //buffer1[x] + buffer1[x+2]
			mulps xmm2, xmm7 //(buffer1[x] + buffer1[x+2])*smooth_k[0]

			movups xmm5, [esi+4]
			mulps xmm5,xmm6  //buffer1[x+1]*smooth_k[1]

			addps xmm2, xmm5 //t1

			mov esi, dstX
			add esi, eax
			movups [esi], xmm1

			mov esi, dstY
			add esi, eax
			movups [esi], xmm2
			}			
		}

		for(; x < dst_width; x++ )
		{
			float t0 = buffer0[x+2] - buffer0[x];
			float t1 = (buffer1[x] + buffer1[x+2])*smooth_k[0] + buffer1[x+1]*smooth_k[1];
			dstX[x] = t0; dstY[x] = t1;
		}
#endif
	}
}



/*PAA version*/
CV_IMPL void
cvCalcOpticalFlowPyrLK_paa( const void* arrA, const void* arrB,
						   void* pyrarrA, void* pyrarrB,
						   const CvPoint2D32f * featuresA,
						   CvPoint2D32f * featuresB,
						   int count, CvSize winSize, int level,
						   char *status, float *error,
						   CvTermCriteria criteria, int flags )
{
	///////////////////////////////////////////////////////////
	//Var decl and initialization
	///////////////////////////////////////////////////////////

/*	Cronometro c,c1,c2,c3,c4,c5,c6,c7;
	c.InhibeOutput=true;
	c1.InhibeOutput=true;
	c2.InhibeOutput=true;
	c3.InhibeOutput=true;
	c4.InhibeOutput=true;
	c5.InhibeOutput=true;
	c6.InhibeOutput=true;
	c7.InhibeOutput=true;*/


	//c.Start();
	uchar *pyrBuffer = 0;
	uchar *buffer = 0;
	float* _error = 0;
	char* _status = 0;

	void* ipp_optflow_state = 0;

	CV_FUNCNAME( "cvCalcOpticalFlowPyrLK_paa" );

	__BEGIN__;

	const int MAX_ITERS = 100;

	CvMat stubA, *imgA = (CvMat*)arrA;
	CvMat stubB, *imgB = (CvMat*)arrB;
	CvMat pstubA, *pyrA = (CvMat*)pyrarrA;
	CvMat pstubB, *pyrB = (CvMat*)pyrarrB;
	CvSize imgSize;
	static const float smoothKernel[] = { 0.09375, 0.3125, 0.09375 };  /* 3/32, 10/32, 3/32 */

	int bufferBytes = 0;
	uchar **imgI = 0;
	uchar **imgJ = 0;
	int *step = 0;
	double *scale = 0;
	CvSize* size = 0;

	int threadCount = cvGetNumThreads();
	float* _patchI[CV_MAX_THREADS];
	float* _patchJ[CV_MAX_THREADS];
	float* _Ix[CV_MAX_THREADS];
	float* _Iy[CV_MAX_THREADS];

	int i, l;

	CvSize patchSize = cvSize( winSize.width * 2 + 1, winSize.height * 2 + 1 );
	int patchLen = patchSize.width * patchSize.height;
	int srcPatchLen = (patchSize.width + 2)*(patchSize.height + 2);

	CV_CALL( imgA = cvGetMat( imgA, &stubA ));
	CV_CALL( imgB = cvGetMat( imgB, &stubB ));

	if( CV_MAT_TYPE( imgA->type ) != CV_8UC1 )
		CV_ERROR( CV_StsUnsupportedFormat, "Image type not supported" );

	if( !CV_ARE_TYPES_EQ( imgA, imgB ))
		CV_ERROR( CV_StsUnmatchedFormats, "images have different format" );

	if( !CV_ARE_SIZES_EQ( imgA, imgB ))
		CV_ERROR( CV_StsUnmatchedSizes, "Images have different size" );

	if( imgA->step != imgB->step )
		CV_ERROR( CV_StsUnmatchedSizes, "imgA and imgB must have equal steps" );

	imgSize = cvGetMatSize( imgA );

	if( pyrA )
	{
		CV_CALL( pyrA = cvGetMat( pyrA, &pstubA ));

		if( pyrA->step*pyrA->height < icvMinimalPyramidSize_paa( imgSize ) )
			CV_ERROR( CV_StsBadArg, "pyramid A has insufficient size" );
	}
	else
	{
		pyrA = &pstubA;
		pyrA->data.ptr = 0;
	}

	if( pyrB )
	{
		CV_CALL( pyrB = cvGetMat( pyrB, &pstubB ));

		if( pyrB->step*pyrB->height < icvMinimalPyramidSize_paa( imgSize ) )
			CV_ERROR( CV_StsBadArg, "pyramid B has insufficient size" );
	}
	else
	{
		pyrB = &pstubB;
		pyrB->data.ptr = 0;
	}

	//count -> Number of tracked points. Number of points of the images (pixel count??)
	if( count == 0 )
		EXIT;

	if( !featuresA || !featuresB )
		CV_ERROR( CV_StsNullPtr, "Some of arrays of point coordinates are missing" );

	if( count < 0 )
		CV_ERROR( CV_StsOutOfRange, "The number of tracked points is negative or zero" );

	if( winSize.width <= 1 || winSize.height <= 1 )
		CV_ERROR( CV_StsBadSize, "Invalid search window size" );

	for( i = 0; i < threadCount; i++ )
		_patchI[i] = _patchJ[i] = _Ix[i] = _Iy[i] = 0;

	CV_CALL( icvInitPyramidalAlgorithm_paa( imgA, imgB, pyrA, pyrB,
		level, &criteria, MAX_ITERS, flags,
		&imgI, &imgJ, &step, &size, &scale, &pyrBuffer ));

	if( !status )
		CV_CALL( status = _status = (char*)cvAlloc( count*sizeof(_status[0]) ));

	/* buffer_size = <size for patches> + <size for pyramids> */
	bufferBytes = (srcPatchLen + patchLen * 3) * sizeof( _patchI[0][0] ) * threadCount;
	CV_CALL( buffer = (uchar*)cvAlloc( bufferBytes ));

	
	//Some kind of buffer initiallization to give each thread a piece of these
	//Pyramid buffer??
	for( i = 0; i < threadCount; i++ )
	{
		_patchI[i] = i == 0 ? (float*)buffer : _Iy[i-1] + patchLen;
		_patchJ[i] = _patchI[i] + srcPatchLen;
		_Ix[i] = _patchJ[i] + patchLen;
		_Iy[i] = _Ix[i] + patchLen;
	}

	memset( status, 1, count );
	if( error )
		memset( error, 0, count*sizeof(error[0]) );

	if( !(flags & CV_LKFLOW_INITIAL_GUESSES) )
		memcpy( featuresB, featuresA, count*sizeof(featuresA[0]));

	//c.Stop();
	//c.PrintTime("tiempo hasta inicializacion");
	//c.Start();

	///////////////////////////////////////////////////////////
	//Image processing
	///////////////////////////////////////////////////////////

	/* do processing from top pyramid level (smallest image)
	to the bottom (original image) */
	for( l = level; l >= 0; l-- )
	{
		CvSize levelSize = size[l];
		int levelStep = step[l];


		/* find flow for each given point */
		//************** Point processing loop START **************
		for( i = 0; i < count; i++ ) //LN1
		{
			CvPoint2D32f v;
			CvPoint minI, maxI, minJ, maxJ;
			CvSize isz, jsz;
			int pt_status;
			CvPoint2D32f u;
			CvPoint prev_minJ = { -1, -1 }, prev_maxJ = { -1, -1 };
			double Gxx = 0, Gxy = 0, Gyy = 0, D = 0, minEig = 0;
			float prev_mx = 0, prev_my = 0;
			int j, x, y;
			//Looks like this gets his thread number and works over a piece of data
			//of the complete set
			int threadIdx = cvGetThreadNum();
			float* patchI = _patchI[threadIdx];
			float* patchJ = _patchJ[threadIdx];
			float* Ix = _Ix[threadIdx];
			float* Iy = _Iy[threadIdx];

		
			v.x = featuresB[i].x;
			v.y = featuresB[i].y;
			if( l < level )
			{ //If is not the last level (simple images)
				v.x += v.x;
				v.y += v.y;
			}
			else
			{ //If it is the last level (original image)
				v.x = (float)(v.x * scale[l]);
				v.y = (float)(v.y * scale[l]);
			}

			pt_status = status[i];
			if( !pt_status )
				continue;

			minI = maxI = minJ = maxJ = cvPoint( 0, 0 );

			u.x = (float) (featuresA[i].x * scale[l]);
			u.y = (float) (featuresA[i].y * scale[l]);

			intersect_paa( u, winSize, levelSize, &minI, &maxI );
			isz = jsz = cvSize(maxI.x - minI.x + 2, maxI.y - minI.y + 2);
			u.x += (minI.x - (patchSize.width - maxI.x + 1))*0.5f;
			u.y += (minI.y - (patchSize.height - maxI.y + 1))*0.5f;

			if( isz.width < 3 || isz.height < 3 ||
				icvGetRectSubPix_8u32f_C1R( imgI[l], levelStep, levelSize,
				patchI, isz.width*sizeof(patchI[0]), isz, u ) < 0 )
			{
				/* point is outside the image. take the next */
				status[i] = 0;
				continue;
			}
			
			//Calcula el gradiente de intensidad en x e y
			//c4.Start();
			icvCalcIxIy_32f_paa( patchI, isz.width*sizeof(patchI[0]), Ix, Iy,
				(isz.width-2)*sizeof(patchI[0]), isz, smoothKernel, patchJ );
			//c4.Stop();

			for( j = 0; j < criteria.max_iter; j++ ) //LN2
			{
				//NOTE: Variables bx and by have been renamed to _bx and _by respectively to avoid
				//confusion in the inline assembly with the register bx (16bit part of ebx) that
				//was causing a lot of headaches
				double _bx = 0, _by = 0;
				float mx, my;
				CvPoint2D32f _v;
				//c7.Start();
				intersect_paa( v, winSize, levelSize, &minJ, &maxJ );

				minJ.x = MAX( minJ.x, minI.x );
				minJ.y = MAX( minJ.y, minI.y );

				maxJ.x = MIN( maxJ.x, maxI.x );
				maxJ.y = MIN( maxJ.y, maxI.y );

				jsz = cvSize(maxJ.x - minJ.x, maxJ.y - minJ.y);

				_v.x = v.x + (minJ.x - (patchSize.width - maxJ.x + 1))*0.5f;
				_v.y = v.y + (minJ.y - (patchSize.height - maxJ.y + 1))*0.5f;
				//c7.Stop();
				if( jsz.width < 1 || jsz.height < 1 ||
					icvGetRectSubPix_8u32f_C1R( imgJ[l], levelStep, levelSize, patchJ,
					jsz.width*sizeof(patchJ[0]), jsz, _v ) < 0 )
				{
					/* point is outside image. take the next */
					pt_status = 0;
					break;
				}

				if( maxJ.x == prev_maxJ.x && maxJ.y == prev_maxJ.y &&
					minJ.x == prev_minJ.x && minJ.y == prev_minJ.y )
				{
					//c1.Start();
					for( y = 0; y < jsz.height; y++ )//LN3
					{
						//Why it uses const???
						float* pi = patchI +
							(y + minJ.y - minI.y + 1)*isz.width + minJ.x - minI.x + 1;
						float* pj = patchJ + y*jsz.width;
						float* ix = Ix +
							(y + minJ.y - minI.y)*(isz.width-2) + minJ.x - minI.x;
						float* iy = Iy + (ix - Ix);
#ifndef EN_ASM_1
						
						for( x = 0; x < jsz.width; x++ )//LN4
						{
							double t = pi[x] - pj[x];
							_bx += t * ix[x];
							_by += t * iy[x];
						}
						
#else
					
#ifdef EN_ASM_1b
  double* tmpBx[] = {0,0};
  double* tmpBy[] = {0,0};
  //you don't need to preserve the EAX, EBX, ECX, EDX, ESI, or EDI registers
__asm{

	mov esi, pi
	mov edi, pj
	mov ebx, ix
	mov ecx, iy
	//We need to get the last position that we can process without getting 
	//out of the data
	mov eax, jsz.width
	shr eax, 3 //Divide _by 8 to get the number of iterations to optimize
	shl eax, 5 //Multiply by 32 to get the number of bytes to process
	mov edx, pi
	//Add to pi and we have the limit address to process to
	add edx, eax //Limit for pi, it will be used as limit condition for all

ln4_block1_opt:
	cmp esi, edx //if esi<edx
	jge ln4_block1_norm_entry //go to normal iterations

	//t * ix[x];
	movups xmm4, [ebx] //ix[x+3], ix[x+2], ix[x+1], ix[x+0]
	//conversion to double
	movaps xmm5, xmm4 //make a copy
	//We make each xmm to be two array items
	shufps xmm4, xmm4, 04h//0000 0100
	shufps xmm5, xmm5, 0eh//0000 1110
	cvtps2pd xmm4, xmm4 //xmm5 is ix[x+1],ix[x+0] to doubles
	cvtps2pd xmm5, xmm5 //xmm6 is ix[x+3],ix[x+2] to doubles

	//t calculation
	movups xmm1, [esi]
	movups xmm0, [edi]
	subps xmm1, xmm0
    //t1,t2,t3,t4 in xmm1
	
	//Convert first 4 floats t to doubles into xmm1 and xmm2
	movaps xmm2, xmm1
	shufps xmm1, xmm1, 04h
	cvtps2pd xmm1, xmm1 //t[x+1], t[0]
	
	mulpd xmm4, xmm1 //two first doubles t1*ix1, t0*ix0

	shufps xmm2, xmm2, 0eh
	cvtps2pd xmm2, xmm2 //t[x+3], t[x+2]

	//t * iy[x]
	movups xmm6, [ecx]
	movaps xmm7, xmm6
	//We make each xmm to be two array items
	shufps xmm6, xmm6, 04h//0000 0100
	shufps xmm7, xmm7, 0eh//0000 1110

	mulpd xmm5, xmm2 //two next doubles t3*ix3, t2*ix2
	addpd xmm4, [tmpBx] //acumulator
	movupd [tmpBx], xmm4 //xmm4 free

	cvtps2pd xmm6, xmm6 //xmm6 is ix[x+1],ix[x+0] to doubles
	cvtps2pd xmm7, xmm7 //xmm7 is ix[x+3],ix[x+2] to doubles

	//t * ix[x+4];
	movups xmm0, [ebx+16] //ix[x+3], ix[x+2], ix[x+1], ix[x+0]
	//conversion to double
	movaps xmm3, xmm0 //make a copy

	mulpd xmm6, xmm1 //two next doubles t1*iy1, t0*iy0
	addpd xmm5, [tmpBx] //bx3+bx1, bx2+bx0
	movupd [tmpBx], xmm5 //xmm5, xmm4 and xmm1 free

	//We make each xmm to be two array items
	shufps xmm0, xmm0, 04h//0000 0100
	shufps xmm3, xmm3, 0eh//0000 1110
	cvtps2pd xmm0, xmm0 //xmm0 is ix[x+1],ix[x+0] to doubles
	cvtps2pd xmm3, xmm3 //xmm3 is ix[x+3],ix[x+2] to doubles

	mulpd xmm7, xmm2 //t3*iy3, t2*iy2
	addpd xmm6, [tmpBy]
	movupd [tmpBy], xmm6 //xmm1, xmm2, xmm4, xmm5, xmm6 free

	//t calculation
	movups xmm1, [esi+16]
	movups xmm2, [edi+16]
	subps xmm1, xmm2
    //t5,t6,t7,t8 in xmm1

	//Convert first 4 floats t to doubles into xmm1 and xmm2
	movaps xmm2, xmm1
	shufps xmm1, xmm1, 04h
	cvtps2pd xmm1, xmm1 //t[x+1], t[0]

	mulpd xmm0, xmm1
	addpd xmm7, [tmpBy]
	movupd [tmpBy], xmm7 //xmm4, xmm5, xmm6, xmm7 free

	shufps xmm2, xmm2, 0eh
	cvtps2pd xmm2, xmm2 //t[x+3], t[x+2]

	//t * iy[x+4]
	movups xmm4, [ecx+16]
	movaps xmm5, xmm4

	mulpd xmm3, xmm2
	addpd xmm0, [tmpBx]
	movupd [tmpBx], xmm0 // xmm0, xmm6, xmm7 free

	//We make each xmm to be two array items
	shufps xmm4, xmm4, 04h//0000 0100
	shufps xmm5, xmm5, 0eh//0000 1110
	cvtps2pd xmm4, xmm4 //xmm4 is ix[x+1],ix[x+0] to doubles
	cvtps2pd xmm5, xmm5 //xmm5 is ix[x+3],ix[x+2] to doubles
	
	mulpd xmm4, xmm1
	addpd xmm3, [tmpBy]
	movupd [tmpBy], xmm3 // xmm0, xmm6, xmm7 free

	add esi, 16
	add edi, 16
	add ebx, 16
	add ecx, 16

	mulpd xmm5, xmm2
	addpd xmm4, [tmpBy]
	movupd [tmpBy], xmm4

	////add them
	//addpd xmm3, xmm4
	//movapd xmm4, xmm3 //copy 
	//shufpd xmm4, xmm3, 1h//0000 0001 reverse
	//addpd xmm3, xmm4 //xmm3 is now to doubles with the same value
	////We add the variable
	//addsd xmm3, _by
	////store it!
	//movsd _by, xmm3

	add esi, 16
	add edi, 16
	add ebx, 16
	add ecx, 16

	addpd xmm5, [tmpBy]
	movupd [tmpBy], xmm5

	jmp ln4_block1_opt

ln4_block1_norm_entry:
	mov eax, jsz.width
	shl eax, 4 //Multiply by 4 to get the total number of bytes to process
	mov edx, pi
	add edx, eax //Limit for pi

	movupd xmm1, [tmpBx]
	movapd xmm2, xmm1
	shufpd xmm1, xmm2, 1h//0000 0001 reverse
	addpd xmm1, xmm2
	movsd _bx, xmm1

	movupd xmm1, [tmpBx]
	movupd xmm3, [tmpBy]
	movapd xmm2, xmm1
	movapd xmm4, xmm3
	shufpd xmm1, xmm2, 1h//0000 0001 reverse
	shufpd xmm3, xmm4, 1h
	addpd xmm1, xmm2
	addpd xmm3, xmm4
	movsd _bx, xmm1
	movsd _by, xmm3

ln4_block1_norm:
	cmp esi, edx
	jge ln4_block1_fin //If ecx >= edx go to normal iterations

	//double t = pi[x] - pj[x];
	mov esi, pi
	add esi, ecx
	movss xmm1, [esi]
	cvtss2sd xmm1, xmm1
	mov esi, pj
	add esi, ecx
	movss xmm2, [esi]
	cvtss2sd xmm2, xmm2
	subsd xmm1, xmm2
	movsd xmm0, xmm1

	//_bx += (double) (t * ix[x]);
	mov esi, ix
	add esi, ecx
	movss xmm1, [esi]
	cvtss2sd xmm1, xmm1
	mulsd xmm1, xmm0
	addsd xmm1, _bx
	movsd _bx, xmm1

	//_by += (double) (t * iy[x]);
	mov esi, iy
	add esi, ecx
	movss xmm1, [esi]
	cvtss2sd xmm1, xmm1
	mulsd xmm1, xmm0
	addsd xmm1, _by
	movsd _by, xmm1

	add esi, 4
	add edi, 4
	jmp ln4_block1_norm

ln4_block1_fin:
						}
#else
						__asm{

							mov eax, jsz.width
								shr eax, 2 //Divide _by 4 to get the number of iterations to optimize
								shl eax, 4 //Multiply by 16 to get the number of bytes to process
								//4 floats per iteration (16 bytes per iteration)
								mov edx, jsz.width
								shl edx, 2 //Total number of bytes to process
								mov ecx, 0h//counter. Bytes processed
						
ln4_lesmult_opt:
							cmp ecx, eax
								jge ln4_lesmult_norm //If ecx >= eax go to normal iterations

								//t calculation
								mov esi, pi
								add esi, ecx
								movups xmm1, [esi]
							mov esi, pj
								add esi, ecx
								movups xmm2, [esi]
							subps xmm1, xmm2
								movaps xmm2, xmm1
								shufps xmm1, xmm1, 04h
								shufps xmm2, xmm2, 0eh
								cvtps2pd xmm1, xmm1 //t[x+1], t[0]
								cvtps2pd xmm2, xmm2 //t[x+3], t[x+2]

								//_bx += t * ix[x];
								mov esi, ix
								add esi, ecx
								movups xmm3, [esi] //ix[x+3], ix[x+2], ix[x+1], ix[x+0]
							//conversion to double
							movaps xmm4, xmm3 //make a copy
								//We make each xmm to be two array items
								shufps xmm3, xmm3, 04h//0000 0100
								shufps xmm4, xmm4, 0eh//0000 1110
								cvtps2pd xmm3, xmm3 //xmm3 is ix[x+1],ix[x+0] to doubles
								cvtps2pd xmm4, xmm4 //xmm4 is ix[x+3],ix[x+2] to doubles
								//make the mults with t => xmm1*xmm3 and xmm2*xmm4
								mulpd xmm3, xmm1
								mulpd xmm4, xmm2
								//add them
								addpd xmm3, xmm4
								movapd xmm4, xmm3 //copy 
								shufpd xmm4, xmm3, 1h//0000 0001 reverse
								addpd xmm3, xmm4 //xmm3 is now to doubles with the same value
								//We add the variable
								addsd xmm3, _bx
								//store it!
								movsd _bx, xmm3

								//_by += t * iy[x];
								mov esi, iy
								add esi, ecx
								movups xmm3, [esi] //ix[x+3], ix[x+2], ix[x+1], ix[x+0]
							//conversion to double
							movaps xmm4, xmm3 //make a copy
								//We make each xmm to be two array items
								shufps xmm3, xmm3, 04h//0000 0100
								shufps xmm4, xmm4, 0eh//0000 1110
								cvtps2pd xmm3, xmm3 //xmm3 is ix[x+1],ix[x+0] to doubles
								cvtps2pd xmm4, xmm4 //xmm4 is ix[x+3],ix[x+2] to doubles
								//make the mults with t => xmm1*xmm3 and xmm2*xmm4
								mulpd xmm3, xmm1
								mulpd xmm4, xmm2
								//add them
								addpd xmm3, xmm4
								movapd xmm4, xmm3 //copy 
								shufpd xmm4, xmm3, 1h//0000 0001 reverse
								addpd xmm3, xmm4 //xmm3 is now to doubles with the same value
								//We add the variable
								addsd xmm3, _by
								//store it!
								movsd _by, xmm3

								add ecx, 16
								jmp ln4_lesmult_opt

ln4_lesmult_norm:
							cmp ecx, edx
								jge ln4_lesmult_fin //If ecx >= edx go to normal iterations

								//double t = pi[x] - pj[x];
								mov esi, pi
								add esi, ecx
								movss xmm1, [esi]
							cvtss2sd xmm1, xmm1
								mov esi, pj
								add esi, ecx
								movss xmm2, [esi]
							cvtss2sd xmm2, xmm2
								subsd xmm1, xmm2
								movsd xmm0, xmm1

								//_bx += (double) (t * ix[x]);
								mov esi, ix
								add esi, ecx
								movss xmm1, [esi]
							cvtss2sd xmm1, xmm1
								mulsd xmm1, xmm0
								addsd xmm1, _bx
								movsd _bx, xmm1

								//_by += (double) (t * iy[x]);
								mov esi, iy
								add esi, ecx
								movss xmm1, [esi]
							cvtss2sd xmm1, xmm1
								mulsd xmm1, xmm0
								addsd xmm1, _by
								movsd _by, xmm1

								add ecx, 4
								jmp ln4_lesmult_norm

ln4_lesmult_fin:
						}
#endif
#endif
					}
					//c1.Stop();
				}
				else
				{
					//c6.Start();
					Gxx = Gyy = Gxy = 0;
					for( y = 0; y < jsz.height; y++ )//LN3
					{
						
						const float* pi = patchI +
							(y + minJ.y - minI.y + 1)*isz.width + minJ.x - minI.x + 1;
						const float* pj = patchJ + y*jsz.width;
						const float* ix = Ix +
							(y + minJ.y - minI.y)*(isz.width-2) + minJ.x - minI.x;
						const float* iy = Iy + (ix - Ix);

#ifndef EN_ASM_2
						//c2.Start();
						for( x = 0; x < jsz.width; x++ )//LN4
						{
							double t = pi[x] - pj[x];
							_bx += (double) (t * ix[x]);
							_by += (double) (t * iy[x]);
							Gxx += ix[x] * ix[x];
							Gxy += ix[x] * iy[x];
							Gyy += iy[x] * iy[x];
						}
						//c2.Stop();
#else
						__asm{

							mov eax, jsz.width
								shr eax, 2 //Divide _by 4 to get the number of iterations to optimize
								shl eax, 4 //Multiply by 16 to get the number of bytes to process
								//4 floats per iteration (16 bytes per iteration)
								mov edx, jsz.width
								shl edx, 2 //Total number of bytes to process
								mov ecx, 0h//counter. Bytes processed

ln4_block2_opt:
							cmp ecx, eax
								jge ln4_block2_norm //If ecx >= eax go to normal iterations

								//t calculation
								mov esi, pi
								add esi, ecx
								movups xmm1, [esi]
							mov esi, pj
								add esi, ecx
								movups xmm2, [esi]
							subps xmm1, xmm2
								movaps xmm2, xmm1
								shufps xmm1, xmm1, 04h
								shufps xmm2, xmm2, 0eh
								cvtps2pd xmm1, xmm1 //t[x+1], t[0]
								cvtps2pd xmm2, xmm2 //t[x+3], t[x+2]

								//_bx += t * ix[x];
								mov esi, ix
								add esi, ecx
								movups xmm3, [esi] //ix[x+3], ix[x+2], ix[x+1], ix[x+0]
							//conversion to double
							movaps xmm4, xmm3 //make a copy
								//We make each xmm to be two array items
								shufps xmm3, xmm3, 04h//0000 0100
								shufps xmm4, xmm4, 0eh//0000 1110
								cvtps2pd xmm3, xmm3 //xmm3 is ix[x+1],ix[x+0] to doubles
								cvtps2pd xmm4, xmm4 //xmm4 is ix[x+3],ix[x+2] to doubles
								//make the mults with t => xmm1*xmm3 and xmm2*xmm4
								mulpd xmm3, xmm1
								mulpd xmm4, xmm2
								//add them
								addpd xmm3, xmm4
								movapd xmm4, xmm3 //copy 
								shufpd xmm4, xmm3, 1h//0000 0001 reverse
								addpd xmm3, xmm4 //xmm3 is now to doubles with the same value
								//We add the variable
								addsd xmm3, _bx
								//store it!
								movsd _bx, xmm3

								//_by += t * iy[x];
								mov esi, iy
								add esi, ecx
								movups xmm3, [esi] //ix[x+3], ix[x+2], ix[x+1], ix[x+0]
							//conversion to double
							movaps xmm4, xmm3 //make a copy
								//We make each xmm to be two array items
								shufps xmm3, xmm3, 04h//0000 0100
								shufps xmm4, xmm4, 0eh//0000 1110
								cvtps2pd xmm3, xmm3 //xmm3 is ix[x+1],ix[x+0] to doubles
								cvtps2pd xmm4, xmm4 //xmm4 is ix[x+3],ix[x+2] to doubles
								//make the mults with t => xmm1*xmm3 and xmm2*xmm4
								mulpd xmm3, xmm1
								mulpd xmm4, xmm2
								//add them
								addpd xmm3, xmm4
								movapd xmm4, xmm3 //copy 
								shufpd xmm4, xmm3, 1h//0000 0001 reverse
								addpd xmm3, xmm4 //xmm3 is now to doubles with the same value
								//We add the variable
								addsd xmm3, _by
								//store it!
								movsd _by, xmm3

								//Gxx += ix[x] * ix[x];
								mov esi, ix
								add esi, ecx
								movups xmm3, [esi] //ix[x+3], ix[x+2], ix[x+1], ix[x+0]
							//conversion to double
							movaps xmm4, xmm3 //make a copy
								//We make each xmm to be two array items
								shufps xmm3, xmm3, 04h//0000 0100
								shufps xmm4, xmm4, 0eh//0000 1110
								cvtps2pd xmm3, xmm3 //xmm3 is ix[x+1],ix[x+0] to doubles
								cvtps2pd xmm4, xmm4 //xmm4 is ix[x+3],ix[x+2] to doubles
								//make the mults with t => xmm1*xmm3 and xmm2*xmm4
								mulpd xmm3, xmm3
								mulpd xmm4, xmm4
								//add them
								addpd xmm3, xmm4
								movapd xmm4, xmm3 //copy 
								shufpd xmm4, xmm3, 1h//0000 0001 reverse
								addpd xmm3, xmm4 //xmm3 is now to doubles with the same value
								//We add the variable
								addsd xmm3, Gxx
								movsd Gxx, xmm3

								//Gxy += ix[x] * iy[x];
								mov esi, ix
								add esi, ecx
								movups xmm3, [esi] //ix[x+3], ix[x+2], ix[x+1], ix[x+0]
							//conversion to double
							movaps xmm4, xmm3 //make a copy
								//We make each xmm to be two array items
								shufps xmm3, xmm3, 04h//0000 0100
								shufps xmm4, xmm4, 0eh//0000 1110
								cvtps2pd xmm3, xmm3 //xmm3 is ix[x+1],ix[x+0] to doubles
								cvtps2pd xmm4, xmm4 //xmm4 is ix[x+3],ix[x+2] to doubles
								mov esi, iy
								add esi, ecx
								movups xmm5, [esi] //ix[x+3], ix[x+2], ix[x+1], ix[x+0]
							//conversion to double
							movaps xmm6, xmm5 //make a copy
								//We make each xmm to be two array items
								shufps xmm5, xmm5, 04h//0000 0100
								shufps xmm6, xmm6, 0eh//0000 1110
								cvtps2pd xmm5, xmm5 //xmm3 is ix[x+1],ix[x+0] to doubles
								cvtps2pd xmm6, xmm6 //xmm4 is ix[x+3],ix[x+2] to doubles
								//make the mults with t => xmm1*xmm3 and xmm2*xmm4
								mulpd xmm3, xmm5
								mulpd xmm4, xmm6
								//add them
								addpd xmm3, xmm4
								movapd xmm4, xmm3 //copy 
								shufpd xmm4, xmm3, 1h//0000 0001 reverse
								addpd xmm3, xmm4 //xmm3 is now to doubles with the same value
								//We add the variable
								addsd xmm3, Gxy
								movsd Gxy, xmm3

								//Gyy += iy[x] * iy[x];
								mov esi, iy
								add esi, ecx
								movups xmm3, [esi] //ix[x+3], ix[x+2], ix[x+1], ix[x+0]
							//conversion to double
							movaps xmm4, xmm3 //make a copy
								//We make each xmm to be two array items
								shufps xmm3, xmm3, 04h//0000 0100
								shufps xmm4, xmm4, 0eh//0000 1110
								cvtps2pd xmm3, xmm3 //xmm3 is ix[x+1],ix[x+0] to doubles
								cvtps2pd xmm4, xmm4 //xmm4 is ix[x+3],ix[x+2] to doubles
								//make the mults with t => xmm1*xmm3 and xmm2*xmm4
								mulpd xmm3, xmm3
								mulpd xmm4, xmm4
								//add them
								addpd xmm3, xmm4
								movapd xmm4, xmm3 //copy 
								shufpd xmm4, xmm3, 1h//0000 0001 reverse
								addpd xmm3, xmm4 //xmm3 is now to doubles with the same value
								//We add the variable
								addsd xmm3, Gyy
								movsd Gyy, xmm3

								add ecx, 16
								jmp ln4_block2_opt
ln4_block2_norm:
							cmp ecx, edx
								jge ln4_block2_fin //If ecx >= ebx go to normal iterations

								//double t = pi[x] - pj[x];
								mov esi, pi
								add esi, ecx
								movss xmm1, [esi]
							cvtss2sd xmm1, xmm1
								mov esi, pj
								add esi, ecx
								movss xmm2, [esi]
							cvtss2sd xmm2, xmm2
								subsd xmm1, xmm2
								movsd xmm0, xmm1

								//_bx += (double) (t * ix[x]);
								mov esi, ix
								add esi, ecx
								movss xmm1, [esi]
							cvtss2sd xmm1, xmm1
								mulsd xmm1, xmm0
								addsd xmm1, _bx
								movsd _bx, xmm1

								//_by += (double) (t * iy[x]);
								mov esi, iy
								add esi, ecx
								movss xmm1, [esi]
							cvtss2sd xmm1, xmm1
								mulsd xmm1, xmm0
								addsd xmm1, _by
								movsd _by, xmm1

								//Gxx += ix[x] * ix[x];
								mov esi, ix
								add esi, ecx
								movss xmm1, [esi]
							cvtss2sd xmm1, xmm1
								mulsd xmm1, xmm1
								addsd xmm1, Gxx
								movsd Gxx, xmm1

								//Gxy += ix[x] * iy[x];
								mov esi, ix
								add esi, ecx
								movss xmm1, [esi]
							cvtss2sd xmm1, xmm1
								mov esi, iy
								add esi, ecx
								movss xmm2, [esi]
							cvtss2sd xmm2, xmm2
								mulsd xmm1, xmm2
								addsd xmm1, Gxy
								movsd Gxy, xmm1

								//Gyy += iy[x] * iy[x];
								mov esi, iy
								add esi, ecx
								movss xmm1, [esi]
							cvtss2sd xmm1, xmm1
								mulsd xmm1, xmm1
								addsd xmm1, Gyy
								movsd Gyy, xmm1

								add ecx, 4
								jmp ln4_block2_norm
ln4_block2_fin:
						}
#endif
					}

					D = Gxx * Gyy - Gxy * Gxy;
					if( D < DBL_EPSILON )
					{
						pt_status = 0;
						break;
					}

					// Adi Shavit - 2008.05
					if( flags & CV_LKFLOW_GET_MIN_EIGENVALS )
						minEig = (Gyy + Gxx - sqrt((Gxx-Gyy)*(Gxx-Gyy) + 4.*Gxy*Gxy))/(2*jsz.height*jsz.width);

					D = 1. / D;

					prev_minJ = minJ;
					prev_maxJ = maxJ;
					//c6.Stop();
				}

				//c5.Start();

				mx = (float) ((Gyy * _bx - Gxy * _by) * D);
				my = (float) ((Gxx * _by - Gxy * _bx) * D);

				v.x += mx;
				v.y += my;

				if( mx * mx + my * my < criteria.epsilon )
					break;

				if( j > 0 && fabs(mx + prev_mx) < 0.01 && fabs(my + prev_my) < 0.01 )
				{
					v.x -= mx*0.5f;
					v.y -= my*0.5f;
					break;
				}
				prev_mx = mx;
				prev_my = my;
				//c5.Stop();
			}
			
			featuresB[i] = v;
			status[i] = (char)pt_status;
			//c3.Start();
			if( l == 0 && error && pt_status ) {

				/* calc error */
				float err = 0;
				if( flags & CV_LKFLOW_GET_MIN_EIGENVALS ) {
					err = minEig;
				} else {

					for( y = 0; y < jsz.height; y++ ) {//LN3 

						const float * pi = patchI + (y + minJ.y - minI.y + 1)*isz.width + minJ.x - minI.x + 1;
						const float * pj = patchJ + y*jsz.width;
#ifndef EN_ASM_3
						
						for( x = 0; x < jsz.width; x++ ) {//LN4
							double t = pi[x] - pj[x];
							err += t * t;
						}
#elif defined(EN_ASM_3_B)

		int size=jsz.width;
		float* pjp=(float*)pj;
		float* pip=(float*)pi;
		float* endpi=(float*)pi+((size/8)*8);
		float* realendpi=(float*)pi+size;
		float zeros[]={0,0,0,0};
		float values[4];
								
							__asm
							{
								

									mov eax, [pip]
									mov ebx, [pjp]
									mov ecx, [endpi]

									movups xmm7,[zeros]
_loop:
								    cmp eax,ecx
									jge _end
									
									movups xmm1, [eax]
									movups xmm2, [ebx]
									movups xmm3, [eax+16]
									movups xmm4, [ebx+16]
									subps xmm1, xmm2
									subps xmm3, xmm4
									mulps xmm1, xmm1 
									mulps xmm3, xmm3
									addps xmm7, xmm1
									add eax,32
									addps xmm3, xmm1
									add ebx,32
									jmp _loop
_end:

									movups [values],xmm7								
									mov ecx,[realendpi]

_loop2:								cmp eax,ecx
									jge _end2
									movss xmm1,[eax]
									movss xmm2,[ebx]
									subss xmm1,xmm2
									movss xmm2,err
									addss xmm1,xmm2
									movss [err],xmm1

									add eax,4
									add ebx,4
									jmp _loop2
_end2:
									


								}
	err+=values[0]+values[1]+values[2]+values[3];
						
#else
						__asm{

							mov eax, jsz.width
								shr eax, 2 //Divide _by 4 to get the number of iterations to optimize
								shl eax, 4 //Multiply by 16 to get the number of bytes to process
								//4 floats per iteration (16 bytes per iteration)
								mov edx, jsz.width
								shl edx, 2 //Total number of bytes to process
								mov ecx, 0h//counter. Bytes processed

ln4_block3_opt:
							cmp ecx, eax
								jge ln4_block3_norm //If ecx >= eax go to normal iterations

								//t calculation
								mov esi, pi
								add esi, ecx
								movups xmm1, [esi]
							mov esi, pj
								add esi, ecx
								movups xmm2, [esi]
							subps xmm1, xmm2
								movaps xmm2, xmm1
								shufps xmm1, xmm1, 04h
								shufps xmm2, xmm2, 0eh
								cvtps2pd xmm1, xmm1 //t[x+1], t[0]
								cvtps2pd xmm2, xmm2 //t[x+3], t[x+2]

								//err += t * t;
								mulpd xmm1, xmm1 //t[x+1]*t[x+1], t[x]*t[x]
								mulpd xmm2, xmm2
								//add them
								addpd xmm1, xmm2
								movapd xmm2, xmm1 //copy 
								shufpd xmm2, xmm1, 1h//0000 0001 reverse
								addpd xmm1, xmm2 //xmm1 is now to doubles with the same value
								//We add the variable
								addsd xmm1, err
								movsd err, xmm1

								add ecx, 16
								jmp ln4_block3_opt

ln4_block3_norm:
							cmp ecx, edx
								jge ln4_block3_fin

								mov esi, pi    //load pi array address
								add esi, ecx   //add counter to address
								movss xmm1, [esi] //load 1 floats from pi
							mov esi, pj
								add esi, ecx
								movss xmm2, [esi] //Load 1 floats from pj

							subss xmm1, xmm2 //1 subs
								cvtss2sd xmm1, xmm1
								mulsd xmm1, xmm1 //(pi[x]-pj[x])^2

								addsd xmm1, err
								movsd err, xmm1

								add ecx, 4 //Add the number of processed bytes
								jmp ln4_block3_norm
								//Fin de bucle
ln4_block3_fin:
						}
#endif
					}
					err = sqrt(err);
				}
				//c3.Stop();
				error[i] = (float)err;
			}
		}
		//************** Point processing loop End **************

	} // end of pyramid levels loop (l)
	//c.Stop();
	/*c.PrintTime("tiempo de proceso:");
	c1.PrintTime("Bloque 1");
	c2.PrintTime("Bloque 2");
	c3.PrintTime("Bloque 3");
	c4.PrintTime("Bloque 4");
	c5.PrintTime("Bloque 5");
	c6.PrintTime("Bloque 6");
	c7.PrintTime("Bloque 7");*/

	__END__;

	if( ipp_optflow_state )
		icvOpticalFlowPyrLKFree_8u_C1R_p( ipp_optflow_state );

	cvFree( &pyrBuffer );
	cvFree( &buffer );
	cvFree( &_error );
	cvFree( &_status );
}



