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

//#define EN_ASM_8
//#define EN_ASM_7
#define EN_ASM_6

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

	//TODO: Optimize this?
    for( ; height--; src += src_step, dstX += dst_step, dstY += dst_step )
    {
        const float* src2 = src + src_step;
        const float* src3 = src + src_step*2;

        for( x = 0; x < src_width; x++ )
        {
            float t0 = (src3[x] + src[x])*smooth_k[0] + src2[x]*smooth_k[1];
            float t1 = src3[x] - src[x];
            buffer0[x] = t0; buffer1[x] = t1;
        }

        for( x = 0; x < dst_width; x++ )
        {
            float t0 = buffer0[x+2] - buffer0[x];
            float t1 = (buffer1[x] + buffer1[x+2])*smooth_k[0] + buffer1[x+1]*smooth_k[1];
            dstX[x] = t0; dstY[x] = t1;
        }
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

	///////////////////////////////////////////////////////////
	//Image processing
	///////////////////////////////////////////////////////////

    /* do processing from top pyramid level (smallest image)
       to the bottom (original image) */
    for( l = level; l >= 0; l-- )
    {
        CvSize levelSize = size[l];
        int levelStep = step[l];

        
#ifdef _OPENMP
        #pragma omp parallel for num_threads(threadCount) schedule(dynamic) 
#endif // _OPENMP
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
            __declspec(align(16)) float* patchI = _patchI[threadIdx];
            __declspec(align(16)) float* patchJ = _patchJ[threadIdx];
            __declspec(align(16)) float* Ix = _Ix[threadIdx];
            __declspec(align(16)) float* Iy = _Iy[threadIdx];

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
            icvCalcIxIy_32f_paa( patchI, isz.width*sizeof(patchI[0]), Ix, Iy,
                (isz.width-2)*sizeof(patchI[0]), isz, smoothKernel, patchJ );

            for( j = 0; j < criteria.max_iter; j++ ) //LN2
            {
                double bx = 0, by = 0;
                float mx, my;
                CvPoint2D32f _v;

                intersect_paa( v, winSize, levelSize, &minJ, &maxJ );

                minJ.x = MAX( minJ.x, minI.x );
                minJ.y = MAX( minJ.y, minI.y );

                maxJ.x = MIN( maxJ.x, maxI.x );
                maxJ.y = MIN( maxJ.y, maxI.y );

                jsz = cvSize(maxJ.x - minJ.x, maxJ.y - minJ.y);

                _v.x = v.x + (minJ.x - (patchSize.width - maxJ.x + 1))*0.5f;
                _v.y = v.y + (minJ.y - (patchSize.height - maxJ.y + 1))*0.5f;

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
                    for( y = 0; y < jsz.height; y++ )//LN3
                    {
						//Why it uses const???
                        __declspec(align(16)) float* pi = patchI +
                            (y + minJ.y - minI.y + 1)*isz.width + minJ.x - minI.x + 1;
                        __declspec(align(16)) float* pj = patchJ + y*jsz.width;
                        __declspec(align(16)) float* ix = Ix +
                            (y + minJ.y - minI.y)*(isz.width-2) + minJ.x - minI.x;
                        __declspec(align(16)) float* iy = Iy + (ix - Ix);
#ifndef EN_ASM_6
                        for( x = 0; x < jsz.width; x++ )//LN4
                        {
                            double t0 = pi[x] - pj[x];
                            bx += t0 * ix[x];
                            by += t0 * iy[x];
                        }
#else
						__asm{
							mov ebx, jsz.width
							shr ebx, 2 //Divide by 4 to get the number of iterations to optimize
							shl ebx, 4 //Multiply by 16 to get the number of bytes to process
							           //4 floats per iteration (16 bytes per iteration)
							mov edx, jsz.width
							shl edx, 2 //Total number of bytes to process
							mov ecx, 0h//counter. Bytes processed
						ln4_lesmult_opt:
							cmp ecx, ebx
							jge ln4_lesmult_norm //If ecx >= ebx go to normal iterations
							// float t = pi[x] - pj[x]; xmm0 is t1,t2,t3,t4
							mov esi, pi
							add esi, ecx
							movaps xmm0, WORD PTR [esi]
							mov esi, pj
							add esi, ecx
							movaps xmm1, WORD PTR [esi]
							subps xmm0, xmm1 //xmm0 is t
							// bx += (double) (t * ix[x]);
							mov esi, ix
							add esi, ecx
							movaps xmm1, [esi] //xmm1 <- ix[4]
							mulps xmm1, xmm0 //xmm1 <- ix[4]*t[4]
							movaps xmm3, xmm1
							shufps xmm3, xmm1, 01bh//00011011
							addps xmm1, xmm3
							movaps xmm3, xmm1
							shufps xmm3, xmm1, 0b1h//10110001
							addps xmm1, xmm3
							cvtss2sd xmm1, xmm1
							addsd xmm1, bx
							movsd bx, xmm1
							//by += (double) (t * iy[x]);
							mov esi, iy
							add esi, ecx
							movaps xmm2, [esi]
							mulps xmm2, xmm0
							movaps xmm3, xmm2
							shufps xmm3, xmm2, 01bh
							addps xmm2, xmm3
							movaps xmm3, xmm2
							shufps xmm3, xmm2, 0b1h
							addps xmm2, xmm3
							cvtss2sd xmm2, xmm2
							addsd xmm2, by
							movsd by, xmm2
							add ecx, 16
							jmp ln4_lesmult_opt
						ln4_lesmult_norm:
							cmp ecx, edx
							jge ln4_lesmult_fin //If ecx >= ebx go to normal iterations
							//1
							mov esi, pi
							add esi, ecx
							movss xmm0, [esi]
							mov esi, pj
							add esi, ecx
							movss xmm1, [esi]
							subss xmm0, xmm1 //xmmo is t
							//2
							mov esi, ix
							add esi, ecx
							movss xmm1, [esi]
							mulss xmm1, xmm0
							cvtss2sd xmm1, xmm1
							addsd xmm1, bx
							movsd bx, xmm1
							//3
							mov esi, iy
							add esi, ecx
							movss xmm2, [esi]
							mulss xmm2, xmm0
							cvtss2sd xmm2, xmm2
							addsd xmm2, by
							movsd by, xmm2
							add ecx, 4
							jmp ln4_lesmult_norm
						ln4_lesmult_fin:
						}
#endif
					}
                }
                else
                {
                    Gxx = Gyy = Gxy = 0;
                    for( y = 0; y < jsz.height; y++ )//LN3
                    {
                        const float* pi = patchI +
                            (y + minJ.y - minI.y + 1)*isz.width + minJ.x - minI.x + 1;
                        const float* pj = patchJ + y*jsz.width;
                        const float* ix = Ix +
                            (y + minJ.y - minI.y)*(isz.width-2) + minJ.x - minI.x;
                        const float* iy = Iy + (ix - Ix);
#ifndef EN_ASM_7
                        for( x = 0; x < jsz.width; x++ )//LN4
                        {
                            float t = pi[x] - pj[x];
                            bx += (double) (t * ix[x]);
                            by += (double) (t * iy[x]);
                            Gxx += ix[x] * ix[x];
                            Gxy += ix[x] * iy[x];
                            Gyy += iy[x] * iy[x];
                        }
#else
						__asm{
							mov ebx, jsz.width
							shr ebx, 2 //Divide by 4 to get the number of iterations to optimize
							shl ebx, 4 //Multiply by 16 to get the number of bytes to process
							           //4 floats per iteration (16 bytes per iteration)
							mov edx, jsz.width
							shl edx, 2 //Total number of bytes to process
							mov ecx, 0h//counter. Bytes processed
						ln4_muchmult_opt:
							cmp ecx, ebx
							jge ln4_muchmult_norm //If ecx >= ebx go to normal iterations
							// float t = pi[x] - pj[x]; xmm0 is t1,t2,t3,t4
							mov esi, pi
							add esi, ecx
							movaps xmm0, [esi]
							mov esi, pj
							add esi, ecx
							movaps xmm1, [esi]
							subps xmm0, xmm1 //xmm0 is t
							// bx += (double) (t * ix[x]);
							mov esi, ix
							add esi, ecx
							movaps xmm1, [esi] //xmm1 <- ix[4]
							mulps xmm1, xmm0 //xmm1 <- ix[4]*t[4]
							movaps xmm3, xmm1
							shufps xmm3, xmm1, 01bh//00011011
							addps xmm1, xmm3
							movaps xmm3, xmm1
							shufps xmm3, xmm1, 0b1h//10110001
							addps xmm1, xmm3
							cvtss2sd xmm1, xmm1
							addsd xmm1, bx
							movsd bx, xmm1
							//Gxx += ix[x] * ix[x];
							movaps xmm1, [esi]
							mulps xmm1, xmm1
							movaps xmm3, xmm1
							shufps xmm3, xmm1, 01bh
							addps xmm1, xmm3
							movaps xmm3, xmm1
							shufps xmm3, xmm1, 0b1h
							addps xmm1, xmm3
							addss xmm1, Gxx
							movss Gxx, xmm1
							//by += (double) (t * iy[x]);
							mov esi, iy
							add esi, ecx
							movaps xmm2, [esi]
							mulps xmm2, xmm0
							movaps xmm3, xmm2
							shufps xmm3, xmm2, 01bh
							addps xmm2, xmm3
							movaps xmm3, xmm2
							shufps xmm3, xmm2, 0b1h
							addps xmm2, xmm3
							cvtss2sd xmm2, xmm2
							addsd xmm2, by
							movsd by, xmm2
							//Gyy += iy[x] * iy[x];
							movaps xmm2, [esi]
							mulps xmm2, xmm2
							movaps xmm3, xmm2
							shufps xmm3, xmm2, 01bh
							addps xmm2, xmm3
							movaps xmm3, xmm2
							shufps xmm3, xmm2, 0b1h
							addps xmm2, xmm3
							addss xmm2, Gyy
							movss Gyy, xmm2
							//Gxy += ix[x] * iy[x];
							mov esi, ix
							add esi, ecx
							movaps xmm1, [esi]
							mov esi, iy
							add esi, ecx
							movaps xmm2, [esi]
							mulps xmm1, xmm2
							movaps xmm2, xmm1
							shufps xmm2, xmm1, 01bh
							addps xmm1, xmm2
							movaps xmm2, xmm1
							shufps xmm2, xmm1, 0b1h
							addps xmm1, xmm2
							addss xmm1, Gyy
							movss Gyy, xmm1
							add ecx, 16
							jmp ln4_muchmult_opt
						ln4_muchmult_norm:
							cmp ecx, edx
							jge ln4_muchmult_fin //If ecx >= ebx go to normal iterations
							//1
							mov esi, pi
							add esi, ecx
							movss xmm0, [esi]
							mov esi, pj
							add esi, ecx
							movss xmm1, [esi]
							subss xmm0, xmm1 //xmmo is t
							//2
							mov esi, ix
							add esi, ecx
							movss xmm1, [esi]
							mulss xmm1, xmm0
							cvtss2sd xmm1, xmm1
							addsd xmm1, bx
							movsd bx, xmm1
							//4
							movss xmm1, [esi]
							mulss xmm1, xmm1
							addss xmm1, Gxx
							movss Gxx, xmm1
							//3
							mov esi, iy
							add esi, ecx
							movss xmm2, [esi]
							mulss xmm2, xmm0
							cvtss2sd xmm2, xmm2
							addss xmm2, by
							movss by, xmm2
							//6
							movaps xmm2, [esi]
							mulss xmm2, xmm2
							addss xmm2, Gyy
							movss Gyy, xmm2
							//5
							mov esi, ix
							add esi, ecx
							movss xmm1, [esi]
							mov esi, iy
							add esi, ecx
							movss xmm2, [esi]
							mulss xmm1, xmm2
							addss xmm1, Gyy
							movss Gyy, xmm1
							add ecx, 4
							jmp ln4_muchmult_norm
						ln4_muchmult_fin:
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
                }

                mx = (float) ((Gyy * bx - Gxy * by) * D);
                my = (float) ((Gxx * by - Gxy * bx) * D);

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
            }

            featuresB[i] = v;
            status[i] = (char)pt_status;
            if( l == 0 && error && pt_status ) {

                /* calc error */
                double err = 0;
				if( flags & CV_LKFLOW_GET_MIN_EIGENVALS ) {
                    err = minEig;
				} else {

					for( y = 0; y < jsz.height; y++ ) {//LN3 

                        const float * pi = patchI + (y + minJ.y - minI.y + 1)*isz.width + minJ.x - minI.x + 1;
                        const float * pj = patchJ + y*jsz.width;
#ifndef EN_ASM_8
						for( x = 0; x < jsz.width; x++ ) {//LN4
							float t = pi[x] - pj[x];
							err += t * t;
						}
#else
						__asm{
							mov ebx, jsz.width
							shr ebx, 2 //Divide by 4 to get the number of iterations to optimize
							shl ebx, 4 //Multiply by 16 to get the number of bytes to process
							           //4 floats per iteration (16 bytes per iteration)
							mov edx, jsz.width
							shl edx, 2 //Total number of bytes to process
							mov ecx, 0h//counter. Bytes processed
						//Optimized loop from 0 to jsz.width/4
						ln4_err_opt:
							cmp ecx, ebx
							jge ln4_err_norm //If ecx >= ebx go to normal iterations
							mov esi, pi    //load pi array address
							add esi, ecx   //add counter to address
							movaps xmm1, [esi] //load 4 floats from pi
							mov esi, pj
						    add esi, ecx
							movaps xmm2, [esi] //Load 4 floats from pj
							subps xmm1, xmm2 //4 subs
							mulps xmm1, xmm1 //(pi[x]-pj[x])^2 four numbers
							movaps xmm2, xmm1
							shufps xmm2, xmm1, 01bh//1110 0100
							addps xmm1, xmm2
							movaps xmm2, xmm1
							shufps xmm2, xmm1, 0b1h//1011 0001 (ab ba)
							addps xmm1, xmm2 //each component of xmm1 is (t1^2 + t2^2 + t3^2 + t4^2)
							cvtss2sd xmm1, xmm1
							addsd xmm1, err
							movsd err, xmm1
							add ecx, 16  //We have processed 16 bytes in this round
							jmp ln4_err_opt //Goto loop again
						//non-optimized loop
						ln4_err_norm:
							cmp ecx, edx
							jge ln4_err_fin
							mov esi, pi    //load pi array address
							add esi, ecx   //add counter to address
							movss xmm1, [esi] //load 1 floats from pi
							mov esi, pj
						    add esi, ecx
							movss xmm2, [esi] //Load 1 floats from pj
							subss xmm1, xmm2 //1 subs
							mulss xmm1, xmm1 //(pi[x]-pj[x])^2
							cvtss2sd xmm1, xmm1
							addsd xmm1, err
							movsd err, xmm1
							add ecx, 4 //Add the number of processed bytes
							jmp ln4_err_norm
						//Fin de bucle
						ln4_err_fin:
						}
#endif
					}
                    err = sqrt(err);
                }
                error[i] = (float)err;
            }
        }
		//************** Point processing loop End **************
        
    } // end of pyramid levels loop (l)

    __END__;

    if( ipp_optflow_state )
        icvOpticalFlowPyrLKFree_8u_C1R_p( ipp_optflow_state );

    cvFree( &pyrBuffer );
    cvFree( &buffer );
    cvFree( &_error );
    cvFree( &_status );
}



/* End of file. */
