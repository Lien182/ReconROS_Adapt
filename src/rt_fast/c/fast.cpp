#include <iostream>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#define W 30
#define N_KEYPOINTS         300
#define FAST_WINDOW_SIZE 	50
#define IMAGE_CACHE_WIDTH   1280
#define IMAGE_CACHE_HEIGHT  150

extern "C" {
#include "reconos_thread.h"
#include "reconos_calls.h"
}
using namespace std;

#define uchar unsigned char

static const int EDGE_THRESHOLD = 19;
static const int iniThFAST = 20;
static const int minThFAST = 14;

#define AP_MAX(a, b) ((a) > (b) ? (a) : (b))
#define AP_MIN(a, b) ((a) < (b) ? (a) : (b))

extern "C" THREAD_ENTRY(); // this is required because of the mixture of c and c++


int cornerScore(const uchar* ptr, const int pixel[], int threshold)
{
    const int K = 8, N = K*3 + 1;
    int k, v = ptr[0];
    short d[N];
    for( k = 0; k < N; k++ )
        d[k] = (short)(v - ptr[pixel[k]]);

    {

        int a0 = threshold;
        for( k = 0; k < 16; k += 2 )
        {
            int a = AP_MIN((int)d[k+1], (int)d[k+2]);
            a = AP_MIN(a, (int)d[k+3]);
            if( a <= a0 )
                continue;
            a = AP_MIN(a, (int)d[k+4]);
            a = AP_MIN(a, (int)d[k+5]);
            a = AP_MIN(a, (int)d[k+6]);
            a = AP_MIN(a, (int)d[k+7]);
            a = AP_MIN(a, (int)d[k+8]);
            a0 = AP_MAX(a0, AP_MIN(a, (int)d[k]));
            a0 = AP_MAX(a0, AP_MIN(a, (int)d[k+9]));
        }

        int b0 = -a0;
        for( k = 0; k < 16; k += 2 )
        {
            int b = AP_MAX((int)d[k+1], (int)d[k+2]);
            b = AP_MAX(b, (int)d[k+3]);
            b = AP_MAX(b, (int)d[k+4]);
            b = AP_MAX(b, (int)d[k+5]);
            if( b >= b0 )
                continue;
            b = AP_MAX(b, (int)d[k+6]);
            b = AP_MAX(b, (int)d[k+7]);
            b = AP_MAX(b, (int)d[k+8]);

            b0 = AP_MIN(b0, AP_MAX(b, (int)d[k]));
            b0 = AP_MIN(b0, AP_MAX(b, (int)d[k+9]));
        }

        threshold = -b0 - 1;
    }

    return threshold;
}


void makeOffsets(int pixel[25], int rowStride, int patternSize)
{
    static const int offsets16[][2] =
    {
        {0,  3}, { 1,  3}, { 2,  2}, { 3,  1}, { 3, 0}, { 3, -1}, { 2, -2}, { 1, -3},
        {0, -3}, {-1, -3}, {-2, -2}, {-3, -1}, {-3, 0}, {-3,  1}, {-2,  2}, {-1,  3}
    };

    static const int offsets12[][2] =
    {
        {0,  2}, { 1,  2}, { 2,  1}, { 2, 0}, { 2, -1}, { 1, -2},
        {0, -2}, {-1, -2}, {-2, -1}, {-2, 0}, {-2,  1}, {-1,  2}
    };

    static const int offsets8[][2] =
    {
        {0,  1}, { 1,  1}, { 1, 0}, { 1, -1},
        {0, -1}, {-1, -1}, {-1, 0}, {-1,  1}
    };

    const int (*offsets)[2] = patternSize == 16 ? offsets16 :
                              patternSize == 12 ? offsets12 :
                              patternSize == 8  ? offsets8  : 0;

  //  CV_Assert(pixel && offsets);

    int k = 0;
    for( ; k < patternSize; k++ )
        pixel[k] = offsets[k][0] + offsets[k][1] * rowStride;
    for( ; k < 25; k++ )
        pixel[k] = pixel[k - patternSize];
}

uint32_t cvFAST(uint8_t _img[FAST_WINDOW_SIZE*FAST_WINDOW_SIZE], uint32_t keypoints[N_KEYPOINTS*2], int threshold, bool nonmax_suppression, uint32_t x_offset, uint32_t y_offset)
{
    uint32_t u32Index = 0;
    const int patternSize = 16;

    const int K = patternSize/2, N = patternSize + K + 1;
    int i, j, k, pixel[25];
    makeOffsets(pixel, (int)FAST_WINDOW_SIZE, patternSize);

    threshold = AP_MIN(AP_MAX(threshold, 0), 255);

    uchar threshold_tab[512];
    for( i = -255; i <= 255; i++ )
        threshold_tab[i+255] = (uchar)(i < -threshold ? 1 : i > threshold ? 2 : 0);

    uchar buf[3][FAST_WINDOW_SIZE] = { 0 };
    int cpbuf[3][FAST_WINDOW_SIZE+1] = { 0 };

    for (unsigned idx = 0; idx < 3; ++idx)
    {
        for(unsigned jdx = 0; jdx < FAST_WINDOW_SIZE; jdx++)
            buf[idx][jdx] = 0;
    }

    for(i = 3; i < FAST_WINDOW_SIZE-2; i++)
    {
        const uchar* ptr = &_img[i*FAST_WINDOW_SIZE] + 3;
        uchar* curr = buf[(i - 3)%3];
        int* cornerpos = cpbuf[(i - 3)%3] + 1; // cornerpos[-1] is used to store a value
        
        for(unsigned jdx = 0; jdx < FAST_WINDOW_SIZE; jdx++)
            curr[jdx] = 0;
        //memset(curr, 0, FAST_WINDOW_SIZE);
        int ncorners = 0;

        if( i < FAST_WINDOW_SIZE - 3 )
        {
            j = 3;
            for( ; j < FAST_WINDOW_SIZE - 3; j++, ptr++ )
            {
                int v = ptr[0];
                const uchar* tab = &threshold_tab[0] - v + 255;
                int d = tab[ptr[pixel[0]]] | tab[ptr[pixel[8]]];

                if( d == 0 )
                    continue;

                d &= tab[ptr[pixel[2]]] | tab[ptr[pixel[10]]];
                d &= tab[ptr[pixel[4]]] | tab[ptr[pixel[12]]];
                d &= tab[ptr[pixel[6]]] | tab[ptr[pixel[14]]];

                if( d == 0 )
                    continue;

                d &= tab[ptr[pixel[1]]] | tab[ptr[pixel[9]]];
                d &= tab[ptr[pixel[3]]] | tab[ptr[pixel[11]]];
                d &= tab[ptr[pixel[5]]] | tab[ptr[pixel[13]]];
                d &= tab[ptr[pixel[7]]] | tab[ptr[pixel[15]]];

                if( d & 1 )
                {
                    int vt = v - threshold, count = 0;

                    for( k = 0; k < N; k++ )
                    {
                        int x = ptr[pixel[k]];
                        if(x < vt)
                        {
                            if( ++count > K )
                            {
                                cornerpos[ncorners++] = j;
                                if(nonmax_suppression)
                                    curr[j] = (uchar)cornerScore(ptr, pixel, threshold);
                                break;
                            }
                        }
                        else
                            count = 0;
                    }
                }

                if( d & 2 )
                {
                    int vt = v + threshold, count = 0;

                    for( k = 0; k < N; k++ )
                    {
                        int x = ptr[pixel[k]];
                        if(x > vt)
                        {
                            if( ++count > K )
                            {
                                cornerpos[ncorners++] = j;
                                if(nonmax_suppression)
                                    curr[j] = (uchar)cornerScore(ptr, pixel, threshold);
                                break;
                            }
                        }
                        else
                            count = 0;
                    }
                }
            }
        }

        cornerpos[-1] = ncorners;

        if( i == 3 )
            continue;

        const uchar* prev = buf[(i - 4 + 3)%3];
        const uchar* pprev = buf[(i - 5 + 3)%3];
        cornerpos = cpbuf[(i - 4 + 3)%3] + 1; // cornerpos[-1] is used to store a value
        ncorners = cornerpos[-1];

        for( k = 0; k < ncorners; k++ )
        {
            j = cornerpos[k];
            int score = prev[j];
            if( !nonmax_suppression ||
               (score > prev[j+1] && score > prev[j-1] &&
                score > pprev[j-1] && score > pprev[j] && score > pprev[j+1] &&
                score > curr[j-1] && score > curr[j] && score > curr[j+1]) )
            {
                //keypoints.push_back(KeyPoint((float)j, (float)(i-1), 7.f, -1, (float)score));
                keypoints[u32Index] = (((uint32_t)(j + x_offset)) | ((((uint32_t)(i-1)) + y_offset) << 16));
                keypoints[u32Index+1] = score;
                u32Index+=2;
            }
        }
    }

    return u32Index;
}





void mat_copy( uint8_t * data, uint8_t* _dest, int iniY, int iniX, int cache_cnt,uint32_t image_ptr_offset, uint32_t image_width)
{
    for(int i = 0; i < (FAST_WINDOW_SIZE); i++)
    {
        uint32_t line_index = (iniY+i);
        uint32_t offset = (image_ptr_offset +  line_index*(image_width))&3;

        for(int j = 0; j < (FAST_WINDOW_SIZE); j++)
        {
            _dest[j+FAST_WINDOW_SIZE*i] = data[offset + (j+iniX)+ (line_index %IMAGE_CACHE_HEIGHT) *IMAGE_CACHE_WIDTH];
        }
    }
}

#define read_next_lines {for(int __i = 0; __i < FAST_WINDOW_SIZE; __i++){ \
                                    uint32_t _offset = (image_ptr + cache_cnt*(image_step))&3; \
                                    MEM_READ(((image_ptr + cache_cnt*(image_step))&(~3)), &image_data[(cache_cnt%IMAGE_CACHE_HEIGHT)*IMAGE_CACHE_WIDTH],((image_width+_offset+3)&(~3)));\
                                    cache_cnt+=1;} }



THREAD_ENTRY() {

    uint32_t nresults;

    THREAD_INIT();
	uint32_t initdata = (uint32_t)GET_INIT_DATA();

	struct mbox * mbox_request;
	struct mbox * mbox_response;

	if(initdata == 0)
	{
		mbox_request  = rorbslam_fast_request_0;
		mbox_response = rorbslam_fast_response_0;
	} 
	else
	{
		mbox_request  = rorbslam_fast_request_1;
		mbox_response = rorbslam_fast_response_1;
	}


    uint8_t image_data[IMAGE_CACHE_HEIGHT*IMAGE_CACHE_WIDTH];

	while(1)
	{
        uint32_t cache_cnt = 0;
        uint32_t nWrittenPoints = 0;
        uint32_t image_ptr    = MBOX_GET(mbox_request);
        uint32_t image_width  = MBOX_GET(mbox_request);
        uint32_t image_height = MBOX_GET(mbox_request);
        uint32_t image_step   = MBOX_GET(mbox_request);
        uint32_t feature_dest = MBOX_GET(mbox_request);

        uint32_t image_ptr_offset = image_ptr & 3;
	    const int minBorderX = EDGE_THRESHOLD-3;
        const int minBorderY = minBorderX;
        const int maxBorderX = image_width -EDGE_THRESHOLD+3;
        const int maxBorderY = image_height-EDGE_THRESHOLD+3;

        const int width = (maxBorderX-minBorderX);
        const int height = (maxBorderY-minBorderY);

        const int wCell = 50;
        const int hCell = 50;
        const int nCols = width/50;
        const int nRows = height/50;

        read_next_lines;

		for(int i=0; i<nRows; i++)
		{
			read_next_lines;

			const int iniY =minBorderY+i*hCell; //this was float
			int maxY = iniY+hCell+6; //this was float

			if(iniY>=maxBorderY-3)
				continue;
			if(maxY>maxBorderY)
				maxY = maxBorderY;

			for(int j=0; j<nCols; j++)
			{
				const int iniX =minBorderX+j*wCell; //this was float
				int maxX = iniX+wCell+6;//this was float
				if(iniX>=maxBorderX-6)
					continue;
				if(maxX>maxBorderX)
					maxX = maxBorderX;

				uint32_t vKeysCell[N_KEYPOINTS*2];
				uint32_t nPoints;
				uint8_t img_tmp[FAST_WINDOW_SIZE*FAST_WINDOW_SIZE];
				{
					nPoints = 0;
					mat_copy( image_data, img_tmp, iniY, iniX, cache_cnt, image_ptr_offset, image_width);
					nPoints = cvFAST(img_tmp, vKeysCell,  minThFAST, true, j*wCell, i*hCell);
					MEM_WRITE(vKeysCell, feature_dest, nPoints*4);
                	feature_dest+=(4*nPoints);
                	nWrittenPoints+= nPoints;
				}

			}
		}

        MBOX_PUT(mbox_response, nWrittenPoints);
	}

}