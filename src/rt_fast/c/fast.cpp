

#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#define W 30


extern "C" {
#include "reconos_thread.h"
#include "reconos_calls.h"
}

//using namespace std;

static const int EDGE_THRESHOLD = 19;
static const int iniThFAST = 20;
static const int minThFAST = 7;

extern "C" THREAD_ENTRY(); // this is required because of the mixture of c and c++

THREAD_ENTRY() {

	while (1) 
	{
		
		uint32_t   image_ptr    = MBOX_GET((mbox*)data);
		uint32_t   image_width  = MBOX_GET((mbox*)data);
		uint32_t   image_height = MBOX_GET((mbox*)data);
        uint32_t   feature_dest = MBOX_GET((mbox*)data);

/*
		const int minBorderX = EDGE_THRESHOLD-3;
    	const int minBorderY = minBorderX;
    	const int maxBorderX = image_width -EDGE_THRESHOLD+3;
    	const int maxBorderY = image_height-EDGE_THRESHOLD+3;

		const float width = (maxBorderX-minBorderX);
        const float height = (maxBorderY-minBorderY);

        const int nCols = width/W;
        const int nRows = height/W;
        const int wCell = ceil(width/nCols);
        const int hCell = ceil(height/nRows);

        for(int i=0; i<nRows; i++)
        {
            const float iniY =minBorderY+i*hCell;
            float maxY = iniY+hCell+6;

            if(iniY>=maxBorderY-3)
                continue;
            if(maxY>maxBorderY)
                maxY = maxBorderY;

            for(int j=0; j<nCols; j++)
            {
                const float iniX =minBorderX+j*wCell;
                float maxX = iniX+wCell+6;
                if(iniX>=maxBorderX-6)
                    continue;
                if(maxX>maxBorderX)
                    maxX = maxBorderX;

                vector<cv::KeyPoint> vKeysCell;
				
               	FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                     vKeysCell,14,true);


                //if(vKeysCell.empty())
                //{
                //    FPGA::FPGA_FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                //         vKeysCell,minThFAST,true);
                //}
                

                if(!vKeysCell.empty())
                {
                    for(vector<cv::KeyPoint>::iterator vit=vKeysCell.begin(); vit!=vKeysCell.end();vit++)
                    {
                        (*vit).pt.x+=j*wCell;
                        (*vit).pt.y+=i*hCell;
                        vToDistributeKeys.push_back(*vit);
                    }
                }

            }

            */
        }

	}
