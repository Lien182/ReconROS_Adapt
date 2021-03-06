#include <stdint.h>
#include <math.h>
#include "LeNet.h"
#include "parameters.h"

#define INPUT_BATCH_SZ	1
#define INPUT_N_ROWS 28
#define INPUT_N_COLS 28


void read_image(uint32_t ram[INPUT_BATCH_SZ*INPUT_N_ROWS*INPUT_N_COLS/4], uint8_t image[INPUT_BATCH_SZ*INPUT_N_ROWS*INPUT_N_COLS]){
	// Read the input stream and put the values in the image array

	int b, r, c, i = 0;

	batch_rd: for(b=0; b<INPUT_BATCH_SZ; b++){
		row_rd: for (r=0; r<INPUT_N_ROWS; r++) {
			col_rd: for (c=0; c<INPUT_N_COLS; c+=4){
				image[b*INPUT_N_COLS*INPUT_N_COLS + INPUT_N_COLS*r + c]   = (uint8_t)((ram[i] & 0x0000ff));
				image[b*INPUT_N_COLS*INPUT_N_COLS + INPUT_N_COLS*r + c+1] = (uint8_t)((ram[i] & 0x0000ff00) >> 8);
				image[b*INPUT_N_COLS*INPUT_N_COLS + INPUT_N_COLS*r + c+2] = (uint8_t)((ram[i] & 0x00ff0000) >> 16); 		
				image[b*INPUT_N_COLS*INPUT_N_COLS + INPUT_N_COLS*r + c+3] = (uint8_t)((ram[i] & 0xff000000) >> 24); 
				i++;
			}
		}
	}

	return;
}

uint32_t calc_mnist(uint8_t * data)
{

	uint32_t ram[INPUT_BATCH_SZ*INPUT_N_ROWS*INPUT_N_COLS/4];
	uint8_t image[INPUT_BATCH_SZ*INPUT_N_ROWS*INPUT_N_COLS];
	float src[BUFFER_SIZE];
	float dst[CLASSES];

	read_image(ram, image);

	for(int i=0; i<1; i++)
	{
		char tmp;
		for(int batch=0; batch<1; batch++)
		{
			for(int rows = 0; rows < 32 ; rows++)
			{
				for(int cols = 0; cols < 32; cols++)
				{
					float scaled;
					uint8_t temp;

					if(cols<2 || rows<2 || cols>=30|| rows>=30)
					{
						temp = 0;
					}
					else
					{
						temp = image[batch*1024+(rows-2)*28+cols-2];
					}

					scaled =  ((((float)temp * (float)2) / (float)255 )- (float)1 );
					src[rows*INPUT_WH+cols] = (float)(scaled*DATA_CONVERT_MUL);
				}

			}

		}
	}
		

	LeNet(src, dst, 0);

	float result[CLASSES];

	float max_num = -10000;
	int max_id = 0;
	for(int index=0; index<10; index++){
		int tmp = dst[index];
		result[index] = (float)tmp/(float)DATA_CONVERT_MUL;
		if(result[index] > max_num){
			max_num = result[index];
			max_id = index;
		}
	}

	return  max_id;
}
