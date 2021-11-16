#define  _GNU_SOURCE

#define RECONOS_DEBUG

#include "reconos.h"
#include "reconos_app.h"
#include "mbox.h"
#include "timer.h"

#include "utils.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <limits.h>

#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>

#include "main.h"

#include "zycap_linux.h"


#define NSLOTS 4
#define NTHREADS 5

#define TINVERSE 	0
#define TSORT		1
#define TSOBEL		2
#define TMNIST		3
#define TFAST		4



void init_msg(void)
{
	//std_msgs__msg__Header header;
  	rsobel_image_msg_out->height = IMAGE_HEIGHT;
  	rsobel_image_msg_out->width = IMAGE_WIDTH;
  	rsobel_image_msg_out->encoding.data = "bgr8";
	rsobel_image_msg_out->encoding.size = 4;  
	rsobel_image_msg_out->encoding.capacity = 5;  
  	rsobel_image_msg_out->is_bigendian = 0;
  	rsobel_image_msg_out->step = IMAGE_WIDTH*3;
  	rsobel_image_msg_out->data.data = malloc(IMAGE_HEIGHT*IMAGE_WIDTH*3);
	rsobel_image_msg_out->data.size = IMAGE_HEIGHT*IMAGE_WIDTH*3;
	rsobel_image_msg_out->data.capacity = IMAGE_HEIGHT*IMAGE_WIDTH*3;

}


void timespec_diff(struct timespec *start, struct timespec *stop,
                   struct timespec *result)
{
    if ((stop->tv_nsec - start->tv_nsec) < 0) {
        result->tv_sec = stop->tv_sec - start->tv_sec - 1;
        result->tv_nsec = stop->tv_nsec - start->tv_nsec + 1000000000;
    } else {
        result->tv_sec = stop->tv_sec - start->tv_sec;
        result->tv_nsec = stop->tv_nsec - start->tv_nsec;
    }

    return;
}


int main(int argc, char **argv) 
{


	uint32_t nThreads = 0;
	
	if(argc != 4)
    {
        printf("Usage: ./reconfadapt (orb)path_to_vocabulary (orb)path_to_settings (orb)<hw/sw>\n");
        return 1;
    }

	
	//parse orbslam settings
	t_orbslam_settings orbslam_settings;
	
	if(strcmp(argv[3], "hw") == 0)
    {
        orbslam_settings.bUseHw = 1;
    }
    else
    {
        orbslam_settings.bUseHw = 0;
    }



	reconos_init();
	reconos_app_init();

	uint32_t inverse_result;
	struct reconos_thread* threads[8];


	t_bitstream bitstreams[NSLOTS][NTHREADS];


	for(int i = 0; i < NSLOTS; i++)
	{
		for( int j = 0; j < NTHREADS; j++)
		{
			bitstreams[i][j].data = 0;
			bitstreams[i][j].size = 0;
			
		}
	}


	Zycap_Prefetch_Bitstream("/mnt/bitstreams/pblock_slot_0_inverse_0_partial.bit", 	&bitstreams[0][TINVERSE]);
	Zycap_Prefetch_Bitstream("/mnt/bitstreams/pblock_slot_0_sobel_0_partial.bit", 		&bitstreams[0][TSOBEL]);
	Zycap_Prefetch_Bitstream("/mnt/bitstreams/pblock_slot_0_sortdemo_0_partial.bit", 	&bitstreams[0][TSORT]);
	Zycap_Prefetch_Bitstream("/mnt/bitstreams/pblock_slot_1_inverse_1_partial.bit", 	&bitstreams[1][TINVERSE]);
	Zycap_Prefetch_Bitstream("/mnt/bitstreams/pblock_slot_1_sobel_1_partial.bit", 		&bitstreams[1][TSOBEL]);
	Zycap_Prefetch_Bitstream("/mnt/bitstreams/pblock_slot_1_sortdemo_1_partial.bit", 	&bitstreams[1][TSORT]);
	Zycap_Prefetch_Bitstream("/mnt/bitstreams/pblock_slot_2_fast_2_partial.bit", 		&bitstreams[2][TFAST]);
	Zycap_Prefetch_Bitstream("/mnt/bitstreams/pblock_slot_2_mnist_2_partial.bit", 		&bitstreams[2][TMNIST]);
	Zycap_Prefetch_Bitstream("/mnt/bitstreams/pblock_slot_3_fast_3_partial.bit", 		&bitstreams[3][TFAST]);
	Zycap_Prefetch_Bitstream("/mnt/bitstreams/pblock_slot_3_mnist_3_partial.bit", 		&bitstreams[3][TMNIST]);

	int bytes_moved = 0;	
	
	

	threads[0] = reconos_thread_create_hwt_sortdemo	(0);
	threads[1] = reconos_thread_create_hwt_sobel	(rsobel_image_msg_out->data.data);
	threads[2] = reconos_thread_create_hwt_mnist	(&rmnist_output_msg->data);
	
	//orbslam_settings.fast_threads[0] = reconos_thread_create_hwt_fast		(0);
	//orbslam_settings.orbslam_thread  = reconos_thread_create_swt_orbslam((void*)&orbslam_settings,0);



	uint32_t count = 0;
	uint32_t nMsgSobel = 0;
	uint32_t nMsgSort = 0;
	uint32_t nMsgMnist = 0;

	



	while(1)
	{
		
		usleep(200000);

		
		uint32_t ret = 0;

		ret |= rcl_subscription_get_unread_count(&rsobel_subdata->sub, &nMsgSobel);
		ret |= rcl_service_get_unread_requests(&rsort_srv->service, 	&nMsgSort);
		ret |= rcl_subscription_get_unread_count(&rmnist_subdata->sub, &nMsgMnist);
		
		
		printf("Unread requests: ret = %d \n", ret);
		printf("Sobel=%06d,Mnist=%06d,Sort=%06d \n", nMsgSobel ,nMsgMnist, nMsgSort);

	}

	reconos_app_cleanup();
	reconos_cleanup();

	return 0;
}