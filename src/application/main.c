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

#include "executor.h"


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

int main(int argc, char **argv) 
{


	t_reconros_executor reconros_executor;
	
	if(argc != 4)
    {
        printf("Usage: ./reconfadapt (orb)path_to_vocabulary (orb)path_to_settings (orb)<hw/sw>\n");
        return 1;
    }

	
	//parse orbslam settings
	// t_orbslam_settings orbslam_settings;
	
	// if(strcmp(argv[3], "hw") == 0)
    // {
    //     orbslam_settings.bUseHw = 1;
    // }
    // else
    // {
    //     orbslam_settings.bUseHw = 0;
    // }



	reconos_init();
	reconos_app_init();


	ReconROS_Executor_Init(&reconros_executor, 4, 2, "/mnt/bitstreams/");

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

	

	threads[0] = reconos_thread_create_hwt_sortdemo	(0);
	threads[1] = reconos_thread_create_hwt_sobel	(rsobel_image_msg_out->data.data);
	threads[2] = reconos_thread_create_hwt_mnist	(&rmnist_output_msg->data);
	
	//orbslam_settings.fast_threads[0] = reconos_thread_create_hwt_fast		(0);
	//orbslam_settings.orbslam_thread  = reconos_thread_create_swt_orbslam((void*)&orbslam_settings,0);


	

	ReconROS_Executor_Spin();

	
	reconos_app_cleanup();
	reconos_cleanup();

	return 0;
}