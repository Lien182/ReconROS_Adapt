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

	struct timespec t_start, t_end, t_res;
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

	init_zycap();

	Prefetch_PR_Bitstream("bitstreams/Sortdemo_ReconfSlotSmall_0");
	Prefetch_PR_Bitstream("bitstreams/Sortdemo_ReconfSlotSmall_1");
	Prefetch_PR_Bitstream("bitstreams/Sobel_ReconfSlotSmall_0");
	Prefetch_PR_Bitstream("bitstreams/Sobel_ReconfSlotSmall_1");
	Prefetch_PR_Bitstream("bitstreams/Mnist_ReconfSlotLarge_0");
	Prefetch_PR_Bitstream("bitstreams/Mnist_ReconfSlotLarge_1");
	Prefetch_PR_Bitstream("bitstreams/Fast_ReconfSlotLarge_0");
	Prefetch_PR_Bitstream("bitstreams/Fast_ReconfSlotLarge_1");

	
	
	clock_gettime(CLOCK_MONOTONIC, &t_start);
	Config_PR_Bitstream("Sortdemo_ReconfSlotSmall_0");
	clock_gettime(CLOCK_MONOTONIC, &t_end);
	timespec_diff(&t_start, &t_end, &t_res);
	printf("Sortdemo_ReconfSlotSmall_0: %3.6f;\n", (double)(t_res.tv_nsec)/1000000000);

	clock_gettime(CLOCK_MONOTONIC, &t_start);
	Config_PR_Bitstream("Sobel_ReconfSlotSmall_1");
	clock_gettime(CLOCK_MONOTONIC, &t_end);
	timespec_diff(&t_start, &t_end, &t_res);
	printf("Sobel_ReconfSlotSmall_1: %3.6f;\n", (double)(t_res.tv_nsec)/1000000000);

	clock_gettime(CLOCK_MONOTONIC, &t_start);
	Config_PR_Bitstream("Mnist_ReconfSlotLarge_0");
	clock_gettime(CLOCK_MONOTONIC, &t_end);
	timespec_diff(&t_start, &t_end, &t_res);
	printf("Mnist_ReconfSlotLarge_0: %3.6f;\n", (double)(t_res.tv_nsec)/1000000000);

	clock_gettime(CLOCK_MONOTONIC, &t_start);
	Config_PR_Bitstream("Fast_ReconfSlotLarge_1");
	clock_gettime(CLOCK_MONOTONIC, &t_end);
	timespec_diff(&t_start, &t_end, &t_res);
	printf("Fast_ReconfSlotLarge_1: %3.6f;\n", (double)(t_res.tv_nsec)/1000000000);

	init_msg();

	threads[0] = reconos_thread_create_hwt_sortdemo	(rsobel_image_msg_out->data.data);
	threads[1] = reconos_thread_create_hwt_sobel	(0);
	threads[2] = reconos_thread_create_hwt_mnist	(&rmnist_output_msg->data);
	
	
	orbslam_settings.fast_threads[0] = reconos_thread_create_hwt_fast		(0);
	orbslam_settings.orbslam_thread  = reconos_thread_create_swt_orbslam((void*)&orbslam_settings,0);



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