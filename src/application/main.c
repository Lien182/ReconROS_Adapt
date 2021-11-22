#define  _GNU_SOURCE

#define RECONOS_DEBUG

#include "reconos.h"
#include "reconos_app.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "executor.h"

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


extern void *rt_sortdemo(void *data);
extern struct reconos_resource *resources_sortdemo[];


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
	ReconROS_Executor_Add_SW_Callback(&reconros_executor, "sortdemo", &rt_sortdemo, ReconROS_SRV, rsort_srv, rsort_sort_srv_req, resources_sortdemo, 4);



	ReconROS_Executor_Spin();

	
	reconos_app_cleanup();
	reconos_cleanup();

	return 0;
}