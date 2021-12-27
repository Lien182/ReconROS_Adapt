#include <stdio.h>
#include <sys/time.h>

#include "reconos.h"
#include "reconos_thread.h"
#include "reconos_calls.h"


#include "sha256.h"

THREAD_ENTRY()
{
	THREAD_INIT();
	uint32_t arg = (uint32_t)GET_INIT_DATA();
	BYTE buf[SHA256_BLOCK_SIZE];

	SHA256_CTX ctx;
	int idx;
	int pass = 1;

	sha256_init(&ctx);

	memcpy(buf, (BYTE*)&arg, 4);
	for(int i = 0; i < 10000; i++)
	{
		sha256_update(&ctx, buf, SHA256_BLOCK_SIZE);
		sha256_final(&ctx, buf);
	}

	memcpy(&rperiodic_output_msg->data, buf, 4);
	ROS_PUBLISH(rperiodic_pub_out,rperiodic_output_msg );
	

	//struct timeval  tv;
	//gettimeofday(&tv, NULL);

	//double time_in_mill = 
    //     (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000 ; // convert tv_sec & tv_usec to millisecond


	//printf("Periodic thread was called! %.2f \n", time_in_mill);
	
}
