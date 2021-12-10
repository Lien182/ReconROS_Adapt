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
	for(int i = 0; i < 4; i++)
	{
		#pragma hls dataflow
		sha256_update(&ctx, buf, SHA256_BLOCK_SIZE);
		sha256_final(&ctx, buf);
	}

	memcpy(&rperiodic_output_msg->data, buf, 4);
	ROS_PUBLISH(rperiodic_pub_out,rperiodic_output_msg );
	
}
