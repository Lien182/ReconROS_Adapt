#include "hw_executor.h"

#include <time.h>





static void * ReconROS_HWExecutor_Agent(void * args)
{

    int bytes_moved;
    struct timespec t_start, t_end, t_res;
    t_reconros_hwexecutor * reconros_hwexecutor = ( t_reconros_hwexecutor *)args;

	clock_gettime(CLOCK_MONOTONIC, &t_start);
	bytes_moved = Zycap_Write_Bitstream(&(reconros_hwexecutor->Zycap), 0);
	clock_gettime(CLOCK_MONOTONIC, &t_end);
	timespec_diff(&t_start, &t_end, &t_res);
	printf("ReconROS_HWExecutor_Agent Slot %d: reconfiguration time %3.6f; bytes_moved = %d\n", reconros_hwexecutor->uSlotid, (double)(t_res.tv_nsec)/1000000000, bytes_moved);

}



void ReconROS_HWExecutor_Init(t_reconros_hwexecutor * reconros_hwexecutor, t_zycap * zycap, uint32_t nSlots)
{

}

void ReconROS_HWExecutor_Spin(t_reconros_hwexecutor * reconros_hwexecutor)
{

}