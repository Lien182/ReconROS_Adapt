#include "hw_executor.h"
#include <time.h>
#include <stdio.h>
#include "time_measurement.h"



static void * ReconROS_HWExecutor_Agent(void * args)
{
	int bytes_moved;
    struct timespec t_start, t_end, t_res;
    t_reconros_hwexecutor * reconros_hwexecutor = ( t_reconros_hwexecutor *)args;
	int callbackid_old = -1;
	uint32_t nCallbackRetention = 0x40000000;

	while(reconros_hwexecutor->bRun)
	{
		
		struct reconos_thread * hwthread;
		t_bitstream * bitstream;
		void * message;

		//printf("[ReconROS_HWExecutor_Agent %d] Check for HW callback \n", reconros_hwexecutor->uSlotid);
		int callbackid = Callbacklist_GetHWCallback(reconros_hwexecutor->pCallbacklists, reconros_hwexecutor->uSlotMask, &nCallbackRetention, &hwthread, &bitstream, &message);
		//printf("[ReconROS_HWExecutor_Agent %d] Hw callback checked; callbackid=%d \n",reconros_hwexecutor->uSlotid, callbackid);
		if(callbackid < 0)
		{
			usleep(10000);
		}
		else
		{
			if(callbackid_old != callbackid)
			{
				clock_gettime(CLOCK_MONOTONIC, &t_start);
				bytes_moved = Zycap_Write_Bitstream(reconros_hwexecutor->Zycap, bitstream);
				clock_gettime(CLOCK_MONOTONIC, &t_end);
				timespec_diff(&t_start, &t_end, &t_res);
				printf("[ReconROS_HWExecutor_Agent %d] reconfiguration time %3.6f; bytes_moved = %d\n", reconros_hwexecutor->uSlotid, (double)(t_res.tv_nsec)/1000000000, bytes_moved);
				usleep(10000);
			}
			else
			{
				printf("[ReconROS_HWExecutor_Agent %d] no reconfiguration needed since received bitstream still in the slot\n");
			}

			reconos_thread_setinitdata(hwthread, message);
			//printf("[ReconROS_HWExecutor_Agent %d] Going to execute the function \n", reconros_hwexecutor->uSlotid);
			reconos_thread_resume(hwthread, reconros_hwexecutor->uSlotid);
			//printf("[ReconROS_HWExecutor_Agent %d] Wait for finishing \n", reconros_hwexecutor->uSlotid);
			reconos_thread_join(hwthread);
			//reconos_thread_suspend(hwthread);

			
			Callbacklist_Release(reconros_hwexecutor->pCallbacklists, callbackid);
		}
	}


	return 0;


}



int ReconROS_HWExecutor_Init(t_reconros_hwexecutor * reconros_hwexecutor, t_zycap * zycap, t_callback_lists * pCallbacklists, uint32_t nSlotId)	//bit n = true for 
{
	reconros_hwexecutor->bRun = 0;
	reconros_hwexecutor->Zycap = zycap;
	reconros_hwexecutor->pCallbacklists = pCallbacklists;
	reconros_hwexecutor->ptAgent = NULL;
	reconros_hwexecutor->uSlotid = nSlotId;
	reconros_hwexecutor->uSlotMask = (1<< nSlotId);
	return 0;
}

int ReconROS_HWExecutor_Spin(t_reconros_hwexecutor * reconros_hwexecutor)
{

	printf("test \n");
	reconros_hwexecutor->bRun = 1UL;
	printf("[ReconROS_HWExecutor_Spin] start agent for slot id %d \n", reconros_hwexecutor->uSlotid);

	if(pthread_create(&reconros_hwexecutor->ptAgent, 0, ReconROS_HWExecutor_Agent, (void*)reconros_hwexecutor) != 0)
	{
		printf("[ReconROS_HWExecutor_Spin] Failed to create agent for slot %d \n", reconros_hwexecutor->uSlotid);
		return -1;
    }
	return 0;
}

int ReconROS_HWExecutor_Join(t_reconros_hwexecutor * reconros_hwexecutor)
{
	uint32_t ret_value;
	pthread_join(reconros_hwexecutor->ptAgent, (void**)&ret_value);
	return 0;
}


int ReconROS_HWExecutor_Terminate(t_reconros_hwexecutor * reconros_hwexecutor)
{
	reconros_hwexecutor->bRun = 0UL;

	return 0;
}