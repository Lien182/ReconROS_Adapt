#include "sw_executor.h"


static void * ReconROS_SWExecutor_Agent(void * args)
{

    t_reconros_swexecutor * reconros_swexecutor = ( t_reconros_swexecutor *)args;


	while(reconros_swexecutor->bRun)
	{

	}



	return 0;

}



void ReconROS_SWExecutor_Init(t_reconros_swexecutor * reconros_swexecutor, int id)
{

}

int ReconROS_SWExecutor_Spin(t_reconros_swexecutor * reconros_swexecutor)
{

	if (pthread_create(&reconros_swexecutor->ptAgent, NULL, ReconROS_SWExecutor_Agent, (void*)reconros_swexecutor) != 0) 
	{
		return -1;
    }

	return 0;
}