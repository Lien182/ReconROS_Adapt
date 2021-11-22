#include "sw_executor.h"
#include "executor.h"


static void * ReconROS_SWExecutor_Agent(void * args)
{

    t_reconros_swexecutor * reconros_swexecutor = ( t_reconros_swexecutor *)args;


	while(reconros_swexecutor->bRun)
	{
		;//ReconROS_GetSWCallback()
	}



	return 0;

}



void ReconROS_SWExecutor_Init(t_reconros_swexecutor * reconros_swexecutor,  int id)
{
	reconros_swexecutor->id = id;
}

int ReconROS_SWExecutor_Spin(t_reconros_swexecutor * reconros_swexecutor)
{

	if (pthread_create(&reconros_swexecutor->ptAgent, NULL, ReconROS_SWExecutor_Agent, (void*)reconros_swexecutor) != 0) 
	{
		return -1;
    }

	return 0;
}

int ReconROS_SWExecutor_Join(t_reconros_swexecutor * reconros_swexecutor)
{
	uint32_t ret_value;
	pthread_join(reconros_swexecutor->ptAgent, (void**)&ret_value);
	return 0;
}