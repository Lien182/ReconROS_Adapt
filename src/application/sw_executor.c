#include "sw_executor.h"
#include "executor.h"

#include <stdio.h>
#include <unistd.h>


static void * ReconROS_SWExecutor_Agent(void * args)
{

    t_reconros_swexecutor * reconros_swexecutor = ( t_reconros_swexecutor *)args;

	function_ptr pCallback;
	void * message;

	while(reconros_swexecutor->bRun)
	{
		printf("[ReconROS_SWExecutor_Agent] Check the callback \n");
		if(Callbacklist_GetSWCallback(reconros_swexecutor->callbacklists,&pCallback, &message) < 0)
		{
			usleep(10000);
		}
		else
		{
			printf("[ReconROS_SWExecutor_Agent] Going to execute the function \n");

			function_ptr(message);
		}


	}



	return 0;

}



int ReconROS_SWExecutor_Init(t_reconros_swexecutor * reconros_swexecutor, t_callback_lists * callbacklists, int id)
{
	reconros_swexecutor->id = id;
	reconros_swexecutor->callbacklists = callbacklists;

	return 0;
}

int ReconROS_SWExecutor_Spin(t_reconros_swexecutor * reconros_swexecutor)
{

	reconros_swexecutor->bRun = 1UL;

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