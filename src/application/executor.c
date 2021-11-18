#include "executor.h"

#include <stdlib.h>
#include <stdio.h>


#include "ros_timer.h"
#include "ros_sub.h"
#include "ros_service_server.h"
#include "ros_service_client.h"

int ReconROS_Executor_Init(t_reconros_executor * reconros_executor, uint32_t nSlots, uint32_t nSwThreads, char * bitstream_dir)
{
    reconros_executor->nExecutorsHw = nSlots;
    reconros_executor->nExecutorsSw = nSwThreads;

    reconros_executor->pBitstreamDir = malloc(strlen(bitstream_dir));
    if(reconros_executor->pBitstreamDir == 0)
        return -1;

    strcpy(reconros_executor->pBitstreamDir, bitstream_dir);

    if(reconros_executor->pExecutorsHw > 0)
    {
        Zycap_Init(&reconros_executor->Zycap);

        reconros_executor->pExecutorsHw = malloc(sizeof(t_reconros_hwexecutor) * nSlots);
        if(reconros_executor->pExecutorsHw == 0)
        {
            return -1;
        }

        for(int i = 0; i < reconros_executor->nExecutorsHw; i++)
        {
            ReconROS_HWExecutor_Init(&(reconros_executor->pExecutorsHw[i]), &reconros_executor->Zycap, i);
        }
    }


    if(reconros_executor->nExecutorsSw > 0)
    {
        reconros_executor->pExecutorsSw = malloc(sizeof(t_reconros_swexecutor) * nSlots);
        if(reconros_executor->pExecutorsSw == 0)
        {
            return -1;
        }

        for(int i = 0; i < reconros_executor->nExecutorsSw; i++)
        {
            ReconROS_SWExecutor_Init(&(reconros_executor->pExecutorsSw)[i], i);
        }
    }


    reconros_executor->alRosTmrCnt = 0;
    reconros_executor->alRosSubCnt = 0;
    reconros_executor->alRosSrvCnt = 0;
    reconros_executor->alRosCltCnt = 0;

    //Object lists
    reconros_executor->alRosTmr = malloc(sizeof(t_callback_list_element) * RECONROS_EXECUTOR_MAX_NR_OF_OBJECTS);
    reconros_executor->alRosSub = malloc(sizeof(t_callback_list_element) * RECONROS_EXECUTOR_MAX_NR_OF_OBJECTS);
    reconros_executor->alRosSrv = malloc(sizeof(t_callback_list_element) * RECONROS_EXECUTOR_MAX_NR_OF_OBJECTS);
    reconros_executor->alRosClt = malloc(sizeof(t_callback_list_element) * RECONROS_EXECUTOR_MAX_NR_OF_OBJECTS);

    if((reconros_executor->alRosTmr == 0) || (reconros_executor->alRosSub == 0) || (reconros_executor->alRosSrv == 0) || (reconros_executor->alRosClt == 0))
        return -1;

    return 0;
}


int ReconROS_Executor_Spin(t_reconros_executor * reconros_executor)
{

    for(int i = 0; i < reconros_executor->nExecutorsHw; i++)
    {
        ReconROS_HWExecutor_Spin(&(reconros_executor->pExecutorsHw)[i]);
    }

    for(int i = 0; i < reconros_executor->nExecutorsSw; i++)
    {
        ReconROS_SWExecutor_Spin(&(reconros_executor->pExecutorsSw)[i]);
    }

    return 0;

}


int ReconROS_Executor_Add_Callback(t_reconros_executor * reconros_executor, t_callback * callback)
{
    return 1;
}


int ReconROS_Executor_GetNextTimer(t_reconros_executor * reconros_executor)
{
    return 1;
}

int ReconROS_Executor_GetNextSubscriber(t_reconros_executor * reconros_executor, t_callback_list_element ** element)
{
    for(int i = 0; i < reconros_executor->alRosSubCnt; i++)
    {
        if(pthread_mutex_trylock(&(reconros_executor->alRosSub[i].object_lock)) == 0) // check if resource is free
        {
            
            if(ros_subscriber_message_try_take((struct t_ros_subscriber*)(reconros_executor->alRosSub[i].pReconROSPrimitive), reconros_executor->alRosSub[i].pReconROSMsgPrimitive ) == 0) // if no 
            {
                //we found a subscriber with a new message;
                *element = &reconros_executor->alRosSub[i];
                return 0;
            }
            else
            {
                pthread_mutex_unlock(&(reconros_executor->alRosSub->object_lock));            
            }

        }
        else 
        {
            continue;
        }

    }

    return 1;
}

int ReconROS_Executor_ReleaseSubscriber(t_reconros_executor * reconros_executor, t_callback_list_element ** element)
{
    pthread_mutex_unlock(&(*element)->object_lock);   
    return 1;
}

int ReconROS_Executor_GetNextService(t_reconros_executor * reconros_executor)
{
    return 1;
}

int ReconROS_Executor_GetNextClient(t_reconros_executor * reconros_executor)
{
    return 1;
}



int ReconROS_GetHWCallback(t_reconros_executor * reconros_executor, uint32_t SlotMask, t_bitstream ** pBitstream, void ** ppMessage)
{
    
}