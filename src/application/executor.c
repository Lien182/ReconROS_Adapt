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


int ReconROS_Executor_Add_HW_Callback(t_reconros_executor * reconros_executor, char * CallbackName, uint32_t nSlotMask, enum ReconROS_primitive primitive, void * object, void * object_result, void * resources, int resource_cnt )
{
    t_callback_list_element * pCallback;

    if(primitive == ReconROS_primitive.ReconROS_TMR)
    {
        pCallback = &(reconros_executor->alRosTmr[reconros_executor->alRosTmrCnt]);
        reconros_executor->alRosTmrCnt++;
    }
    else if(primitive == ReconROS_primitive.ReconROS_SUB)
    {
        pCallback = &(reconros_executor->alRosSub[reconros_executor->alRosSubCnt]);
        reconros_executor->alRosSubCnt++;
    }
    else if(primitive == ReconROS_primitive.ReconROS_SRV)
    {
        pCallback = &(reconros_executor->alRosSvr[reconros_executor->alRosSvrCnt]);
        reconros_executor->alRosSvrCnt++;
    }
    else (primitive == ReconROS_primitive.ReconROS_CLT)
    {
        pCallback = &(reconros_executor->alRosClt[reconros_executor->alRosCltCnt]);
        reconros_executor->alRosCltCnt++;
    }

    int nSlotCnt = 0;
    int *  slots = malloc(reconros_executor->nExecutorsHw);

    for(int i = 0; i < reconros_executor->nExecutorsHw; i++)
    {
        if(nSlotMask & (1<<i))
        {
            slots[nSlotCnt] = i
            nSlotCnt++;
        }
    }


    pCallback->bitstreams = malloc(sizeof(t_bitstreams) * nSlotCnt);
    if(!pCallback->bitstreams)
        return -1;

    char buf[255];

    for(int i = 0; i < nSlotCnt; i++)
    {
        sprintf(buf, "%s/pblock_slot_%d_%s_%d_partial.bit", reconros_executor->pBitstreamDir, slots[i], CallbackName, slots[i]);
        printf("%s", buf);
        Zycap_Prefetch_Bitstream(buf, &pCallback->bitstreams[i]);
    }

    pCallback->pReconROSPrimitive = object;
    pCallback->eReconROSPrimitive = primitive;
    pCallback->pReconROSResultPrimitive = object_result;
    pCallback->nSlotMask = nSlotMask;
    

    pCallback->callback_id = reconros_executor->nCallbackIdCnt;
    reconros_executor->nCallbackIdCnt++;
    pthread_mutex_init(&reconros_executor->object_lock, 0);




    pCallback->pHWthread = (struct reconos_thread *)malloc(sizeof(struct reconos_thread));
	if (!pCallback->pHWthread) {
		panic("[reconos-core] ERROR: failed to allocate memory for thread\n");
	}


    
	reconos_thread_init(pCallback->pHWthread, name, 0);
	reconos_thread_setinitdata(pCallback->pHWthread, 0); //later set: object ptr
	reconos_thread_setallowedslots(pCallback->pHWthread, &slot, 1);
	reconos_thread_setresourcepointers(pCallback->pHWthread, resources, resource_cnt);
	reconos_thread_create_auto(pCallback->pHWthread, RECONOS_THREAD_HW);


    return 1;
}


int ReconROS_Executor_Add_SW_Callback(t_reconros_executor * reconros_executor, char * CallbackName, uint32_t nSlotMask, enum ReconROS_primitive primitive, void * object, void * object_result, void * resources, int resource_cnt )
{
    t_callback_list_element * pCallback;

    if(primitive == ReconROS_primitive.ReconROS_TMR)
    {
        pCallback = &(reconros_executor->alRosTmr[reconros_executor->alRosTmrCnt]);
        reconros_executor->alRosTmrCnt++;
    }
    else if(primitive == ReconROS_primitive.ReconROS_SUB)
    {
        pCallback = &(reconros_executor->alRosSub[reconros_executor->alRosSubCnt]);
        reconros_executor->alRosSubCnt++;
    }
    else if(primitive == ReconROS_primitive.ReconROS_SRV)
    {
        pCallback = &(reconros_executor->alRosSvr[reconros_executor->alRosSvrCnt]);
        reconros_executor->alRosSvrCnt++;
    }
    else (primitive == ReconROS_primitive.ReconROS_CLT)
    {
        pCallback = &(reconros_executor->alRosClt[reconros_executor->alRosCltCnt]);
        reconros_executor->alRosCltCnt++;
    }

    int nSlotCnt = 0;
    int *  slots = malloc(reconros_executor->nExecutorsHw);

    for(int i = 0; i < reconros_executor->nExecutorsHw; i++)
    {
        if(nSlotMask & (1<<i))
        {
            slots[nSlotCnt] = i
            nSlotCnt++;
        }
    }


    pCallback->bitstreams = malloc(sizeof(t_bitstreams) * nSlotCnt);
    if(!pCallback->bitstreams)
        return -1;

    char buf[255];

    for(int i = 0; i < nSlotCnt; i++)
    {
        sprintf(buf, "%s/pblock_slot_%d_%s_%d_partial.bit", reconros_executor->pBitstreamDir, slots[i], CallbackName, slots[i]);
        printf("%s", buf);
        Zycap_Prefetch_Bitstream(buf, &pCallback->bitstreams[i]);
    }

    pCallback->pReconROSPrimitive = object;
    pCallback->eReconROSPrimitive = primitive;
    pCallback->pReconROSResultPrimitive = object_result;
    pCallback->nSlotMask = nSlotMask;
    

    pCallback->callback_id = reconros_executor->nCallbackIdCnt;
    reconros_executor->nCallbackIdCnt++;
    pthread_mutex_init(&reconros_executor->object_lock, 0);




    pCallback->pHWthread = (struct reconos_thread *)malloc(sizeof(struct reconos_thread));
	if (!pCallback->pHWthread) {
		panic("[reconos-core] ERROR: failed to allocate memory for thread\n");
	}


    
	reconos_thread_init(pCallback->pHWthread, name, 0);
	reconos_thread_setinitdata(pCallback->pHWthread, 0); //later set: object ptr
	reconos_thread_setallowedslots(pCallback->pHWthread, &slot, 1);
	reconos_thread_setresourcepointers(pCallback->pHWthread, resources, resource_cnt);
	reconos_thread_create_auto(pCallback->pHWthread, RECONOS_THREAD_HW);


    return 1;
}


int ReconROS_Executor_GetNextTimer(t_reconros_executor * reconros_executor)
{
    return 1;
}

int ReconROS_Executor_GetNextSubscriber(t_reconros_executor * reconros_executor, t_callback_list_element ** element, uint32_t u32SlotMask) // if slotmask is 0, return sw
{
    for(int i = 0; i < reconros_executor->alRosSubCnt; i++)
    {
        if((u32SlotMask == 0 ) || (reconros_executor->alRosSub[i].nSlotMask & u32SlotMask))
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
       

    }

    return 1;
}

int ReconROS_Executor_ReleaseSubscriber(t_reconros_executor * reconros_executor, t_callback_list_element ** element, uint32_t u32SlotMask)
{
    pthread_mutex_unlock(&(*element)->object_lock);   
    return 1;
}

int ReconROS_Executor_GetNextService(t_reconros_executor * reconros_executor)
{
    return 1;
}

int ReconROS_Executor_ReleaseService(t_reconros_executor * reconros_executor, t_callback_list_element ** element, uint32_t u32SlotMask)
{
    pthread_mutex_unlock(&(*element)->object_lock);   
    return 1;
}

int ReconROS_Executor_GetNextClient(t_reconros_executor * reconros_executor)
{
    return 1;
}

int ReconROS_Executor_ReleaseClient(t_reconros_executor * reconros_executor, t_callback_list_element ** element, uint32_t u32SlotMask)
{
    pthread_mutex_unlock(&(*element)->object_lock);   
    return 1;
}


int ReconROS_GetHWCallback(t_reconros_executor * reconros_executor, uint32_t SlotMask, t_bitstream ** pBitstream, void ** ppMessage)
{
    
}