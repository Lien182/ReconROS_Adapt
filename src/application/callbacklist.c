#include "callbacklist.h"

#include "ros_timer.h"
#include "ros_sub.h"
#include "ros_service_server.h"
#include "ros_service_client.h"


int Callbacklist_Init(t_callback_lists * callback_lists)
{
    callback_lists->alRosTmrCnt = 0;
    callback_lists->alRosSubCnt = 0;
    callback_lists->alRosSrvCnt = 0;
    callback_lists->alRosCltCnt = 0;

    //Object lists
    callback_lists->alRosTmr = malloc(sizeof(t_callback_list_element) * RECONROS_EXECUTOR_MAX_NR_OF_OBJECTS);
    callback_lists->alRosSub = malloc(sizeof(t_callback_list_element) * RECONROS_EXECUTOR_MAX_NR_OF_OBJECTS);
    callback_lists->alRosSrv = malloc(sizeof(t_callback_list_element) * RECONROS_EXECUTOR_MAX_NR_OF_OBJECTS);
    callback_lists->alRosClt = malloc(sizeof(t_callback_list_element) * RECONROS_EXECUTOR_MAX_NR_OF_OBJECTS);

    if((callback_lists->alRosTmr == 0) || (callback_lists->alRosSub == 0) || (callback_lists->alRosSrv == 0) || (callback_lists->alRosClt == 0))
        return -1;

    return 0;
}

int Callbacklist_GetNextTimer(t_callback_lists * callback_lists, t_callback_list_element ** element, uint32_t u32SlotMask) // if slotmask is 0, return sw
{
    return 0;
}

int Callbacklist_ReleaseTimer(t_callback_lists * callback_lists, t_callback_list_element ** element)
{
    pthread_mutex_unlock(&(*element)->object_lock);   
    return 1;
}

int Callbacklist_GetNextSubscriber(t_callback_lists * callback_lists, t_callback_list_element ** element, uint32_t u32SlotMask) // if slotmask is 0, return sw
{
    for(int i = 0; i < callback_lists->alRosSubCnt; i++)
    {
        if((u32SlotMask == 0 ) || (callback_lists->alRosSub[i].nSlotMask & u32SlotMask))
        {
            if(pthread_mutex_trylock(&(callback_lists->alRosSub[i].object_lock)) == 0) // check if resource is free
            {
                
                if(ros_subscriber_message_try_take((struct ros_subscriber_t*)(callback_lists->alRosSub[i].pReconROSPrimitive), callback_lists->alRosSub[i].pReconROSResultPrimitive ) == 0) // if no 
                {
                    //we found a subscriber with a new message;
                    *element = &callback_lists->alRosSub[i];
                    return i;
                }
                else
                {
                    pthread_mutex_unlock(&(callback_lists->alRosSub[i].object_lock));            
                }

            }
            else 
            {
                continue;
            }
        }
       

    }

    return 0;
}

int Callbacklist_ReleaseSubscriber(t_callback_lists * callback_lists, t_callback_list_element ** element, uint32_t u32SlotMask)
{
    pthread_mutex_unlock(&(*element)->object_lock);   
    return 1;
}

int Callbacklist_GetNextService(t_callback_lists * callback_lists, t_callback_list_element ** element, uint32_t u32SlotMask) // if slotmask is 0, return sw
{
    for(int i = 0; i < callback_lists->alRosSrvCnt; i++)
    {
        if((u32SlotMask == 0 ) || (callback_lists->alRosSrv[i].nSlotMask & u32SlotMask))
        {
            if(pthread_mutex_trylock(&(callback_lists->alRosSrv[i].object_lock)) == 0) // check if resource is free
            {
                
                if(ros_service_server_request_try_take((struct ros_service_server_t*)(callback_lists->alRosSrv[i].pReconROSPrimitive), callback_lists->alRosSrv[i].pReconROSResultPrimitive ) == 0) // if no 
                {
                    //we found a subscriber with a new message;
                    *element = &callback_lists->alRosSrv[i];
                    return i;
                }
                else
                {
                    pthread_mutex_unlock(&(callback_lists->alRosSrv[i].object_lock));            
                }

            }
            else 
            {
                continue;
            }
        }
       

    }

    return 0;
}

int Callbacklist_ReleaseService(t_callback_lists * callback_lists, t_callback_list_element ** element, uint32_t u32SlotMask)
{
    pthread_mutex_unlock(&(*element)->object_lock);   
    return 1;
}

int Callbacklist_GetNextClient(t_callback_lists * callback_lists, t_callback_list_element ** element, uint32_t u32SlotMask) // if slotmask is 0, return sw
{
    return 0;
}

int Callbacklist_ReleaseClient(t_callback_lists * callback_lists, t_callback_list_element ** element, uint32_t u32SlotMask)
{
    pthread_mutex_unlock(&(*element)->object_lock);   
    return 1;
}


int Callbacklist_GetHWCallback(t_callback_lists * callback_lists, uint32_t nSlotMask, t_bitstream ** pBitstream, void ** ppMessage)
{
    t_callback_list_element * element;

    if(Callbacklist_GetNextTimer(callback_lists, &element, nSlotMask))
    {
        uint32_t tmpmask = 1UL;
        t_bitstream * tmpBitstream = element->bitstreams;
        do{
            if(tmpmask & element->nSlotMask && !(tmpmask & nSlotMask) )
                tmpBitstream++;

        }while(tmpmask & nSlotMask);
        *pBitstream = tmpBitstream;
        *ppMessage = element->pReconROSResultPrimitive;

        printf("[ReconROS_GetHWCallback] Got Timer Callback");

        return element->callback_id;
    }
    else if(Callbacklist_GetNextSubscriber(callback_lists, &element, nSlotMask))
    {
        uint32_t tmpmask = 1UL;
        t_bitstream * tmpBitstream = element->bitstreams;
        do{
            if(tmpmask & element->nSlotMask && !(tmpmask & nSlotMask) )
                tmpBitstream++;

        }while(tmpmask & nSlotMask);
        *pBitstream = tmpBitstream;
        *ppMessage = element->pReconROSResultPrimitive;

        printf("[ReconROS_GetHWCallback] Got Subscriber Callback");

        return element->callback_id;
    }
    else if(Callbacklist_GetNextService(callback_lists, &element, nSlotMask))
    {
        uint32_t tmpmask = 1UL;
        t_bitstream * tmpBitstream = element->bitstreams;
        do{
            if(tmpmask & element->nSlotMask && !(tmpmask & nSlotMask) )
                tmpBitstream++;

        }while(tmpmask & nSlotMask);
        *pBitstream = tmpBitstream;
        *ppMessage = element->pReconROSResultPrimitive;

        printf("[ReconROS_GetHWCallback] Got Service Callback");

        return element->callback_id;
    }
    else if(Callbacklist_GetNextClient(callback_lists, &element, nSlotMask))
    {
        uint32_t tmpmask = 1UL;
        t_bitstream * tmpBitstream = element->bitstreams;
        do{
            if(tmpmask & element->nSlotMask && !(tmpmask & nSlotMask) )
                tmpBitstream++;

        }while(tmpmask & nSlotMask);
        *pBitstream = tmpBitstream;
        *ppMessage = element->pReconROSResultPrimitive;

        printf("[ReconROS_GetHWCallback] Got Client Callback");

        return element->callback_id;
    }

    return -1;
}


int Callbacklist_GetSWCallback(t_callback_lists * callbackliists, function_ptr * pCallback, void ** ppMessage)
{
    t_callback_list_element * element;

    if(Callbacklist_GetNextTimer(callbackliists, &element, 0))
    {
        *pCallback = element->pSWCallback;
        *ppMessage = element->pReconROSResultPrimitive;

        printf("[ReconROS_GetHWCallback] Got Timer Callback");

        return element->callback_id;
    }
    else if(Callbacklist_GetNextSubscriber(callbackliists, &element, 0))
    {
        *pCallback = element->pSWCallback;
        *ppMessage = element->pReconROSResultPrimitive;

        printf("[ReconROS_GetHWCallback] Got Subscriber Callback");

        return element->callback_id;
    }
    else if(Callbacklist_GetNextService(callbackliists, &element, 0))
    {
        *pCallback = element->pSWCallback;
        *ppMessage = element->pReconROSResultPrimitive;

        printf("[ReconROS_GetHWCallback] Got Service Callback");

        return element->callback_id;
    }
    else if(Callbacklist_GetNextClient(callbackliists, &element, 0))
    {
        *pCallback = element->pSWCallback;
        *ppMessage = element->pReconROSResultPrimitive;

        printf("[ReconROS_GetHWCallback] Got Client Callback");

        return element->callback_id;
    }

    return -1;
}
