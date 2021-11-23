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
    //printf("[Callbacklist_GetNextTimer] debug 0 \n");
    return 0;
}

int Callbacklist_ReleaseTimer(t_callback_lists * callback_lists, t_callback_list_element * element)
{
    pthread_mutex_unlock(&element->object_lock);   
    return 1;
}

int Callbacklist_GetNextSubscriber(t_callback_lists * callback_lists, t_callback_list_element ** element, uint32_t u32SlotMask) // if slotmask is 0, return sw
{
    //printf("[Callbacklist_GetNextSubscriber] debug -1, %x \n", callback_lists);
    for(int i = 0; i < callback_lists->alRosSubCnt; i++)
    {
        //printf("[Callbacklist_GetNextSubscriber] debug 0 \n");
        if((u32SlotMask == 0 ) || (callback_lists->alRosSub[i].nSlotMask & u32SlotMask))
        {
            //printf("[Callbacklist_GetNextSubscriber] debug 1 \n");
            if(pthread_mutex_trylock(&(callback_lists->alRosSub[i].object_lock)) == 0) // check if resource is free
            {
                //printf("[Callbacklist_GetNextSubscriber] debug 2 \n");
                if(ros_subscriber_message_try_take((struct ros_subscriber_t*)(callback_lists->alRosSub[i].pReconROSPrimitive), callback_lists->alRosSub[i].pReconROSResultPrimitive ) == 0) // if no 
                {
                    //printf("[Callbacklist_GetNextSubscriber] debug 3 \n");
                    //we found a subscriber with a new message;
                    *element = &callback_lists->alRosSub[i];
                    return 1;
                }
                else
                {
                    //printf("[Callbacklist_GetNextSubscriber] debug 4 \n");
                    pthread_mutex_unlock(&(callback_lists->alRosSub[i].object_lock));            
                }

            }
            else 
            {
                continue;
            }
        }
       

    }
    //printf("[Callbacklist_GetNextSubscriber] debug 5 \n");
    return 0;
}

int Callbacklist_ReleaseSubscriber(t_callback_lists * callback_lists, t_callback_list_element * element)
{
    pthread_mutex_unlock(&element->object_lock);   
    return 1;
}

int Callbacklist_GetNextService(t_callback_lists * callback_lists, t_callback_list_element ** element, uint32_t u32SlotMask) // if slotmask is 0, return sw
{
    for(int i = 0; i < callback_lists->alRosSrvCnt; i++)
    {
        //printf("[Callbacklist_GetNextService] debug 1 \n");
        if((u32SlotMask == 0 ) || (callback_lists->alRosSrv[i].nSlotMask & u32SlotMask))
        {
            //printf("[Callbacklist_GetNextService] debug 2 \n");
            if(pthread_mutex_trylock(&(callback_lists->alRosSrv[i].object_lock)) == 0) // check if resource is free
            {
                //printf("[Callbacklist_GetNextService] debug 3 \n");
                if(ros_service_server_request_try_take((struct ros_service_server_t*)(callback_lists->alRosSrv[i].pReconROSPrimitive), callback_lists->alRosSrv[i].pReconROSResultPrimitive ) == 0) // if no 
                {
                    //printf("[Callbacklist_GetNextService] debug 4 \n");
                    //we found a subscriber with a new message;
                    *element = &callback_lists->alRosSrv[i];
                    return 1;
                }
                else
                {
                    //printf("[Callbacklist_GetNextService] debug 5 \n");
                    pthread_mutex_unlock(&(callback_lists->alRosSrv[i].object_lock));            
                }

            }
            else 
            {
                continue;
            }
        }
       

    }
    //printf("[Callbacklist_GetNextService] debug 6 \n");
    return 0;
}

int Callbacklist_ReleaseService(t_callback_lists * callback_lists, t_callback_list_element * element)
{
    pthread_mutex_unlock(&element->object_lock);   
    return 1;
}

int Callbacklist_GetNextClient(t_callback_lists * callback_lists, t_callback_list_element ** element, uint32_t u32SlotMask) // if slotmask is 0, return sw
{
    return 0;
}

int Callbacklist_ReleaseClient(t_callback_lists * callback_lists, t_callback_list_element * element)
{
    pthread_mutex_unlock(&element->object_lock);   
    return 1;
}


int Callbacklist_GetHWCallback(t_callback_lists * callback_lists, uint32_t nSlotMask, t_bitstream ** pBitstream, void ** ppMessage)
{
    t_callback_list_element * element;
     printf("[Callbacklist_GetHWCallback] check for new events \n");

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

        printf("[Callbacklist_GetHWCallback] Got Timer Callback");

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

        printf("[Callbacklist_GetHWCallback] Got Subscriber Callback");

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

        printf("[Callbacklist_GetHWCallback] Got Service Callback");

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

        printf("[Callbacklist_GetHWCallback] Got Client Callback");

        return element->callback_id;
    }

    printf("[Callbacklist_GetHWCallback] No events found\n");
    return -1;
}


int Callbacklist_GetSWCallback(t_callback_lists * callbacklists, function_ptr * pCallback, void ** ppMessage)
{
    t_callback_list_element * element;
    //printf("[Callbacklist_GetSWCallback] check for new events \n");

    if(Callbacklist_GetNextTimer(callbacklists, &element, 0))
    {
        printf("[Callbacklist_GetSWCallback] Got Timer Callback \n");
        *pCallback = element->pSWCallback;
        *ppMessage = element->pReconROSResultPrimitive;

        return element->callback_id;
    }
    else if(Callbacklist_GetNextSubscriber(callbacklists, &element, 0))
    {
        printf("[Callbacklist_GetSWCallback] Got Subscriber Callback \n");
        *pCallback = element->pSWCallback;
        *ppMessage = element->pReconROSResultPrimitive;

        return element->callback_id;
    }
    else if(Callbacklist_GetNextService(callbacklists, &element, 0))
    {
        printf("[Callbacklist_GetSWCallback] Got Service Callback \n");
        *pCallback = element->pSWCallback;
        *ppMessage = element->pReconROSResultPrimitive;

        return element->callback_id;
    }
    else if(Callbacklist_GetNextClient(callbacklists, &element, 0))
    {
        printf("[Callbacklist_GetSWCallback] Got Client Callback \n");
        *pCallback = element->pSWCallback;
        *ppMessage = element->pReconROSResultPrimitive;

        return element->callback_id;
    }

    //printf("[Callbacklist_GetSWCallback] No events found\n");

    return -1;
}



int Callbacklist_Release(t_callback_lists * callbackliists, int id)
{
    printf("[Callbacklist_Release] release element\n");

    for(int i = 0; i < callbackliists->alRosTmrCnt; i++)
    {
        if(callbackliists->alRosTmr[i].callback_id == id)
        {
            Callbacklist_ReleaseTimer(callbackliists, &callbackliists->alRosTmr[i]);
            return 1;
        }
    }

    for(int i = 0; i < callbackliists->alRosSubCnt; i++)
    {
        if(callbackliists->alRosSub[i].callback_id == id)
        {
            Callbacklist_ReleaseTimer(callbackliists, &callbackliists->alRosSub[i]);
            return 1;
        }
    }

    for(int i = 0; i < callbackliists->alRosSrvCnt; i++)
    {
        if(callbackliists->alRosSrv[i].callback_id == id)
        {
            Callbacklist_ReleaseTimer(callbackliists, &callbackliists->alRosSrv[i]);
            return 1;
        }
    }

    for(int i = 0; i < callbackliists->alRosCltCnt; i++)
    {
        if(callbackliists->alRosClt[i].callback_id == id)
        {
            Callbacklist_ReleaseTimer(callbackliists, &callbackliists->alRosClt[i]);
            return 1;
        }
    }


    return 0;
}