#ifndef CALLBACKLIST_H
#define CALLBACKLIST_H

#include <stdint.h>
#include <pthread.h>
#include "zycap_linux.h"
#include "helpers.h"

typedef struct 
{
    uint32_t                    callback_id;
    pthread_mutex_t             object_lock;
    void *                      pReconROSPrimitive;
    enum ReconROS_primitive     eReconROSPrimitive;
    void *                      pReconROSResultPrimitive;

    struct reconos_thread*      pHWthread;
    uint32_t                    nSlotMask;
    t_bitstream *               bitstreams;

    struct reconos_thread*      pSWthread;
    function_ptr                pSWCallback;
    void *                      args;
}t_callback_list_element;

typedef struct 
{
    t_callback_list_element *   alRosTmr;
    uint32_t                    alRosTmrCnt;
    t_callback_list_element *   alRosSub;
    uint32_t                    alRosSubCnt;
    t_callback_list_element *   alRosSrv;
    uint32_t                    alRosSrvCnt;
    t_callback_list_element *   alRosClt;
    uint32_t                    alRosCltCnt;
}t_callback_lists;



int Callbacklist_GetHWCallback(t_callback_lists * callback_lists, uint32_t nSlotMask, t_bitstream ** pBitstream, void ** ppMessage);

int Callbacklist_GetSWCallback(t_callback_lists * callbackliists, function_ptr * pCallback, void ** ppMessage);


#endif