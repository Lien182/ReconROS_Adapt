#include <stdint.h>
#include <zycap_linux.h>

#include "queue.h"

#include "hw_executor.h"
#include "sw_executor.h"

#define RECONROS_EXECUTOR_MAX_NR_OF_OBJECTS 10



enum ReconROS_primitive{ ReconROS_TMR = 0, ReconROS_SUB = 1, ReconROS_SRV = 2, ReconROS_CLT = 3 };


typedef struct 
{
    uint32_t                callback_id;
    pthread_mutex_t         object_lock;
    void *                  pReconROSPrimitive;
    enum ReconROS_primitive eReconROSPrimitive;

    uint32_t                nSlotMask;
    t_bitstream *           bitstreams;

    void *                  function_ptr;
    void *                  args;
}t_callback_list_element;

typedef struct 
{
    uint32_t    callback_id;
    void *      message;
}t_readylistelement;

typedef struct
{
    t_reconros_hwexecutor *     pExecutorsHw;
    uint32_t                    nExecutorsHw;
    t_reconros_swexecutor *     pExecutorsSw;
    uint32_t                    nExecutorsSw;

    t_zycap                     Zycap;    

    t_callback_list_element *  alRosTmr;
    uint32_t                   alRosTmrCnt;
    t_callback_list_element *  alRosSub;
    uint32_t                   alRosTmrSub;
    t_callback_list_element *  alRosSrv;
    uint32_t                   alRosTmrSrv;
    t_callback_list_element *  alRosClt;
    uint32_t                   alRosTmrClt;

}t_reconros_executor;



typedef struct 
{
    void *                      pReconROSPrimitive;
    enum ReconROS_primitive     eReconROSPrimitive;
    uint32_t                    uExecutionType;
    t_bitstream *               pBitstreams;
    uint32_t                    nSlotMask;
    void *                      pFunction;
    void *                      pArgs;

}t_callback;


int ReconROS_Executor_Add_Callback(t_reconros_executor * reconros_executor, t_callback * callback);