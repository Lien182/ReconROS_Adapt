#include <stdint.h>
#include <zycap_linux.h>


#include "hw_executor.h"
#include "sw_executor.h"

enum ReconROS_primitive{ ReconROS_TMR = 0, ReconROS_SUB = 1, ReconROS_SRV = 2, ReconROS_CLT = 3 };


typedef struct
{
    t_reconros_hwexecutor * pExecutorsHw;
    uint32_t                nExecutorsHw;
    t_reconros_swexecutor * pExecutorsSw;
    uint32_t                nExecutorsSw;

    t_zycap                 Zycap;    
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