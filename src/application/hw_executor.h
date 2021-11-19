#include <stdint.h>
#include <pthread.h>
#include "zycap_linux.h"

#include "reconos.h"


typedef struct
{
    pthread_t                   ptAgent;
    t_zycap *                   Zycap;
    uint32_t                    uSlotid;
    t_bitstream *               pBitstreams;

    uint32_t                    bRun;


}t_reconros_hwexecutor;

void ReconROS_HWExecutor_Init(t_reconros_hwexecutor * t_reconros_hwexecutor, t_zycap * zycap, uint32_t slotid);


int ReconROS_HWExecutor_Spin(t_reconros_hwexecutor * reconros_hwexecutor);