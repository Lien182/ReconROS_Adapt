#include <stdint.h>

#include "zycap_linux.h"


typedef struct
{
    t_zycap *       Zycap;
    uint32_t        uSlotid;
    t_bitstream *   pBitstreams;
}t_reconros_hwexecutor;

void ReconROS_HWExecutor_Init(t_reconros_hwexecutor * t_reconros_hwexecutor, t_zycap * zycap, uint32_t slotid);


