#ifndef SWEXECUTOR_H
#define SWEXECUTOR_H

#include <stdint.h>
#include <pthread.h>

typedef struct
{
    uint32_t                bRun;
    pthread_t               ptAgent;
    int                     id;
}t_reconros_swexecutor;




void    ReconROS_SWExecutor_Init(t_reconros_swexecutor * reconros_swexecutor, int id);

int     ReconROS_SWExecutor_Spin(t_reconros_swexecutor * reconros_swexecutor);

int     ReconROS_SWExecutor_Join(t_reconros_swexecutor * reconros_swexecutor);

#endif