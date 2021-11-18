#include <stdint.h>
#include <pthread.h>

typedef struct
{
    uint32_t    bRun;
    pthread_t   ptAgent;
}t_reconros_swexecutor;

void    ReconROS_SWExecutor_Init(t_reconros_swexecutor * reconros_swexecutor, int id);

int     ReconROS_SWExecutor_Spin(t_reconros_swexecutor * reconros_swexecutor);


