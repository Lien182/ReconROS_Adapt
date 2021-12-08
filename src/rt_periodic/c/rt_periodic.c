#include <stdio.h>
#include <sys/time.h>

#include "reconos.h"
#include "reconos_thread.h"
#include "reconos_calls.h"


THREAD_ENTRY()
{
	THREAD_INIT();

	struct timeval  tv;
	gettimeofday(&tv, NULL);

	double time_in_mill = 
         (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000 ; // convert tv_sec & tv_usec to millisecond


	printf("Periodic thread was called! %.2f \n", time_in_mill);
	
}
