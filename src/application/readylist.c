#include "readylist.h"

#include "ros_timer.h"
#include "ros_sub.h"
#include "ros_service_client.h"
#include "ros_service_server.h"


//static pthread_mutex_t __mutexPR = PTHREAD_MUTEX_INITIALIZER;;



int select_next_timer        (struct ros_timer_t * ros_timer, uint32_t nElements, t_readylistelement * readylistelement)
{

    for(int i = 0; i < nElements;i++)
    {

    }

}

int select_next_subscriber   (struct ros_subscriber_t * ros_subscriber, uint32_t nElements, t_readylistelement * readylistelement)
{

    for(int i = 0; i < nElements;i++)
    {
        if(ros_subscriber_message_try_take(&(ros_subscriber[i]), readylistelement->message) == 1)
            return 0;
    }

    return -1;

}

int select_next_service      (struct ros_service_server_t * ros_service, uint32_t nElements, t_readylistelement * readylistelement)
{
    
    for(int i = 0; i < nElements;i++)
    {
        
    }

}

int select_next_client       (struct ros_service_client_t * ros_client, uint32_t nElements, t_readylistelement * readylistelement)
{
    for(int i = 0; i < nElements;i++)
    {
        
    }
}

/*
t_readylistelement update_readylist(t_readylist * ready_list)
{

    if(select_next_timer())

}
*/