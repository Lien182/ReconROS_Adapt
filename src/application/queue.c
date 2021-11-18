/********************************************************************          
* queue.c           -- queue structure for the reconf server        *
*                                                                   *
*                                                                   *  
* Author(s):  Christian Lienen                                      *   
*                                                                   *   
********************************************************************/

#include "queue.h"
#include <stdio.h>
#include <sys/mman.h>
#include <limits.h>
#include <string.h>
#include <pthread.h>

int queue_init(t_queue * queue, uint32_t size, uint32_t element_size)
{
        queue->buf = (uint32_t*)malloc(size * sizeof(element_size));
        if(queue->buf == NULL)
                return -1;
        queue->head = 0;
        queue->tail = 0;
        queue->size = size;
        queue->element_size = size;
        memset(queue->buf, 0, queue->size * sizeof(element_size));

        sem_init(&(queue->sem_read),0 , 0);
        sem_init(&(queue->sem_write), 0, size);
        pthread_mutex_init(&queue->queue_mutex, NULL);

        return 0;
}


void queue_enqueue(t_queue * queue, void* newval)
{
        sem_wait(&(queue->sem_write));        
        pthread_mutex_lock(&queue->queue_mutex);


        memcpy(&queue->buf[queue->tail*queue->element_size], newval, queue->element_size);
        if(queue->tail == queue->size-1)
                queue->tail = 0;
        else
                queue->tail++;        

        pthread_mutex_unlock(&queue->queue_mutex);
        sem_post(&(queue->sem_read));
}

uint32_t queue_dequeue(t_queue * queue, void * dest)
{
        sem_wait(&(queue->sem_read));
        pthread_mutex_lock(&queue->queue_mutex);
        
        memcpy(dest, queue->buf[queue->head*queue->element_size], queue->element_size);
        if(queue->head == queue->size-1)
                queue->head = 0;
        else
                queue->head++;
        
        pthread_mutex_unlock(&queue->queue_mutex);
        sem_post(&(queue->sem_write));
        return 0;

}

uint32_t queue_isempty(t_queue * queue)
{
        if(queue->head == queue->tail)
                return 1;
        else
                return 0;        
}