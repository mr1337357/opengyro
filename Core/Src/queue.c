/*
 * queue.c
 *
 *  Created on: Feb 18, 2024
 *      Author: misspapaya
 */

#include "queue.h"

#include "cmsis_gcc.h"

struct queue_s
{
	uint16_t start;
	uint16_t end;
	uint16_t size;
	void *buffer;
};

void queue_init(queue *q,void *buff, uint16_t size)
{
	while(size&(size-1))
	{
		size = size & (size-1);
	}
	q->buffer = buff;
	q->size = size;
	q->start = 0;
	q->end = 1;
}

int queue_add(queue *q,void *value,uint16_t size)
{
	uint32_t primask;
	primask = __get_PRIMASK();
	__disable_irq();



	__set_PRIMASK(primask);
	return 1;
}
