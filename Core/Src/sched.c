/*
 * sched.c
 *
 *  Created on: Feb 19, 2024
 *      Author: misspapaya
 */

#include <stdint.h>

#include "cmsis_gcc.h"

#include "sched.h"

#define QUEUE_SIZE 16

struct sched_t
{
	uint16_t start;
	uint16_t end;
	task tasks[QUEUE_SIZE];
};

struct sched_t sched;

void sched_init()
{
	sched.start = 0;
	sched.end = 1;
}

int sched_add_task(task *t)
{
	if(sched.end == sched.start)
	{
		return 0;
	}
	sched.tasks[sched.end].arg = t->arg;
	sched.tasks[sched.end].func = t->func;
	sched.end = (sched.end + 1) & (QUEUE_SIZE-1);
	return 1;
}

int sched_get_task(task *t)
{
	uint32_t primask;
	if(sched.start+1 == sched.end)
	{
		return 0;
	}
	primask = __get_PRIMASK();
	__disable_irq();

	sched.start = (sched.start+1) & (QUEUE_SIZE - 1);
	t->arg = sched.tasks[sched.start].arg;
	t->func = sched.tasks[sched.start].func;

	__set_PRIMASK(primask);
	return 1;
}
