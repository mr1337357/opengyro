/*
 * sched.h
 *
 *  Created on: Feb 19, 2024
 *      Author: misspapaya
 */

#ifndef INC_SCHED_H_
#define INC_SCHED_H_

union task_arg_u
{
	void *argp;
	uint32_t argi;
};

typedef union task_arg_u task_arg;

struct task_s
{
	union task_arg_u arg;
	void (*func)(union task_arg_u);
};

typedef struct task_s task;

void sched_init();

int sched_add_task(task *t);

int sched_get_task(task *t);

#endif /* INC_SCHED_H_ */
