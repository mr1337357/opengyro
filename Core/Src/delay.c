/*
 * delay_mem.c
 *
 *  Created on: Feb 21, 2024
 *      Author: dc-tb
 */

#include "stdint.h"
#include "sched.h"

#define DELAY_SIZE 16

typedef struct
{
	int milis_left;
	task t;
} delay_item;

delay_item delay_mem[DELAY_SIZE];

void delay_init()
{
	int i;
	for(i=0;i<DELAY_SIZE;i++)
	{
		delay_mem[i].milis_left = -1;
	}
}

void delay_update(int time)
{
	int i;
	for(i=0;i<DELAY_SIZE;i++)
	{
		if(delay_mem[i].milis_left <= time && delay_mem[i].milis_left > 0)
		{
			delay_mem[i].milis_left = -1;
			sched_add_task(&delay_mem[i].t);
		}
		else if(delay_mem[i].milis_left > 0)
		{
			delay_mem[i].milis_left -= time;
		}
	}
}

void delay(int time, task *t)
{
	int i;
	for(i=0;i<DELAY_SIZE;i++)
	{
		if(delay_mem[i].milis_left == -1)
		{
			delay_mem[i].milis_left = time;
			delay_mem[i].t = *t;
			break;
		}
	}
}
