/*
 * pwmin.c
 *
 *  Created on: Feb 19, 2024
 *      Author: misspapaya
 */

#include <stdint.h>

#include "gpio_wrapper.h"
#include "sched.h"

#define NUM_PWMS 4

struct pwmin_s
{
	int pin;
	int32_t start_time;
	int32_t width;
};

extern TIM_HandleTypeDef htim1;

typedef struct pwmin_s pwmin;

static int next_handle;

pwmin pwms[NUM_PWMS];

void pwmin_handler(int pin)
{
	int pinvalue;
	int i;
	for(i=0;i<next_handle;i++)
	{
		if(pwms[i].pin == pin)
		{
			break;
		}
	}
	if(i == next_handle)
	{
		while(1);
	}
	pinvalue = gpio_read(pin);
	if(pinvalue == 1)
	{
		pwms[i].start_time = htim1.Instance->CNT;
	}
	else
	{
		pwms[i].width = htim1.Instance->CNT - pwms[i].start_time;
	}
}

int pwmin_init(int pin)
{
	int handle;
	if(next_handle == NUM_PWMS)
	{
		return -1;
	}
	handle = next_handle++;
	pwms[handle].pin = pin;
	pwms[handle].width = 0;
	gpio_interrupt(pin,pwmin_handler);
	return handle;
}

int32_t pwmin_width(int handle)
{
	return pwms[handle].width;
}
