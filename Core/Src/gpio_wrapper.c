/*
 * gpio_wrapper.c
 *
 *  Created on: Feb 17, 2024
 *      Author: misspapaya
 */

#include <stdio.h>

#include "gpio_wrapper.h"

#include "sched.h"

static const gpio_wrapper wrappers[] =
{
		{GPIOB, GPIO_PIN_3 }, //LED
		{GPIOC, GPIO_PIN_13}, //GYRO_EXTI
		{GPIOA, GPIO_PIN_0 }, //STEERING
		{GPIOA, GPIO_PIN_1 }, //THROTTLE
		{GPIOB, GPIO_PIN_11}, //GAIN
		{GPIOA, GPIO_PIN_6 }, //MOTOR 1
		{GPIOA, GPIO_PIN_7 }, //MOTOR 2
		{GPIOB, GPIO_PIN_4 }, //MOTOR 3
		{GPIOB, GPIO_PIN_5 }, //MOTOR 4
		{GPIOA, GPIO_PIN_11}, //MOTOR 5
		{GPIOA, GPIO_PIN_12}, //MOTOR 6

};

#define NUM_PINS (sizeof(wrappers)/sizeof(wrappers[0]))

void (*pin_handlers[NUM_PINS])(int) =
{

};

static const int gpio2hal_map[NUM_PINS] =
{
		0,
		0, //GYRO_EXTI
		1, //STEERING
		2, //THROTTLE
		3, //GAIN
};

static const int irq_nums[NUM_PINS] =
{
		0,
		0,
		EXTI0_IRQn,
		EXTI1_IRQn,
		EXTI15_10_IRQn,
		0,
};

void test_gpio_task(task_arg pin)
{
	int val = gpio_read(pin.argi);
	printf("pin interrupt %ld %ld\n",pin.argi,val);
}

void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	int i;
//	task gpio_task = { .arg.argi = GPIO_Pin,
//					   .func = test_gpio_task};
//	sched_add_task(&gpio_task);
	for(i=0;i<NUM_PINS;i++)
	{
		if(gpio2hal_map[i] == GPIO_Pin)
		{
			break;
		}
	}
	if(pin_handlers[i] != 0)
	{
		pin_handlers[i](i);
	}
}

void gpio_init()
{
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();
	  __HAL_RCC_GPIOC_CLK_ENABLE();
}

int gpio_num_pins()
{
	return NUM_PINS;
}

int gpio_read(int pin)
{
	return HAL_GPIO_ReadPin(wrappers[pin].port, wrappers[pin].pin);
}

void gpio_write(int pin, int value)
{
	HAL_GPIO_WritePin(wrappers[pin].port, wrappers[pin].pin,!!value);
}

void gpio_dir(int pin, int value) //0 is in 1 is out
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	//HAL_GPIO_DeInit(wrappers[pin].port, wrappers[pin].pin);
	GPIO_InitStruct.Pin = wrappers[pin].pin;

	if(value == 0)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
	}
	else
	{
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	}
	HAL_GPIO_Init(wrappers[pin].port, &GPIO_InitStruct);
}

void gpio_pulldown(int pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	//HAL_GPIO_DeInit(wrappers[pin].port, wrappers[pin].pin);

	GPIO_InitStruct.Pin = wrappers[pin].pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;

	HAL_GPIO_Init(wrappers[pin].port, &GPIO_InitStruct);
}

void gpio_pullup(int pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	//HAL_GPIO_DeInit(wrappers[pin].port, wrappers[pin].pin);

	GPIO_InitStruct.Pin = wrappers[pin].pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;

	HAL_GPIO_Init(wrappers[pin].port, &GPIO_InitStruct);
}



void gpio_interrupt(int pin,void (*handler)(int))
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	//HAL_GPIO_DeInit(wrappers[pin].port, wrappers[pin].pin);

	pin_handlers[pin] = handler;

	GPIO_InitStruct.Pin = wrappers[pin].pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(wrappers[pin].port, &GPIO_InitStruct);

	if(irq_nums[pin] > 0)
	{
		HAL_NVIC_SetPriority(irq_nums[pin], 0, 0);
		HAL_NVIC_EnableIRQ(irq_nums[pin]);
	}
}
