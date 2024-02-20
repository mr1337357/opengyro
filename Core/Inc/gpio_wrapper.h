/*
 * gpio_wrapper.h
 *
 *  Created on: Feb 17, 2024
 *      Author: misspapaya
 */

#ifndef INC_GPIO_WRAPPER_H_
#define INC_GPIO_WRAPPER_H_

#include <stdint.h>

#include "stm32f3xx_hal.h"

typedef struct
{
	GPIO_TypeDef *port;
	uint16_t pin;

} gpio_wrapper;

typedef enum
{
	LED_PIN,
	GYRO_EXTI,
	STEERING,
	THROTTLE,
	GAIN,
	MOTOR_1,
	MOTOR_2,
	MOTOR_3,
	MOTOR_4,
	MOTOR_5,
	MOTOR_6,
} pin_names;

void gpio_init();
int gpio_num_pins();
int gpio_read(int pin);
void gpio_write(int pin, int value);
void gpio_dir(int pin, int value); //0 is in 1 is out
void gpio_pulldown(int pin);
void gpio_pullup(int pin);
void gpio_interrupt(int pin,void (*handler)(int));

#endif /* INC_GPIO_WRAPPER_H_ */
