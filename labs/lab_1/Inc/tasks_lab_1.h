/*
 * tasks_lab_1.h
 *
 *  Created on: Jan 31, 2019
 *      Author: Maciej
 */

#ifndef TASKS_LAB_1_H_
#define TASKS_LAB_1_H_

#include <inttypes.h>
#include <stddef.h>
#include "stm32f4xx_hal.h"


extern void lab1_power_up();
extern void lab1_init();


extern void timer_cb_test1(uint8_t *x);
extern void timer_cb_test2(uint8_t *x);

extern void check_input_loop ( void );

#endif /* TASKS_LAB_1_H_ */
