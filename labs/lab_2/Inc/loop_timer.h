/*
 * timer.h
 *
 *  Created on: Feb 21, 2021
 *      Author: Kylie Outlaw
 */

#ifndef LOOP_TIMER_H_
#define LOOP_TIMER_H_


#include <float.h>
#include <inttypes.h>
#include "main.h"
#include "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"


/****************************************************************************
**	Types
****************************************************************************/
enum {
	INNER_LOOP,
	OUTER_LOOP,
	BLINK_LOOP,
	ADC_LOOP,
	UART_RX_LOOP,
	UART_TX_LOOP,
	NUM_LOOPS
} Loop_Location_type ;


struct loop_timer {
	/* Qualitative */
	uint8_t location;
	char * Name;

	/* variables added to */
	uint16_t total_loops;
	uint32_t total_time;

	/* variables updated each loop */
	uint32_t start_tick;
	uint32_t end_tick;
	uint8_t loop_counting;
};


/****************************************************************************
**	Procedures
****************************************************************************/
void begin_loop_timer( struct loop_timer * LT );
void stop_loop_timer( struct loop_timer * LT );
void reset_loop_timer( struct loop_timer LT );
void print_loop_timer( struct loop_timer * LT );
void init_loop( struct loop_timer * LT );

#endif /* LOOP_TIMER_H_ */
