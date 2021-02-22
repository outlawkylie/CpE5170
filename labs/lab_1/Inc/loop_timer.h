/*
 * timer.h
 *
 *  Created on: Feb 21, 2021
 *      Author: Kylie Outlaw
 */

#ifndef LOOP_TIMER_H_
#define LOOP_TIMER_H_

#include <inttypes.h>
#include "main.h"
#include "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"
#include "../Drivers/RTS_libs/FEAT_Scheduler/sch_basic_pub.h"

/****************************************************************************
**	Types
****************************************************************************/
struct loop_timer {

	/* variables added to */
	uint8_t total_loops;
	uint32_t total_time;

	/* variables updated each loop */
	long long int start_tick;
	long long int end_tick;
	uint8_t loop_counting;
};


/****************************************************************************
**	Procedures
****************************************************************************/
void begin_loop_timer( struct loop_timer * LT );
void stop_loop_timer( struct loop_timer * LT );
void reset_loop_timer( struct loop_timer LT );
void print_loop_timer( struct loop_timer * LT );

#endif /* LOOP_TIMER_H_ */
