/*
 * timer.c
 *
 *  Created on: Feb 21, 2021
 *      Author: Kylie Outlaw
 */
#include <loop_timer.h>

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim2;

void begin_loop_timer( struct loop_timer * LT )
	{
	reset_loop_timer( *LT );
	LT->start_tick = __HAL_TIM_GET_COUNTER(&htim2);
	return;
	} /* begin_loop_timer() */


void stop_loop_timer( struct loop_timer * LT )
	{
	LT->end_tick = __HAL_TIM_GET_COUNTER(&htim2);

	if( LT->start_tick < LT->end_tick )
	{
	uint32_t elapsed = ((LT->end_tick) - (LT->start_tick));
	LT->total_loops += 1;
	LT->total_time += (elapsed);
	}
	reset_loop_timer( *LT );
	return;
	} /* stop_loop_timer() */


void reset_loop_timer( struct loop_timer LT )
	{
	LT.start_tick = 0;
	LT.end_tick = 0;

	return;
	} /* reset_loop_timer() */

void print_loop_timer( struct loop_timer * LT )
	{
	if( LT->total_loops == 255 && LT->loop_counting )
		{
		LT->loop_counting = 0x00;
		char temp_str[50];
		sprintf(temp_str, "Loop has been run %i times.\n", (LT->total_loops));
		HAL_UART_Transmit(&huart2, (uint8_t*)temp_str, strlen((char*)temp_str),10);
		sprintf(temp_str, "Loop had an average runtime of %li ticks.\n", (LT->total_time)/(LT->total_loops));
		HAL_UART_Transmit(&huart2, (uint8_t*)temp_str, strlen((char*)temp_str),10);

		LT->total_time = 1;
		}

	}
