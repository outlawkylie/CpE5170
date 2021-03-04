/*
 * timer.c
 *
 *  Created on: Feb 21, 2021
 *      Author: Kylie Outlaw
 */
#include "loop_timer.h"

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

	if (LT->location == BLINK_LOOP) { LT->end_tick = rtc_get_ticks(); }
	else { LT->end_tick = __HAL_TIM_GET_COUNTER(&htim2); }

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
	if( LT->location != BLINK_LOOP )
		{
		if( LT->total_loops == 65535 && LT->loop_counting )
			{
			LT->loop_counting = 0x00;
			char temp_str[50];
			HAL_UART_Transmit(&huart2, (uint8_t*)temp_str, strlen((char*)temp_str),10);
			sprintf(temp_str, "%s has been run %i times.\n", LT->Name, (LT->total_loops));
			HAL_UART_Transmit(&huart2, (uint8_t*)temp_str, strlen((char*)temp_str),10);
			sprintf(temp_str, "%s had an average runtime of %.2f us.\n", LT->Name, (float)(((LT->total_time))/(LT->total_loops))/7);
			HAL_UART_Transmit(&huart2, (uint8_t*)temp_str, strlen((char*)temp_str),10);
			sprintf(temp_str, "* * * * * * * * * * * * * * * * * * * * * * *\n");
			HAL_UART_Transmit(&huart2, (uint8_t*)temp_str, strlen((char*)temp_str),10);

			LT->total_time = 1;
			}
		}
	else //LT->location == BLINK_LOOP
		{
		if( LT->total_loops == 30 && LT->loop_counting )
			{
			LT->loop_counting = 0x00;
			char temp_str[50];
			HAL_UART_Transmit(&huart2, (uint8_t*)temp_str, strlen((char*)temp_str),10);
			sprintf(temp_str, "%s has been run %i times.\n", LT->Name, (LT->total_loops));
			HAL_UART_Transmit(&huart2, (uint8_t*)temp_str, strlen((char*)temp_str),10);
			sprintf(temp_str, "%s had an average runtime of %.2f ms.\n", LT->Name, (float)((LT->total_time)/(LT->total_loops)));
			HAL_UART_Transmit(&huart2, (uint8_t*)temp_str, strlen((char*)temp_str),10);
			sprintf(temp_str, "* * * * * * * * * * * * * * * * * * * * * * *\n");
			HAL_UART_Transmit(&huart2, (uint8_t*)temp_str, strlen((char*)temp_str),10);
			}
		}
	}

void init_loop( struct loop_timer * LT )
	{
	LT->loop_counting = 0xFF;

	switch( LT-> location )
		{
		case( INNER_LOOP ):
			LT->Name = "Inner loop";
			break;

		case( OUTER_LOOP ):
			LT->Name = "Outer loop";
			break;

		case( BLINK_LOOP ):
			LT->Name = "Blink loop";
			break;

		default:
			break;
		}

	}
