/*
 * tasks_lab_1.c
 *
 *  Created on: Jan 31, 2019
 *      Author: Maciej
 */
#include "tasks_lab_1.h"
#include "loop_timer.h"
#include "../Drivers/RTS_libs/FEAT_Scheduler/sch_basic_pub.h"

#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_uart.h"
#include <string.h>

extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart2;



char str_LED_BLINK_1[]="LED1";
char str_LED_BLINK_2[]="LED2";
char str_RTR_CH_SWITCH[] = "RTR_CH_SWITCH_MMCR";

char str_TEST[] = "TEST 1\n";

struct loop_timer BLT = {.location = BLINK_LOOP, .loop_counting = 0xFF};
extern struct loop_timer ADC_LT;
extern struct loop_timer RX_LT;
extern struct loop_timer TX_LT;

#define LED_G_PORT GPIOA
#define LED_G_PIN	GPIO_PIN_5

void read_adc_loop();
void check_uart_loop();
void check_input_loop();

uint8_t adc_loop_id;
uint8_t uart_loop_id;
uint8_t input_loop_id;


void lab1_power_up()
{
	adc_loop_id = SCH_NO_TIMEOUT_ID;
	uart_loop_id = SCH_NO_TIMEOUT_ID;
	input_loop_id = SCH_NO_TIMEOUT_ID;

}

void lab1_init()
{
	timer_cb_test1(NULL);

	adc_loop_id = sch_add_loop(read_adc_loop);
	//uart_loop_id = sch_add_loop(check_uart_loop);
	//input_loop_id = sch_add_loop(check_input_loop);

	init_loop( &BLT );
	init_loop( &ADC_LT );
	init_loop( &RX_LT );
	init_loop( &TX_LT );
}



void timer_cb_test1(uint8_t *x)
{
	//timing analysis
	stop_loop_timer( &BLT );
	reset_loop_timer( BLT );

	//functionality
	HAL_GPIO_WritePin(LED_G_PORT,LED_G_PIN, 1); //Toggle LED
	sch_create_timeout(rtc_get_ticks()+1000, timer_cb_test2, 0, str_LED_BLINK_2);
}
void timer_cb_test2(uint8_t *x)
{
	//timing analysis
	BLT.start_tick = rtc_get_ticks();

	//functionality
	HAL_GPIO_WritePin(LED_G_PORT, LED_G_PIN, 0); //Toggle LED
	sch_create_timeout(rtc_get_ticks()+1000, timer_cb_test1, 0, str_LED_BLINK_1);
}
///////////////////////////////////////////////


//HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart
//							, uint8_t *pData, uint16_t Size, uint32_t Timeout);
//HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);


/*void check_input_loop()
{

	begin_loop_timer( &TX_LT );
	if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))
	{
		if (HAL_OK != HAL_UART_Transmit(&huart2, msg1, strlen((char*)msg1), 5) )
			printf("Debug error while UART Tx");
		// 5 ticks ~= 5ms
	}
	stop_loop_timer( &TX_LT );
}*/

/*void check_uart_loop()
{
	uint8_t buf[10];
	uint16_t buf_len = 1; // reading one char at a time
	begin_loop_timer( &RX_LT );
	if (HAL_OK == HAL_UART_Receive(&huart2, buf, buf_len, 0))
	{
		// receive successful a byte
		if ((buf[0]=='t')||(buf[0] == 'T'))
		{
			// Start temperature reading from ADC
			HAL_ADC_Start(&hadc1);
		}
		if ((buf[0]=='h')||(buf[0]=='H')||(buf[0] == '?'))
		{
			// Start temperature reading from ADC
			HAL_UART_Transmit(&huart2, msg_help, strlen((char*)msg_help),10);
		}
	}
	stop_loop_timer( &RX_LT );
}*/


void read_adc_loop()
{
	begin_loop_timer( &ADC_LT );
	if (HAL_OK == HAL_ADC_PollForConversion(&hadc1, 0))
	{
		// ADC ready
		char temp_str[15];
		uint32_t temp = HAL_ADC_GetValue(&hadc1);
		sprintf(temp_str, "T=%d", (int)temp);
		HAL_UART_Transmit(&huart2, (uint8_t*)temp_str, strlen(temp_str),5);
	}
	stop_loop_timer( &ADC_LT );
}
