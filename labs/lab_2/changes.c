
//// Src / main.c (begining of main() function )


  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim4);

  /* USER CODE END 2 */





//// main.c -> at the end:
/* USER CODE BEGIN 4 */
uint32_t LED_delay = 0;
uint8_t LED_speedup = 0;


#define LED_G_PORT GPIOA
#define LED_G_PIN	GPIO_PIN_5
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
		if (LED_delay < 1000) LED_delay += 100; else LED_delay=1000;
		vTaskDelay(LED_delay);
		HAL_GPIO_WritePin(LED_G_PORT,LED_G_PIN, 1); //Toggle LED
		vTaskDelay(LED_delay);
		HAL_GPIO_WritePin(LED_G_PORT, LED_G_PIN, 0); //Toggle LED
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartUartTxTask */
/**
* @brief Function implementing the uartTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartTxTask */
void StartUartTxTask(void const * argument)
{
  /* USER CODE BEGIN StartUartTxTask */
	uint32_t notification = 1;
	uint8_t *pTxBuff = NULL;
  /* Infinite loop */
  for(;;)
  {
	  if (notification!=0)
	  {
		  while (pdFALSE==xQueueGenericReceive( uartTxQueueHandle, &pTxBuff, portMAX_DELAY, pdFALSE ));
		  HAL_UART_Transmit_DMA(&huart2, pTxBuff, strlen((const char*)pTxBuff));
	  }
	  notification = ulTaskNotifyTake( pdTRUE, 100 );
    osDelay(1);
  }
  /* USER CODE END StartUartTxTask */
}

/* USER CODE BEGIN Header_StartStatsTask */
/**
* @brief Function implementing the statsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStatsTask */
void StartStatsTask(void const * argument)
{
  /* USER CODE BEGIN StartStatsTask */
	UBaseType_t dstack;
	UBaseType_t t2stack;
	uint8_t ps_buffer[40*4];
	uint8_t msg_buffer[80];
	uint8_t *pTxBuff;
  /* Infinite loop */
	  for(;;)
	  {
	    //osDelay(1000);
	    vTaskDelay(1000);
	    vTaskGetRunTimeStats( (char*) ps_buffer );
	    pTxBuff = ps_buffer;
	    xQueueGenericSend(uartTxQueueHandle, (void *)&pTxBuff, 10, queueSEND_TO_BACK);
		//HAL_UART_Transmit(&huart2, msg_buffer, strlen((const char*)msg_buffer),1000);

		dstack = uxTaskGetStackHighWaterMark(defaultTaskHandle);
		t2stack = uxTaskGetStackHighWaterMark(statsTaskHandle);
		sprintf((char*)msg_buffer, "Stack High Mark: T_default=%ld, T_2=%ld\n\r", dstack, t2stack);
		pTxBuff = msg_buffer;
	    xQueueGenericSend(uartTxQueueHandle, &pTxBuff, 10, queueSEND_TO_BACK);
		//HAL_UART_Transmit(&huart2, msg_buffer, strlen((const char*)msg_buffer),1000);
	  }

  /* USER CODE END StartStatsTask */






////  Src / stm32f4xx_it.c 

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	tim4_ov_counter++;
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}


//// Src / freertos.c

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

extern TIM_HandleTypeDef htim4;
extern uint32_t tim4_ov_counter;
__weak unsigned long getRunTimeCounterValue(void)
{
	return ((tim4_ov_counter << 16) + (uint32_t)__HAL_TIM_GET_COUNTER(&htim4));
}

/* USER CODE END 1 */
