/*
 * mz_dma_uart.c
 *
 *  Created on: Apr 20, 2020
 *      Author: macie
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os.h"
#include "main.h"


extern osThreadId uartTxTaskHandle;
extern UART_HandleTypeDef huart2;

/**
  * @brief  Tx Transfer completed callbacks.
  * @param  huart pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  //UNUSED(huart);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */
  //if (huart == huart2)
  {
	  xTaskGenericNotifyFromISR(uartTxTaskHandle, 1, eIncrement, NULL, NULL);
  }
}
