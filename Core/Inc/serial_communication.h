/*
 * serial_communication.h
 *
 *  Created on: Oct 12, 2021
 *      Author: samuel
 */

#ifndef INC_SERIAL_COMMUNICATION_H_
#define INC_SERIAL_COMMUNICATION_H_

#include "usart.h"

#define UART_RX_TIMEOUT 40
#define UART_TX_TIMEOUT UART_RX_TIMEOUT

int uart_rx_data(void *buffer, uint32_t n_bytes)
{
	HAL_StatusTypeDef ret;
	ret = HAL_UART_Receive(&huart3, (uint8_t *) buffer, n_bytes,UART_RX_TIMEOUT);

	if(ret != HAL_OK)
	{
		return -1;
	}
	return 0;
}

int uart_tx_data(void *buffer, uint32_t n_bytes)
{
	HAL_StatusTypeDef ret;
	ret = HAL_UART_Transmit(&huart3, (uint8_t *) buffer, n_bytes,UART_TX_TIMEOUT);

	if(ret != HAL_OK)
	{
		return -1;
	}
	return 0;
}

#endif /* INC_SERIAL_COMMUNICATION_H_ */
