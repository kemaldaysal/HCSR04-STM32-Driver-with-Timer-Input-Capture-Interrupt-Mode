/*
 * uart_driver.h
 *
 *  Created on: Dec 29, 2023
 *      Author: Kemal
 */

#ifndef INC_UART_DRIVER_H_
#define INC_UART_DRIVER_H_

#include "stm32f0xx_hal.h"
#include "stm32f070xb.h"
#include <string.h>
#include <stdint.h>
#include "stdio.h"

#define BUFFER_SIZE 256

typedef struct UART_Buffer_Type {
	uint32_t buffer[BUFFER_SIZE];
	uint32_t head_pointer;
	uint32_t tail_pointer;
} UART_Buffer_t;

/**
  * @brief UART Initialization Function:
  * 	   Initializes UART with the following settings:
  * 	   Word Length = 8 Bits
      	   Stop Bit = One Stop bit
      	   Parity = None
           BaudRate = 115200 baud
           Hardware flow control disabled (RTS and CTS signals)

  * @param void
  * @retval void
  */
//HAL_StatusTypeDef UART_init(int64_t baudrate);
HAL_StatusTypeDef UART_init(uint32_t baudrate);

void UART_send_byte(uint8_t data);

void UART_send_byte_array(char *buffer, uint32_t size);

void UART_send_string(const char *str);

int32_t UART_read_byte(void);

uint32_t UART_bytes_left_to_read(void);

#endif /* INC_UART_DRIVER_H_ */
