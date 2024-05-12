/*
 * application_layer.h
 *
 *  Created on: May 10, 2024
 *      Author: Kemal
 */

#ifndef INC_APPLICATION_LAYER_DRIVER_H_
#define INC_APPLICATION_LAYER_DRIVER_H_

#include <timer_and_hcsr04_combined_driver.h>
#include "uart_driver.h"

#define TIMER_PRESCALER ((uint16_t) 48)
#define TIMER_ARR ((uint16_t) 65536)
#define UART_BAUDRATE ((uint32_t) 115200)

void main_timer_and_uart_init_function(void);
void enable_and_start_timers(void);

#endif /* INC_APPLICATION_LAYER_DRIVER_H_ */
