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

#define TIMER_PRESCALER_STARTUP MICROSECONDS // default
#define TIMER_PERIOD_STARTUP ((uint16_t) 65536) // default max
#define UART_BAUDRATE_DEFAULT ((uint32_t) 115200) // default

void init_and_start_distance_measuring(uint16_t measurement_period_in_ms, uint32_t uart_baudrate);

#endif /* INC_APPLICATION_LAYER_DRIVER_H_ */
