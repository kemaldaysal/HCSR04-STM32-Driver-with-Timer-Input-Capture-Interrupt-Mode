/*
 * application_layer.c
 *
 *  Created on: May 10, 2024
 *      Author: Kemal
 */

#include "application_layer_driver.h"

void init_and_start_distance_measuring(uint16_t distance_measurement_period_in_ms, uint16_t temp_measurement_period_in_ms, uint32_t uart_baudrate)
{
	basic_timer_init(TIMER_PRESCALER_STARTUP, TIMER_PERIOD_STARTUP);
	set_measurement_period_in_ms(distance_measurement_period_in_ms);
	IC_timer_init();
	UART_init(uart_baudrate);

	dht11_and_timer_start(temp_measurement_period_in_ms);

	basic_timer_enable();
	IC_timer_start();
}




