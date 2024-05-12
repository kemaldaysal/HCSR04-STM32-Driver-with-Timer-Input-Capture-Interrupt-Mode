/*
 * application_layer.c
 *
 *  Created on: May 10, 2024
 *      Author: Kemal
 */

#include "application_layer_driver.h"

void main_timer_and_uart_init_function(void)
{
	basic_timer_init(TIMER_PRESCALER, TIMER_ARR);
	IC_timer_init();
	UART_init(UART_BAUDRATE);
}


void enable_and_start_timers(void)
{
	basic_timer_enable();
	IC_timer_start();
}




