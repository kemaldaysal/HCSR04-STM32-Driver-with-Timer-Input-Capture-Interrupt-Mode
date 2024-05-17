/*
 * timer_driver.h
 *
 *  Created on: Dec 19, 2023
 *      Author: Kemal
 */

#ifndef INC_TIMER_DRIVER_H_
#define INC_TIMER_DRIVER_H_

#include "stm32f0xx_hal.h"
#include "stm32f070xb.h"
#include <stdint.h>
#include "string.h"
#include "stdio.h"

#include "uart_driver.h"

#define MEASURING_FREQ_IN_US ((uint16_t) 60000) // max is 65536 (2^16) for now

#define TRIG_PIN GPIO_PIN_9
#define TRIG_PORT GPIOA
#define ECHO_PIN GPIO_PIN_8
#define ECHO_PORT GPIOA

#define SPEED_OF_SOUND_BASIC_IN_CM_US (0.0344487f) // in 22.7 *C

typedef enum{
	MICROSECONDS,
	MILLISECONDS,
}timer_range_ms_or_us_e;

void IC_timer_init(void);
void IC_timer_start(void);

void set_measurement_period_in_ms(uint16_t period_in_ms);

void basic_timer_basic_while_delay(uint16_t delay);
void basic_timer_reinit_with_new_settings(timer_range_ms_or_us_e timer_range, uint16_t stoptime);
void basic_timer_reset_counter(void);
void basic_timer_init(timer_range_ms_or_us_e timer_range, uint16_t arr);
void basic_timer_enable(void);
void basic_timer_disable(void);
void basic_timer_enable_interrupt(void);
void basic_timer_disable_interrupt(void);
uint16_t timer_get_counter_value(void);

void send_sensor_data_to_uart(void);

#endif /* INC_TIMER_DRIVER_H_ */
