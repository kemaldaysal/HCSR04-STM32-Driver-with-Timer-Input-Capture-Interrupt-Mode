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

#define TRIG_PIN GPIO_PIN_9
#define TRIG_PORT GPIOA
#define ECHO_PIN GPIO_PIN_8
#define ECHO_PORT GPIOA

#define SPEED_OF_SOUND_BASIC_IN_CM_US (0.0344487f) // in 22.7 *C

void IC_timer_init(void);
void IC_timer_start(void);

void basic_timer_init(uint16_t prescaler, uint16_t arr);
void basic_timer_set_delay_time_in_us(uint16_t time);
void basic_timer_reset(void);
void basic_timer_enable(void);
void basic_timer_disable(void);

void send_sensor_data_to_uart(void);

#endif /* INC_TIMER_DRIVER_H_ */
