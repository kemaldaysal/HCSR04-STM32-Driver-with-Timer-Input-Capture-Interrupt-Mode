/*
 * dht11_driver.h
 *
 *  Created on: May 12, 2024
 *      Author: Kemal
 */

#ifndef HARDWARE_DRIVERS_INC_DHT11_DRIVER_H_
#define HARDWARE_DRIVERS_INC_DHT11_DRIVER_H_

#include "stm32f0xx_hal.h"

#define DURATION_START_SIGNAL_IN_US ((uint16_t) 18000 ) // minimum 18 ms
#define DURATION_WAIT_FOR_FIRST_REPLY ((uint8_t) 20) // 20-40 us
#define DURATION_UNTIL_MIDDLE_OF_FIRST_REPLY ((uint8_t) 40) // 80 us reply on both high-low replies
#define DURATION_UNTIL_MIDDLE_OF_SECOND_REPLY ((uint8_t) 80) // 80 us reply on both high-low replies
//#define TIMEOUT_REPLY ((uint8_t) 100)

// PA10
#define DHT11_PORT (GPIOA)
#define DHT11_PIN (GPIO_PIN_10)
#define DHT11_PIN_REG_POS_8BIT ((uint8_t) 10 )
#define DHT11_PIN_REG_POS_16BIT_H ((uint8_t) 21 )
#define DHT11_PIN_REG_POS_16BIT_L ((uint8_t) 20 )

typedef enum{

	REPLY_TIMEOUT,
	REPLY_OK,
	REPLY_NOT_REACHED

}dht11_reply_status_e;

typedef enum{

	READ_ERROR_UNEXPECTED_BIT,
	READ_OK,
	READ_NOT_REACHED

}dht11_read_status_e;

typedef enum{

	CHECKSUM_FAILED,
	CHECKSUM_OK,
	CHECKSUM_NOT_REACHED

}dht11_checksum_status_e;

dht11_reply_status_e dht11_start_signal_and_check_response(void);
dht11_read_status_e dht11_read_data(void);
dht11_checksum_status_e dht11_checksum(void);
void dht11_interpret_data(void);

void dht11_and_timer_start(uint16_t dht11_measure_period_in_ms);
void dht11_set_measurement_period_in_ms(uint16_t period_in_ms);
void dht11_pin_config(void);

void dht11_wait_until_pin_goes_high(void);
void dht11_wait_until_pin_goes_low(void);

uint8_t dht11_check_if_pin_is_high(void);
uint8_t dht11_check_if_pin_is_low(void);

void dht11_pin_set_to_output(void);
void dht11_pin_set_to_input(void);
void dht11_pin_set_to_high(void);
void dht11_pin_set_to_low(void);

#endif /* HARDWARE_DRIVERS_INC_DHT11_DRIVER_H_ */
