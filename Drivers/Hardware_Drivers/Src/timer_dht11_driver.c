/*
 * timer_driver.c
 *
 *  Created on: May 13, 2024
 *      Author: Kemal
 */

#include <timer_dht11_driver.h>
//#include "uart_driver.h"

//dht11_reply_status_e reply_status;
//dht11_read_status_e read_status;
//dht11_checksum_status_e checksum_status;

static uint8_t first_unwanted_interrupt_bypassed = 0;
timer_range_ms_or_us_e timer_range;
extern volatile int8_t dht_state;
extern volatile uint16_t dht11_measurement_period_in_ms;
extern volatile float humidity;
extern volatile float temperature;

//uint16_t measurements_done_count = 0;
//uint16_t timer_interrupt_triggered_count = 0;

uint16_t reply_fail_count = 0;
uint16_t read_fail_count = 0;
uint16_t checksum_fail_count = 0;

void TIM6_IRQHandler(void)
{
	TIM6->SR &= ~(1 << 0);

//	timer_interrupt_triggered_count++; // for debugging

	if (first_unwanted_interrupt_bypassed == 0) {
		first_unwanted_interrupt_bypassed = 1;
	} else {

		if (dht_state == 0)
		{
			timer_dht11_reinit_with_new_settings(MICROSECONDS, 65535);
			dht_state = 1;
//			reply_status = REPLY_NOT_REACHED; // for debugging
//			read_status = READ_NOT_REACHED; // for debugging
//			read_status = CHECKSUM_NOT_REACHED; // for debugging


			if (REPLY_OK == dht11_start_signal_and_check_response())
			{

				if (READ_OK == dht11_read_data())
				{
					timer_dht11_disable();

					if (CHECKSUM_OK == dht11_checksum())
					{

					dht11_interpret_data();
//					send_sensor_data_to_uart_when_everything_is_ok();
//					measurements_done_count++; // for debugging
					while(!((GPIOA->IDR) & (1<<DHT11_PIN_REG_POS_8BIT))); // make sure the connection is closed and dht11 is ready to send new data
					dht_state = 0; // set state to starting state
					timer_dht11_reinit_with_new_settings(MILLISECONDS, dht11_measurement_period_in_ms); // delay between dht11 measurements (IT MUSTN'T SET BELOW 1150MS IN ORDER TO READ PROPERLY,
																		// IF IT'S SET BELOW 1150, DHT11'S 1HZ POLLING RATE CAN'T CATCH IT AND START THROWING ERRORS
					} else {
						checksum_fail_count++;
//						send_sensor_data_to_uart_checksum_error();
						humidity = 0;
						temperature = 0;
						dht_state = 0;
						return;
					}

				} else {
					read_fail_count++;
//					send_sensor_data_to_uart_read_error();
					humidity = 0;
					temperature = 0;
					dht_state = 0;
					return;
				}
			} else {
				reply_fail_count++;
//				send_sensor_data_to_uart_reply_error();
				humidity = 0;
				temperature = 0;
				dht_state = 0;
				return;
			}

		}
	}
}

void timer_dht11_basic_while_delay(uint16_t delay)
{
	TIM6->CNT = 0;;
	while ((TIM6->CNT) < delay);
}

void timer_dht11_reinit_with_new_settings(timer_range_ms_or_us_e timer_range, uint16_t stoptime)
{

	TIM6->CR1 &= ~(1<<0); // disable the timer temporarily

	if (timer_range == MILLISECONDS) {

		TIM6->PSC = 48000-1;

	} else if (timer_range == MICROSECONDS) {

		TIM6->PSC = 48-1;
	}

	TIM6->ARR = (stoptime-1);

	TIM6->EGR |= (1<<0); // restart the timer with new settings
	TIM6->SR &= ~(1 << 0);

	TIM6->CR1 |= (1<<0); // enable the timer again
	first_unwanted_interrupt_bypassed = 0;

}

void timer_dht11_reset_counter(void)
{
	TIM6->CNT = 0;

}

uint16_t timer_dht11_get_counter_value(void)
{
	return (TIM6->CNT);
}

void timer_dht11_init(timer_range_ms_or_us_e timer_range, uint16_t arr)
{

	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM6EN);
//	__HAL_RCC_TIM6_CLK_ENABLE();

	if (timer_range == MICROSECONDS)
	{
		TIM6->PSC = 48-1;

	} else if (timer_range == MILLISECONDS)
	{
		TIM6->PSC = 48000-1;

	}

	TIM6->ARR = (arr)-1;

	timer_dht11_enable_interrupt();
}


void timer_dht11_enable(void)
{
	TIM6->CR1 |= (1<<0);
	first_unwanted_interrupt_bypassed = 0;
}

void timer_dht11_disable(void)
{

	TIM6->CR1 &= ~(1<<0);
//	first_unwanted_interrupt_bypassed = 0;

}

void timer_dht11_enable_interrupt(void)
{
	TIM6->DIER |= (1<<0);
	NVIC_EnableIRQ(TIM6_IRQn);
	NVIC_SetPriority(TIM6_IRQn, 1);
}

void timer_dht11_disable_interrupt(void)
{
	TIM6->DIER &= ~(1<<0);
	NVIC_DisableIRQ(TIM6_IRQn);
}

//void send_sensor_data_to_uart_when_everything_is_ok(void)
//{
//	char uart_buffer[50];
//	sprintf((char*) uart_buffer, "Temperature: %.1f *C, Relative Humidity: %.2f\r\n", temperature, humidity);
//	UART_send_byte_array(uart_buffer, strlen((char*) uart_buffer));
//}
//
//void send_sensor_data_to_uart_reply_error(void)
//{
//	char uart_buffer[20];
//	sprintf((char*) uart_buffer, "Reply error\r\n");
//	UART_send_byte_array(uart_buffer, strlen((char*) uart_buffer));
//}
//
//void send_sensor_data_to_uart_read_error(void)
//{
//	char uart_buffer[20];
//	sprintf((char*) uart_buffer, "Read error\r\n");
//	UART_send_byte_array(uart_buffer, strlen((char*) uart_buffer));
//}
//
//void send_sensor_data_to_uart_checksum_error(void)
//{
//	char uart_buffer[20];
//	sprintf((char*) uart_buffer, "Checksum error\r\n");
//	UART_send_byte_array(uart_buffer, strlen((char*) uart_buffer));
//}
