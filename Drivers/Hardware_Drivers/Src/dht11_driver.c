#include "dht11_driver.h"
#include <timer_dht11_driver.h>

uint16_t dht11_measurement_period_in_ms = 1150; // default 1150ms

volatile int8_t dht_state = -1;

uint16_t bit_time_start = 0;
uint16_t bit_time_end = 0;
uint16_t elapsed_time = 0;

uint8_t raw_data[40];

uint8_t humidity_integer = 0;
uint8_t humidity_decimal = 0;
uint8_t temp_integer = 0;
uint8_t temp_decimal = 0;
uint8_t checksum = 0;

volatile float humidity = 0;
volatile float temperature = 0;

dht11_reply_status_e dht11_start_signal_and_check_response(void)
{

	dht11_pin_set_to_output();
	dht11_pin_set_to_low();
	timer_dht11_basic_while_delay(DURATION_START_SIGNAL_IN_US);
	dht11_pin_set_to_high();
	timer_dht11_basic_while_delay(DURATION_WAIT_FOR_FIRST_REPLY);
	dht11_pin_set_to_input();
	timer_dht11_basic_while_delay(DURATION_UNTIL_MIDDLE_OF_FIRST_REPLY); // jump to the the middle of first 80us low state reply from dht11

	if (dht11_check_if_pin_is_low()) // check if dht11 responded with 80us low signal at that time
	{

		timer_dht11_basic_while_delay(DURATION_UNTIL_MIDDLE_OF_SECOND_REPLY); // now jump to the just the middle of the second 80us high state reply from dht11

		if (dht11_check_if_pin_is_high())
		{
			// check if dht11 responded with 80us high signal at that time
			dht11_wait_until_pin_goes_low(); // wait until the start of 50us wait
			timer_dht11_reset_counter();
			return REPLY_OK;

		} else {
			return REPLY_TIMEOUT;
		}
	} else {
		return REPLY_TIMEOUT;
	}
}


dht11_read_status_e dht11_read_data(void)
{
	for (uint8_t i = 0; i <= 39; i++)
	{

		dht11_wait_until_pin_goes_high(); // wait until the 50us wait ends
		timer_dht11_reset_counter();
		bit_time_start = timer_dht11_get_counter_value();
		dht11_wait_until_pin_goes_low(); // wait until the high bit representing real data ends
		bit_time_end = timer_dht11_get_counter_value();

		elapsed_time = bit_time_end - bit_time_start;

		if ((20 <= elapsed_time) && (elapsed_time <= 28))
		{
			raw_data[i] = 0;
		} else if ((65 <= elapsed_time) && (elapsed_time <= 75)) {
			raw_data[i] = 1;
		} else {
			return READ_ERROR_UNEXPECTED_BIT;
		}
	}
	return READ_OK;
}

dht11_checksum_status_e dht11_checksum(void)
{

	uint8_t sum = 0;
    for (uint8_t i = 0; i < 32; i += 8)
    {
        uint8_t byte = 0;
        for (uint8_t j = 0; j < 8; j++) {
            byte |= (raw_data[i + j] << (7 - j));
        }
        sum += byte;
    }

    uint8_t checksum = 0;

    for (uint8_t i = 32; i<40; i++)
    {
    	checksum |= (raw_data[i] << (39-i));
    }

	if (sum == checksum) {
		return CHECKSUM_OK;
	} else {
		return CHECKSUM_FAILED;
	}
}

void dht11_interpret_data(void)
{
	humidity_integer = 0;
	humidity_decimal = 0;
	temp_integer = 0;
	temp_decimal = 0;
	humidity = 0;
	temperature = 0;

	for (uint8_t j = 0; j<8; j++)
	{
		humidity_integer |= ((raw_data[j]) << (7-j));
		humidity_decimal |= ((raw_data[j+8]) << (7-j));
		temp_integer |= ((raw_data[j+16]) << (7-j));
		temp_decimal |= ((raw_data[j+24]) << (7-j));
	}

	humidity = humidity_integer + (humidity_decimal * 0.1f);
	temperature = temp_integer + (temp_decimal * 0.1f);
}

void dht11_and_timer_start(uint16_t dht11_measure_period_in_ms)
{
	dht11_pin_config();
	dht11_set_measurement_period_in_ms(dht11_measure_period_in_ms);
	timer_dht11_init(MILLISECONDS, 1300); // To pass the sensor's unstable status, do not send any instruction within 1 seconds after power-up.
	dht_state = 0;
	timer_dht11_enable();
}

void dht11_set_measurement_period_in_ms(uint16_t period_in_ms)
{
	// delay between dht11 measurements
	// IT MUST NOT SET BELOW 1150MS IN ORDER TO READ PROPERLY,
	// IF IT'S SET BELOW 1150, DHT11'S 1HZ POLLING RATE CAN'T MAINTAIN IT AND START THROWING ERRORS

	if (period_in_ms <= 1150)  {
		dht11_measurement_period_in_ms = 1150; // prevent user from setting the period below 1150 ms.
	} else {
		dht11_measurement_period_in_ms = period_in_ms;
	}
}

void dht11_wait_until_pin_goes_high(void)
{
	while (!((GPIOA->IDR) & (1 << DHT11_PIN_REG_POS_8BIT)));
}

void dht11_wait_until_pin_goes_low(void)
{
	while ((GPIOA->IDR) & (1 << DHT11_PIN_REG_POS_8BIT));
}

uint8_t dht11_check_if_pin_is_high(void) {

	if ((GPIOA->IDR) & (1 << DHT11_PIN_REG_POS_8BIT)) {
		return 1;
	} else {
		return 0;
	}
}

uint8_t dht11_check_if_pin_is_low(void) {

	if (!((GPIOA->IDR) & (1 << DHT11_PIN_REG_POS_8BIT))) {
		return 1;
	} else {
		return 0;
	}
}



void dht11_pin_config(void)
{
	// DHT11 is connected to PA10

	SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);
	/// Mode
	// start it as output mode (01) to send start signal
	// bits 21-20 must be 0-1 for output mode
	GPIOA->MODER &= ~(1<<DHT11_PIN_REG_POS_16BIT_H);
	GPIOA->MODER |= (1<<DHT11_PIN_REG_POS_16BIT_L);
	// bits 21-20 must be 0-0 for input mode

	/// Type
	// push-pull, bit 10 must be 0.
	GPIOA->OTYPER &= ~(1<<DHT11_PIN_REG_POS_8BIT);

	/// Speed
	// let's try high-speed, bit 21-20 must be 1-1.
	GPIOA->OSPEEDR |= (1<<DHT11_PIN_REG_POS_16BIT_H);
	GPIOA->OSPEEDR |= (1<<DHT11_PIN_REG_POS_16BIT_L);

	/// Pull-up pull-down
	// Because of communication between MCU and DHT11 is idle high,
	// Even a 4.7 kOhm resistor is put between VDD and output for pull-up,
	// let's also add pull-up in software just in case.
	// bits 21-20 must be 0-1 for pull-up
//	GPIOA->PUPDR &= ~(1<<DHT11_PIN_REG_POS_16BIT_H);
//	GPIOA->PUPDR |= (1<<DHT11_PIN_REG_POS_16BIT_L);

	// bits 21-20 must be 0-0 for no-pull
	GPIOA->PUPDR &= ~(1<<DHT11_PIN_REG_POS_16BIT_H);
	GPIOA->PUPDR &= ~(1<<DHT11_PIN_REG_POS_16BIT_L);

	dht11_pin_set_to_high(); // set it initially as high (because it's idle is high);

}


void dht11_pin_set_to_high(void)
{
	GPIOA->ODR |= (1<<DHT11_PIN_REG_POS_8BIT); // set it initially as high (because it's idle is high);
	GPIOA->IDR |= (1<<DHT11_PIN_REG_POS_8BIT); // set it initially as high (because it's idle is high);
}

void dht11_pin_set_to_low(void)
{
	GPIOA->ODR &= ~(1<<DHT11_PIN_REG_POS_8BIT);
	GPIOA->IDR &= ~(1<<DHT11_PIN_REG_POS_8BIT);
}

void dht11_pin_set_to_output(void)
{
	GPIOA->MODER &= ~(1<<DHT11_PIN_REG_POS_16BIT_H);
	GPIOA->MODER |= (1<<DHT11_PIN_REG_POS_16BIT_L);
}

void dht11_pin_set_to_input(void)
{
	GPIOA->MODER &= ~(1<<DHT11_PIN_REG_POS_16BIT_H);
	GPIOA->MODER &= ~(1<<DHT11_PIN_REG_POS_16BIT_L);
}
