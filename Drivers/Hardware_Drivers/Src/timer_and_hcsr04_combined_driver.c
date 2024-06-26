/*
 * timer_driver.c
 *
 *  Created on: Dec 19, 2023
 *      Author: Kemal
 */

#include <timer_and_hcsr04_combined_driver.h>

uint16_t hcsr04_measurement_period_in_ms = 500; // default 500ms

TIM_HandleTypeDef htim1;

uint8_t last_rising_edge_captured = 0;
uint8_t timer_trig_mode = 0;
static uint8_t first_unwanted_interrupt_bypassed = 0;

extern volatile float temperature;

double difference = 0;
double distance = 0;

static void MX_TIM1_Init(void);
extern void Error_Handler(void);

// This is for echo pin, this callback function is called whenever a rising or a falling edge is captured
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {

//		First, capture a rising edge and start the timer
		if (last_rising_edge_captured == 0) {
//			Set the capture mode to detect and capture falling edge just after catching the rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
			TIM1->CNT = 0; // reset the counter start the counting process until a falling edge is detected
			last_rising_edge_captured = 1;

		}

		else if (last_rising_edge_captured == 1) { // now it's time for capturing the falling edge

			difference = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // store the time spent between resetting the5
			// Based on this difference, the distance will be calculated by using the formula below, which is provided in the sensor's datasheet.
			// NOTE: This formula must/will be improved because the speed of sound gets changed by ambient temperature and humidity.
			// A temperature sensor will be involved and this code will be updated later. Therefore, the formula below can be used to get more accurate results.

			if (temperature != 0) {
				distance = (difference/2)*((331*(sqrt(1+(temperature/273))))*0.0001);
			} else {
				distance = (difference/2)*((331*(sqrt(1+(22.7/273))))*0.0001);
			}

//			distance = (difference/2)*SPEED_OF_SOUND_BASIC_IN_CM_US; // sound of speed in 22.7*C: 345 m/s -> 0.035 cm/us

			last_rising_edge_captured = 0; // Set it to the default state to capture the rising edge on the next cycle

			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING); // enable advanced timer's (TIM1) input capture interrupt and wait for next echo loop
			__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);

		}

	}

}


void TIM7_IRQHandler(void) {

	TIM7->SR &= ~(TIM_SR_UIF); // reset the interrupt flag for basic timer

	if (first_unwanted_interrupt_bypassed == 0){ // Pass the first count because it generates an unwanted interrupt.
		first_unwanted_interrupt_bypassed = 1;

	} else if (first_unwanted_interrupt_bypassed == 1) {

		if (timer_trig_mode == 0) {

			// START TRIG MODE
			timer_trig_mode = 1;

			HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
			basic_timer_reinit_with_new_settings(MICROSECONDS, 100);
//			basic_timer_set_delay_time_in_us(100); // send soundwaves for 100 us
//			basic_timer_reset();

		}

		else if (timer_trig_mode == 1) {

			// unpower the trig pin after sending signal for 10 ms.
			HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
			basic_timer_reinit_with_new_settings(MILLISECONDS, hcsr04_measurement_period_in_ms);
			// START ECHO MODE
//			basic_timer_set_delay_time_in_us(MEASURING_FREQ_IN_US); // wait for the echo and repeat the reading processes in each x us.
//			basic_timer_reset();
			timer_trig_mode = 0;
			send_sensor_data_to_uart(); // send the data while waiting for the next loop

			__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1); // enable advanced timer's (TIM1) input capture interrupt

		}
	}
}

void IC_timer_start(void) {

	HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_1);

}

void IC_timer_init(void) {

	MX_TIM1_Init();

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 48-1; // we need to work in us range because hcsr04 sends signals in few us's, therefore we have 48.000.000 / 48 = 1.000.000 Hz, 1/1Mhz = 1us timer period
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535; // use the highest arr value, since we won't use it.
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

void basic_timer_reinit_with_new_settings(timer_range_ms_or_us_e timer_range, uint16_t stoptime)
{

	TIM7->CR1 &= ~(1<<0); // disable the timer temporarily

	if (timer_range == MILLISECONDS) {

		TIM7->PSC = 48000-1;

	} else if (timer_range == MICROSECONDS) {

		TIM7->PSC = 48-1;
	}

	TIM7->ARR = (stoptime-1);

	TIM7->EGR |= (1<<0); // restart the timer with new settings
	TIM7->SR &= ~(1 << 0);

	TIM7->CR1 |= (1<<0); // enable the timer again
	first_unwanted_interrupt_bypassed = 0;

}

void set_measurement_period_in_ms(uint16_t period_in_ms)
{

//	if (period_in_ms <= 1150)  {
//		dht11_measurement_period_in_ms = 1150; // prevent user from setting the period below 1150 ms.
//	} else {
		hcsr04_measurement_period_in_ms = period_in_ms;
//	}

}

void basic_timer_reset_counter(void) {
	TIM7->CNT = 0;

}

uint16_t basic_timer_get_counter_value(void)
{
	return (TIM7->CNT);
}

void basic_timer_init(timer_range_ms_or_us_e timer_range, uint16_t arr)
{

	// We'll use this timer to generate a 1000 ms delay.

//	__HAL_RCC_TIM7_CLK_ENABLE();
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM7EN);
	//TIM6->CR1 |= (1<<2); // commented on 291223 1623, test it later

	// Example: The clock source is 48 MHz.
	// 48.000.000 / 48000 = 1000 Hz -> 1 / 1kHz = 1 ms timer period

	if (timer_range == MICROSECONDS)
	{
		TIM7->PSC = 48-1;

	} else if (timer_range == MILLISECONDS)
	{
		TIM7->PSC = 48000-1;

	}

	// Example: We had a 1ms timer period, let it count 200 times, which is equal to 200 ms.
	// and generate a event caused by arr's limit.

	TIM7->ARR = (arr)-1;

	basic_timer_enable_interrupt();

}

void basic_timer_enable(void)
{
	TIM7->CR1 |= (1<<0);
	first_unwanted_interrupt_bypassed = 0;
}

void basic_timer_disable(void)
{

	TIM7->CR1 &= ~(1<<0);
//	first_unwanted_interrupt_bypassed = 0;

}

void basic_timer_enable_interrupt(void)
{
	TIM7->DIER |= (1<<0);
	NVIC_EnableIRQ(TIM7_IRQn);
	NVIC_SetPriority(TIM7_IRQn, 1);
}

void basic_timer_disable_interrupt(void)
{
	TIM7->DIER &= ~(1<<0);
	NVIC_DisableIRQ(TIM7_IRQn);
}

void send_sensor_data_to_uart(void)
{
	char uart_buffer[80];
	if (temperature != 0) {
		sprintf((char*) uart_buffer, "Distance: %.2f cm, temperature: %.2f *C\r\n", distance, temperature);
	} else {
		sprintf((char*) uart_buffer, "Distance: %.2f cm, temperature read error!! (default 22.7 *C is used)\r\n", distance);
	}

	UART_send_byte_array(uart_buffer, strlen((char*) uart_buffer));
}

