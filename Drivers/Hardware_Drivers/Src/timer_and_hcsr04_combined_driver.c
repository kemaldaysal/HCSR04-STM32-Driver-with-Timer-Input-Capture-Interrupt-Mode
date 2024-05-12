/*
 * timer_driver.c
 *
 *  Created on: Dec 19, 2023
 *      Author: Kemal
 */

#include <timer_and_hcsr04_combined_driver.h>

TIM_HandleTypeDef htim1;

uint8_t last_rising_edge_captured = 0;
uint8_t timer_trig_mode = 0;
uint8_t first_basic_timer_interrupt_bypassed = 0;

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
			//			distance = (difference/2)*((331*(sqrt(1+(temp/273))))*0.0001);

			distance = (difference/2)*SPEED_OF_SOUND_BASIC_IN_CM_US; // sound of speed in 22.7*C: 345 m/s -> 0.035 cm/us

			last_rising_edge_captured = 0; // Set it to the default state to capture the rising edge on the next cycle

			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING); // enable advanced timer's (TIM1) input capture interrupt and wait for next echo loop
			__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);

		}

	}

}


void TIM7_IRQHandler(void) {

	TIM7->SR &= ~(TIM_SR_UIF); // reset the interrupt flag for basic timer

	if (first_basic_timer_interrupt_bypassed == 0){ // Pass the first count because it generates an unwanted interrupt.
		first_basic_timer_interrupt_bypassed = 1;

	} else if (first_basic_timer_interrupt_bypassed == 1) {

		if (timer_trig_mode == 0) {

			// START TRIG MODE
			timer_trig_mode = 1;

			HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
			basic_timer_set_delay_time_in_us(100); // send soundwaves for 100 us
			basic_timer_reset();

		}

		else if (timer_trig_mode == 1) {

			// unpower the trig pin after sending signal for 10 ms.
			HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

			// START ECHO MODE
			basic_timer_set_delay_time_in_us(60000); // wait for the echo and repeat the reading processes in each x us.
			basic_timer_reset();
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


void basic_timer_init(uint16_t prescaler, uint16_t arr) {

	// We'll use this timer to generate a 1000 ms delay.

	__HAL_RCC_TIM7_CLK_ENABLE();

	//TIM6->CR1 |= (1<<2); // commented on 291223 1623, test it later

	// The clock source is 48 MHz.
	// 48.000.000 / 48000 = 1000 Hz -> 1 / 1kHz = 1 ms timer period

	TIM7->PSC = prescaler-1; // default: 48000-1

	// We had a 1ms timer period, let it count 200 times, which is equal to 200 ms.
	// and generate a event caused by arr's limit.

	TIM7->ARR = arr-1; // default: 200-1

	// Update interrupt (UIE) enable, in DIER register, bit 0 must be 1

	//TIM7->DIER |= (1<<0);
	TIM7->DIER |= TIM_DIER_UIE;

	NVIC_EnableIRQ(TIM7_IRQn);
	NVIC_SetPriority(TIM7_IRQn, 2);

}

void basic_timer_set_delay_time_in_us(uint16_t time) {
	TIM7->ARR = time-1;
}

void basic_timer_reset(void) {
	TIM7->CNT = 0;

}


void basic_timer_enable(void) {
	// In order to enable the timer, bit 0 of CR1 register must be 1

	TIM7->CR1 |= (1<<0);

}

void basic_timer_disable(void) {

	// In order to disable the timer, bit 0 of CR1 register must be 0

	TIM7->CR1 &= ~(1<<0);
}


void send_sensor_data_to_uart(void) {
	char uart_buffer[30];
	sprintf((char*) uart_buffer, "Distance: %.2f cm\r\n", distance);
	UART_send_byte_array(uart_buffer, strlen((char*) uart_buffer));
}




