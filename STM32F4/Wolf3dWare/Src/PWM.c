#include "stm32f4xx_hal.h"

#define TIMx                           TIM9
#define TIMx_CLK_ENABLE                __HAL_RCC_TIM9_CLK_ENABLE

/* Definition for USARTx Pins */
#define TIMx_CHANNEL_GPIO_PORT()       __HAL_RCC_GPIOE_CLK_ENABLE()
#define GPIO_PIN_CHANNEL1              GPIO_PIN_5
#define GPIO_PIN_CHANNEL2              GPIO_PIN_6


#define  PERIOD_VALUE       (1000 - 1)  /* Period Value 2Khz */
#define  PULSE1_VALUE       100           /* Capture Compare 1 Value  */
#define  PULSE2_VALUE       249        /* Capture Compare 2 Value  */

TIM_HandleTypeDef    TimHandle;

/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef sConfig;

/* Counter Prescaler value */
uint32_t uhPrescalerValue = 0;

#define __debugbreak()  { __asm volatile ("bkpt #0"); }
static void Error_Handler(void)
{
	__debugbreak();
	while(1) {
	}
}

void setPWM(int channel, float percent)
{
	uint16_t p= (1000.0F * percent/100.0) + 0.5F;
	switch(channel) {
		case 1: TIMx->CCR1 = p; break;
		case 2: TIMx->CCR2 = p; break;
	}
}

void InitializePWM()
{

	/* Compute the prescaler value to have TIMx counter clock equal to 1 MHz */
	uhPrescalerValue = (uint32_t) ((SystemCoreClock / 2) / 1000000) - 1;

	/*##-1- Configure the TIM peripheral #######################################*/
	/* -----------------------------------------------------------------------
	TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles.

	  In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1),
	  since APB1 prescaler is different from 1.
		TIM3CLK = 2 * PCLK1
		PCLK1 = HCLK / 4
		=> TIM3CLK = HCLK / 2 = SystemCoreClock /2

	  To get TIM3 counter clock at 21 MHz, the prescaler is computed as follows:
		 Prescaler = (TIM3CLK / TIM3 counter clock) - 1
		 Prescaler = ((SystemCoreClock /2) /21 MHz) - 1

	  To get TIM3 output clock at 30 KHz, the period (ARR)) is computed as follows:
		 ARR = (TIM3 counter clock / TIM3 output clock) - 1
			 = 665

	  TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
	  TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%


	----------------------------------------------------------------------- */

	/* Initialize TIMx peripheral as follow:
		 + Prescaler = (SystemCoreClock/2)/21000000
		 + Period = 665
		 + ClockDivision = 0
		 + Counter direction = Up
	*/
	TimHandle.Instance = TIMx;

	TimHandle.Init.Prescaler = uhPrescalerValue;
	TimHandle.Init.Period = PERIOD_VALUE;
	TimHandle.Init.ClockDivision = 0;
	TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	if(HAL_TIM_PWM_Init(&TimHandle) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

	/*##-2- Configure the PWM channels #########################################*/
	/* Common configuration for all channels */
	sConfig.OCMode = TIM_OCMODE_PWM1;
	sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfig.OCFastMode = TIM_OCFAST_DISABLE;

	/* Set the pulse value for channel 1 */
	sConfig.Pulse = PULSE1_VALUE;
	if(HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK) {
		/* Configuration Error */
		Error_Handler();
	}

	/* Set the pulse value for channel 2 */
	sConfig.Pulse = PULSE2_VALUE;
	if(HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK) {
		/* Configuration Error */
		Error_Handler();
	}

	/*##-3- Start PWM signals generation #######################################*/
	/* Start channel 1 */
	if(HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK) {
		/* PWM Generation Error */
		Error_Handler();
	}
	/* Start channel 2 */
	if(HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK) {
		/* PWM Generation Error */
		Error_Handler();
	}
}

/**
  * @brief TIM MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef   GPIO_InitStruct;
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* TIMx Peripheral clock enable */
  TIMx_CLK_ENABLE();

  /* Enable GPIO Channels Clock */
  TIMx_CHANNEL_GPIO_PORT();

  /* Configure PE.5 (TIM9_Channel1), PE.6 (TIM9_Channel2),
     in output, push-pull, alternate function mode
  */
  /* Common configuration for all channels */
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;

  GPIO_InitStruct.Pin = GPIO_PIN_CHANNEL1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_CHANNEL2;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

