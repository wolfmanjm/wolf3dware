#include "stm32f4xx_hal.h"

#include <math.h>

#define TIMx                           TIM9
#define TIMx_CLK_ENABLE                __HAL_RCC_TIM9_CLK_ENABLE

/* Definition for USARTx Pins */
#define TIMx_CHANNEL_GPIO_PORT()       __HAL_RCC_GPIOE_CLK_ENABLE()
#define GPIO_PIN_CHANNEL1              GPIO_PIN_5
#define GPIO_PIN_CHANNEL2              GPIO_PIN_6



#define __debugbreak()  { __asm volatile ("bkpt #0"); }

static void Error_Handler(void)
{
	__debugbreak();
	while(1) {
	}
}

static TIM_HandleTypeDef TimHandle;
static TIM_OC_InitTypeDef sConfig;

void setPWM(uint8_t channel, float percent)
{
	// percentage of 1KHz
	uint32_t p= roundf(10.0F * percent); // (1000.0F * percent/100.0F) + 0.5F;
	//sConfig.Pulse = p;
	switch(channel) {
		case 0:
			//HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1);
			TIMx->CCR1 = p;
			break;

		case 1:
			//HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2);
			TIMx->CCR2 = p;
			break;
	}
}

void InitializePWM()
{
	TimHandle.Instance = TIMx;
	/* Compute the prescaler value to have TIMx counter clock equal to 1 MHz */
	TimHandle.Init.Prescaler = (uint32_t) ((SystemCoreClock) / 1000000) - 1;
	TimHandle.Init.Period = (1000 - 1);  /* Period Value PWM frequency 1Khz */
	TimHandle.Init.ClockDivision = 0;
	TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	if(HAL_TIM_PWM_Init(&TimHandle) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

	/*##-2- Configure the PWM channels #########################################*/
	/* Common configuration for all channels */
	const uint32_t PULSE1_VALUE= 0;           /* Capture Compare 1 Value  */
	const uint32_t PULSE2_VALUE= 1000;        /* Capture Compare 2 Value  */
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

