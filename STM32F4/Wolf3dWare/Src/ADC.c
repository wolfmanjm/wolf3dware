#include "stm32f4xx_hal.h"

#define ADCx                            ADC3
#define ADCx_CLK_ENABLE()               __HAL_RCC_ADC3_CLK_ENABLE()
#define DMAx_CLK_ENABLE()               __HAL_RCC_DMA2_CLK_ENABLE()
#define ADCx_CHANNEL_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOF_CLK_ENABLE()

#define ADCx_FORCE_RESET()              __HAL_RCC_ADC_FORCE_RESET()
#define ADCx_RELEASE_RESET()            __HAL_RCC_ADC_RELEASE_RESET()

/* Definition for ADCx Channel Pin */
#define ADCx_CHANNEL_PIN                GPIO_PIN_6
#define ADCx_CHANNEL_GPIO_PORT          GPIOF

/* Definition for ADCx's Channel */
#define ADCx_CHANNEL                    ADC_CHANNEL_4

/* Definition for ADCx's DMA */
#define ADCx_DMA_CHANNEL                DMA_CHANNEL_2
#define ADCx_DMA_STREAM                 DMA2_Stream1

/* Definition for ADCx's NVIC */
#define ADCx_DMA_IRQn                   DMA2_Stream1_IRQn
#define ADCx_DMA_IRQHandler             DMA2_Stream1_IRQHandler


static ADC_HandleTypeDef    AdcHandle;

/* Variable used to get converted value */
uint16_t uhADCxConvertedValue[8];

#define __debugbreak()  { __asm volatile ("bkpt #0"); }
static void Error_Handler(void)
{
	__debugbreak();
	while(1) {
	}
}

uint16_t* getADC(uint8_t ch)
{
	switch(ch) {
		case 0: return uhADCxConvertedValue;
		default: return 0;
	}
}

void InitializeADC()
{
	ADC_ChannelConfTypeDef sConfig;


	/*##-1- Configure the ADC peripheral #######################################*/
	AdcHandle.Instance          = ADCx;

	AdcHandle.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4; // set to 21MHz, APB2 is 84MHz / 4
	AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;
	AdcHandle.Init.ScanConvMode = DISABLE;
	AdcHandle.Init.ContinuousConvMode = ENABLE;
	AdcHandle.Init.DiscontinuousConvMode = DISABLE;
	AdcHandle.Init.NbrOfDiscConversion = 0;
	AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	AdcHandle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	AdcHandle.Init.NbrOfConversion = 1;
	AdcHandle.Init.DMAContinuousRequests = ENABLE;
	AdcHandle.Init.EOCSelection = DISABLE;

	if(HAL_ADC_Init(&AdcHandle) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

	/*##-2- Configure ADC regular channel ######################################*/
	sConfig.Channel = ADCx_CHANNEL;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES; //  144 is 7.4us/sample, 480 is 23.4us/sample
	sConfig.Offset = 0;

	if(HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK) {
		/* Channel Configuration Error */
		Error_Handler();
	}

	/*##-3- Start the conversion process and enable interrupt ##################*/
	if(HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)&uhADCxConvertedValue[0], 8) != HAL_OK) {
		/* Start Conversation Error */
		Error_Handler();
	}

}

/**
  * @brief ADC MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
	GPIO_InitTypeDef          GPIO_InitStruct;
	static DMA_HandleTypeDef  hdma_adc;

	/*##-1- Enable peripherals and GPIO Clocks #################################*/
	/* Enable GPIO clock */
	ADCx_CHANNEL_GPIO_CLK_ENABLE();
	/* ADC3 Periph clock enable */
	ADCx_CLK_ENABLE();
	/* Enable DMA2 clock */
	DMAx_CLK_ENABLE();

	/*##-2- Configure peripheral GPIO ##########################################*/
	/* ADC3 Channel8 GPIO pin configuration */
	GPIO_InitStruct.Pin = ADCx_CHANNEL_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ADCx_CHANNEL_GPIO_PORT, &GPIO_InitStruct);

	/*##-3- Configure the DMA streams ##########################################*/
	/* Set the parameters to be configured */
	hdma_adc.Instance = ADCx_DMA_STREAM;

	hdma_adc.Init.Channel  = ADCx_DMA_CHANNEL;
	hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_adc.Init.MemInc = DMA_MINC_ENABLE;
	hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	hdma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	hdma_adc.Init.Mode = DMA_CIRCULAR;
	hdma_adc.Init.Priority = DMA_PRIORITY_MEDIUM;
	hdma_adc.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	hdma_adc.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
	hdma_adc.Init.MemBurst = DMA_MBURST_SINGLE;
	hdma_adc.Init.PeriphBurst = DMA_PBURST_SINGLE;

	HAL_DMA_Init(&hdma_adc);

	/* Associate the initialized DMA handle to the the ADC handle */
	__HAL_LINKDMA(hadc, DMA_Handle, hdma_adc);

	/*##-4- Configure the NVIC for DMA #########################################*/
	/* NVIC configuration for DMA transfer complete interrupt */
	HAL_NVIC_SetPriority(ADCx_DMA_IRQn, 15, 0);
	HAL_NVIC_EnableIRQ(ADCx_DMA_IRQn);
}

/**
  * @brief ADC MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO to their default state
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
	static DMA_HandleTypeDef  hdma_adc;

	/*##-1- Reset peripherals ##################################################*/
	ADCx_FORCE_RESET();
	ADCx_RELEASE_RESET();

	/*##-2- Disable peripherals and GPIO Clocks ################################*/
	/* De-initialize the ADC3 Channel8 GPIO pin */
	HAL_GPIO_DeInit(ADCx_CHANNEL_GPIO_PORT, ADCx_CHANNEL_PIN);

	/*##-3- Disable the DMA Streams ############################################*/
	/* De-Initialize the DMA Stream associate to transmission process */
	HAL_DMA_DeInit(&hdma_adc);

	/*##-4- Disable the NVIC for DMA ###########################################*/
	HAL_NVIC_DisableIRQ(ADCx_DMA_IRQn);
}

/**
* @brief  This function handles DMA interrupt request.
* @param  None
* @retval None
*/
static uint8_t time_cnt= 0;
static uint32_t adc_start= 0;
volatile uint32_t adc_ave_time= 0;
extern uint32_t start_time();
extern uint32_t stop_time();
void ADCx_DMA_IRQHandler(void)
{
	// if(time_cnt++ == 0) adc_start= start_time();
	// else if(time_cnt >= 100){
	// 	adc_ave_time= stop_time() - adc_start;
	// 	time_cnt= 0;
	// }

	HAL_DMA_IRQHandler(AdcHandle.DMA_Handle);

}

/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  AdcHandle : AdcHandle handle
  * @note   This example shows a simple way to report end of conversion, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
{
	/* Turn LED3 on: Transfer process is correct */
	//BSP_LED_On(LED3);
	// gets called every 185.5 us
	if(time_cnt++ == 0) adc_start= start_time();
	else if(time_cnt >= 100){
		adc_ave_time= stop_time() - adc_start;
		time_cnt= 0;
	}
}
/*
From http://stm32f4-discovery.com/2014/04/library-06-ad-converter-on-stm32f4xx/

CHANNEL 		ADC1	ADC2	ADC3
APB				2		2		2
ADC Channel 0	PA0		PA0		PA0
ADC Channel 1	PA1		PA1		PA1
ADC Channel 2	PA2		PA2		PA2
ADC Channel 3	PA3		PA3		PA3
ADC Channel 4	PA4		PA4		PF6
ADC Channel 5	PA5		PA5		PF7
ADC Channel 6	PA6		PA6		PF8
ADC Channel 7	PA7		PA7		PF9
ADC Channel 8	PB0		PB0		PF10
ADC Channel 9	PB1		PB1		PF3
ADC Channel 10	PC0		PC0		PC0
ADC Channel 11	PC1		PC1		PC1
ADC Channel 12	PC2		PC2		PC2
ADC Channel 13	PC3		PC3		PC3
ADC Channel 14	PC4		PC4		PF4
ADC Channel 15	PC5		PC5		PF5


Table 3. DMA2 request mapping
http://www.st.com/web/en/resource/technical/document/application_note/DM00046011.pdf
page 9


*/
