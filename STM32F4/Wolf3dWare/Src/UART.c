/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include <stdint.h>
#include <stdbool.h>

/* Definition for USARTx clock resources */
#define USARTx                           USART1
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __HAL_RCC_USART1_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART1_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_6
#define USARTx_TX_GPIO_PORT              GPIOB
#define USARTx_TX_AF                     GPIO_AF7_USART1
#define USARTx_RX_PIN                    GPIO_PIN_7
#define USARTx_RX_GPIO_PORT              GPIOB
#define USARTx_RX_AF                     GPIO_AF7_USART1

static UART_HandleTypeDef UartHandle;

#define __debugbreak()  { __asm volatile ("bkpt #0"); }

static void Error_Handler(void)
{
	__debugbreak();
	while(1) {
	}
}

void uart_tx(const void *buf, uint32_t len)
{
	// running concurrently with receive loop so can be busy
	while(HAL_UART_Transmit(&UartHandle, (uint8_t *)buf, len, HAL_MAX_DELAY) == HAL_BUSY) ;
}

bool uart_rx(uint8_t *ch)
{
	if(HAL_UART_Receive(&UartHandle, (uint8_t *)ch, 1, 0) == HAL_OK)
		return true;

	return false;
}

void uart_init()
{
	/*##-1- Configure the UART peripheral ######################################*/
	/* Put the USART peripheral in the Asynchronous mode (UART Mode) */
	/* UART1 configured as follow:
		- Word Length = 8 Bits
		- Stop Bit = One Stop bit
		- Parity = None
		- BaudRate = 9600 baud
		- Hardware flow control disabled (RTS and CTS signals) */
	UartHandle.Instance          = USARTx;
	UartHandle.Init.BaudRate     = 115200;
	UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits     = UART_STOPBITS_1;
	UartHandle.Init.Parity       = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode         = UART_MODE_TX_RX;
	UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

	if(HAL_UART_Init(&UartHandle) != HAL_OK) {
		Error_Handler();
	}

	uart_tx("Hello\r\n", 7);
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
	uart_tx("Error\r\n", 7);
}

/**
  * @brief UART MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	/*##-1- Enable peripherals and GPIO Clocks #################################*/
	/* Enable GPIO TX/RX clock */
	USARTx_TX_GPIO_CLK_ENABLE();
	USARTx_RX_GPIO_CLK_ENABLE();
	/* Enable USART2 clock */
	USARTx_CLK_ENABLE();

	/*##-2- Configure peripheral GPIO ##########################################*/
	/* UART TX GPIO pin configuration  */
	GPIO_InitStruct.Pin       = USARTx_TX_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = USARTx_TX_AF;

	HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

	/* UART RX GPIO pin configuration  */
	GPIO_InitStruct.Pin = USARTx_RX_PIN;
	GPIO_InitStruct.Alternate = USARTx_RX_AF;

	HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief UART MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO configuration to their default state
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
	/*##-1- Reset peripherals ##################################################*/
	USARTx_FORCE_RESET();
	USARTx_RELEASE_RESET();

	/*##-2- Disable peripherals and GPIO Clocks #################################*/
	/* Configure UART Tx as alternate function  */
	HAL_GPIO_DeInit(USARTx_TX_GPIO_PORT, USARTx_TX_PIN);
	/* Configure UART Rx as alternate function  */
	HAL_GPIO_DeInit(USARTx_RX_GPIO_PORT, USARTx_RX_PIN);
}
