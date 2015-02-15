/**
  ******************************************************************************
  * @file    GPIO/GPIO_EXTI/Src/main.c
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    26-December-2014
  * @brief   This example describes how to configure and use GPIOs through
  *          the STM32F4xx HAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "event_groups.h"

#include "lcd_log.h"

#include <usbd_core.h>
#include <usbd_cdc.h>
#include <usbd_cdc_if_template.h>
#include <usbd_desc.h>

#include <stdbool.h>

USBD_HandleTypeDef USBD_Device;

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_EXTI
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

osThreadId MainThreadHandle;

// reads lins from CDC
osThreadId CDCThreadHandle;
// sends lines to be parsed
osThreadId CommandHandlerThreadHandle;

osMutexDef(lcdMutex);
osMutexId lcdMutex;

#define MAXLINELEN 132
typedef struct {
	char buf[MAXLINELEN];
	uint8_t len;
} T_MEAS;

osPoolDef(mpool, 16, T_MEAS);                    // Define memory pool
osPoolId  mpool;
osMessageQDef(MsgBox, 16, T_MEAS);               // Define message queue
osMessageQId  MsgBox;
EventGroupHandle_t xEventGroup;

static void cdcThread(void const *argument);
static void commandThread(void const *argument);
static void mainThread(void const *argument);

static void SystemClock_Config(void);
static void Error_Handler(void);
static void Timer_Config(void);

TIM_HandleTypeDef PerformanceTimHandle;
TIM_HandleTypeDef StepTickerTimHandle;
volatile uint32_t delta_time= 0;

#define BUTTON_BIT 0x01

/* Private functions ---------------------------------------------------------*/

extern bool commandLineHandler(const char*);
extern void TimingTests();
extern int os_started;
extern int maincpp();
extern void issueTicks(void);

uint32_t start_time()
{
	PERFORMANCE_TIMx->CNT = 0;
	return PERFORMANCE_TIMx->CNT;
}

uint32_t stop_time()
{
	return PERFORMANCE_TIMx->CNT;
}


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	HAL_NVIC_SetPriorityGrouping( NVIC_PRIORITYGROUP_4 );

	// use straight malloc initially
	os_started = 0;

	/* STM32F4xx HAL library initialization:
		 - Configure the Flash prefetch, instruction and Data caches
		 - Configure the Systick to generate an interrupt each 1 msec
		 - Set NVIC Group Priority to 4
		 - Global MSP (MCU Support Package) initialization
	   */
	HAL_Init();

	/* Configure LED3 and LED4 */
	BSP_LED_Init(LED3);
	BSP_LED_Init(LED4);
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

	/* Configure the system clock to 180 MHz */
	SystemClock_Config();

	/*##-1- LCD Initialization #################################################*/
	/* Initialize the LCD */
	BSP_LCD_Init();

	/* Initialize the LCD Layers */
	BSP_LCD_LayerDefaultInit(1, LCD_FRAME_BUFFER);

	/* Set LCD Foreground Layer  */
	BSP_LCD_SelectLayer(1);

	BSP_LCD_SetFont(&LCD_DEFAULT_FONT);

	/* Clear the LCD */
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	BSP_LCD_Clear(LCD_COLOR_WHITE);

	/* Set the LCD Text Color */
	BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);

	LCD_LOG_Init();
	LCD_LOG_ClearTextZone();
	LCD_LOG_SetHeader((uint8_t *)"Wolf3dWare");

	// setup USB CDC
	SetupVCP();
	USBD_Init(&USBD_Device, &VCP_Desc, 0);

	USBD_RegisterClass(&USBD_Device, &USBD_CDC);
	USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_Template_fops);
	USBD_Start(&USBD_Device);

	// for (int i = 0; i < 100; ++i) {
	//  TIMx->CNT= 0;
	//  uint32_t start = TIMx->CNT;
	//  HAL_Delay(1000);
	//  uint32_t stop= TIMx->CNT;
	//      uint32_t delta= (stop - start);
	//      LCD_UsrLog("Delay 1000ms = %lu us\n", delta);
	//  }
	//  TimingTests();


	// setup FreeRTOS

	/* Attempt to create the event group. */
	xEventGroup = xEventGroupCreate();

	mpool = osPoolCreate(osPool(mpool));                 // create memory pool
	MsgBox = osMessageCreate(osMessageQ(MsgBox), NULL);  // create msg queue

	// start threads
	osThreadDef(Main, mainThread, osPriorityNormal, 0, 1000);
	MainThreadHandle = osThreadCreate (osThread(Main), NULL);

	osThreadDef(CDC, cdcThread, osPriorityNormal, 0, 1000);
	CDCThreadHandle = osThreadCreate (osThread(CDC), NULL);

	osThreadDef(Commands, commandThread, osPriorityNormal, 0, 1000);
	CommandHandlerThreadHandle = osThreadCreate (osThread(Commands), NULL);

	maincpp(); // any cpp setup needed, including assigning pins to the actuators

	Timer_Config(); // setup a 1us counter for performance tests

	// use thread safe malloc after this
	os_started = 1;


	osKernelStart();

	/* Infinite loop */
	while (1) { }
}

// called to send replies back to USB Serial
bool serial_reply(const char *buf, size_t len)
{
	int n= VCP_write(buf, len);
	return n == len;
}

extern void testGpio();
static void mainThread(void const *argument)
{
	while(1) {
		const TickType_t xTicksToWait = pdMS_TO_TICKS( 1000 );
		uint32_t ulNotifiedValue;
		BaseType_t xResult = xTaskNotifyWait( 0,          /* Don't clear bits on entry. */
                                   			  0xFFFFFFFF,  /* Clear all bits on exit. */
                                   			  &ulNotifiedValue, /* Stores the notified value. */
                                 			  xTicksToWait );

        if( xResult == pdPASS ) {
			if( ulNotifiedValue & BUTTON_BIT ) {
				BSP_LED_Toggle(LED3);
			}
			LCD_UsrLog("stepticker: %lu us\n", delta_time);
		} else {
			//BSP_LED_Toggle(LED4);
			testGpio();
		}
	}
}

// void vApplicationIdleHook( void )
// {

// }

bool dispatch(char *line, int cnt)
{
	//line[cnt]= 0;
	//LCD_UsrLog("%s\n", line);
	T_MEAS *mptr = osPoolAlloc(mpool); // Allocate memory for the message
	if(mptr == NULL) return false;

	memcpy(mptr->buf, line, cnt);
	mptr->len = cnt;
	osMessagePut(MsgBox, (uint32_t)mptr, osWaitForever);  // Send Message
	return true;
}

// reads lines from CDC serial port
// discards long lines
// dispatches line to command handler thread
void setCDCEventFromISR()
{
	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken= pdFALSE;

    vTaskNotifyGiveFromISR( CDCThreadHandle, &xHigherPriorityTaskWoken );

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

//#define MD5TEST
#ifdef MD5TEST
#include "md5.h"
static void MDPrint (MD5_CTX *mdContext)
{
  int i;
  for (i = 0; i < 16; i++){
  	LCD_UsrLog("%02x", mdContext->digest[i]);
  }
  LCD_UsrLog("\n");
}
static MD5_CTX mdContext;
static bool testing= false;
#endif

static char line[MAXLINELEN];
static int cnt = 0;

static void cdcThread(void const *argument)
{
	const TickType_t xTicksToWait = pdMS_TO_TICKS( 100 );
	uint8_t c;
	for (;;) {

		while(!VCP_get(&c)) {
			// wait until we have something to process
			ulTaskNotifyTake( pdFALSE, xTicksToWait);
		}

#ifdef MD5TEST
		// for testing download
		if(!testing && c == 26) {
			testing= true;
			MD5Init (&mdContext);
			cnt= 0;
			LCD_UsrLog("Started MD5\n");
			continue;
  		}
  		if(testing && c == 26) {
  			testing= false;
			LCD_UsrLog("Ended MD5\n");
			if(cnt > 0) {
  				MD5Update (&mdContext, line, cnt);
 				cnt= 0;
  			}
			MD5Final (&mdContext);
  			MDPrint (&mdContext);
  			continue;
  		}

		if(testing) {
			line[cnt++]= c;
			if(cnt >= sizeof(line)) {
  				MD5Update (&mdContext, line, cnt);
  				cnt= 0;
  			}
			//LCD_UsrLog("Added %c\n", c);
  			continue;
		}

#endif

		if(c == '\n') {
			if(cnt == 0) continue; //ignore empty lines

			// dispatch on NL, if out of memory wait for the other thread to catch up
			while(!dispatch(line, cnt)) {
				osDelay (100);
			}
			cnt= 0;

		}else if(c == '\r'){
			// ignore CR
			continue;

		}else if(c == 8 || c == 127) { // BS or DEL
			if(cnt > 0) --cnt;

		}else if(cnt >= sizeof(line)) {
			// discard the excess of long lines
			continue;

		}else{
			line[cnt++]= c;
		}
	}
}

static void commandThread(void const *argument)
{
	T_MEAS  *rptr;
	osEvent  evt;
	char  line[MAXLINELEN+1];
	for (;;) {
		evt = osMessageGet(MsgBox, osWaitForever);  // wait for message
		if (evt.status == osEventMessage) {
			rptr = evt.value.p;
			uint8_t cnt = rptr->len;
			memcpy(line, rptr->buf, cnt);
			osPoolFree(mpool, rptr);                  // free memory allocated for message
			line[cnt] = 0;
			//LCD_UsrLog("%s\n", line);
			commandLineHandler(line);
		}
	}
}


/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 180000000
 *            HCLK(Hz)                       = 180000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 4
 *            APB2 Prescaler                 = 2
 *            HSE Frequency(Hz)              = 8000000
 *            PLL_M                          = 8
 *            PLL_N                          = 360
 *            PLL_P                          = 2
 *            PLL_Q                          = 7
 *            VDD(V)                         = 3.3
 *            Main regulator output voltage  = Scale1 mode
 *            Flash Latency(WS)              = 5
 * @param  None
 * @retval None
 */
static void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* Enable Power Control clock */
	__HAL_RCC_PWR_CLK_ENABLE();

	/* The voltage scaling allows optimizing the power consumption when the device is
	   clocked below the maximum system frequency, to update the voltage scaling value
	   regarding system frequency refer to product datasheet.  */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 360;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/* Activate the Over-Drive mode */
	HAL_PWREx_EnableOverDrive();

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
	   clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

static void Timer_Config()
{
	// setup Performance timer

	/* Compute the prescaler value to have TIMx counter clock equal to 1000 KHz */
	uint32_t uwPrescalerValue = (uint32_t) ((SystemCoreClock / 2) / 1000000) - 1;

	/* Set TIMx instance */
	PerformanceTimHandle.Instance = PERFORMANCE_TIMx;

	/* Initialize TIM3 peripheral as follows:
		 + Period = 10000 - 1
		 + Prescaler = ((SystemCoreClock/2)/10000) - 1
		 + ClockDivision = 0
		 + Counter direction = Up
	*/
	PerformanceTimHandle.Init.Period = 0xFFFFFFFF;
	PerformanceTimHandle.Init.Prescaler = uwPrescalerValue;
	PerformanceTimHandle.Init.ClockDivision = 0;
	PerformanceTimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	if(HAL_TIM_Base_Init(&PerformanceTimHandle) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

	/*##-2- Start the TIM Base generation */
	/* Start Channel1 */
	if(HAL_TIM_Base_Start(&PerformanceTimHandle) != HAL_OK) {
		/* Starting Error */
		Error_Handler();
	}

	// setup the stepticker timer interrupt

    /* Compute the prescaler value to have TIM3 counter clock equal to 1Mhz */
    uwPrescalerValue = (uint32_t) ((SystemCoreClock / 2) / 1000000) - 1;

    /* Set TIMx instance */
    StepTickerTimHandle.Instance = STEPTICKER_TIMx;

    /* Initialize TIM3 peripheral as follows:
         + Period = xxx where 10000/xxx Hz
         + Prescaler = ((SystemCoreClock/2)/10000) - 1
         + ClockDivision = 0
         + Counter direction = Up
    */
    StepTickerTimHandle.Init.Period = 10 - 1; // set period to trigger interrupt at 10us or 100KHz
    StepTickerTimHandle.Init.Prescaler = uwPrescalerValue;
    StepTickerTimHandle.Init.ClockDivision = 0;
    StepTickerTimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;

    if(HAL_TIM_Base_Init(&StepTickerTimHandle) != HAL_OK) {
        /* Initialization Error */
        Error_Handler();
    }

    /*##-2- Start the TIM Base generation in interrupt mode ####################*/
    /* Start Channel1 */
    if(HAL_TIM_Base_Start_IT(&StepTickerTimHandle) != HAL_OK) {
        /* Starting Error */
        Error_Handler();
    }


}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == KEY_BUTTON_PIN) {
		/* Toggle LED3 */
		//BSP_LED_Toggle(LED3);
		BaseType_t xHigherPriorityTaskWoken= pdFALSE;
		xTaskNotifyFromISR( MainThreadHandle, BUTTON_BIT, eSetBits, &xHigherPriorityTaskWoken );
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// this is the step ticker
	// if(htim->Instance != STEPTICKER_TIMx) {
	// 	Error_Handler();
	// }

	// handle stepticker
	uint32_t s= start_time();

	issueTicks();

	uint32_t e= stop_time();
	delta_time= e-s;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
	__debugbreak();
	while(1) {
	}
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1) {
	}
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
