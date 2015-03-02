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

#ifdef USE_STM32F429I_DISCO
#include "lcd_log.h"
#endif

#include <usbd_core.h>
#include <usbd_cdc.h>
#include <usbd_cdc_if_template.h>
#include <usbd_desc.h>

#include <stdbool.h>

// if not defined will run at 180MHz, but USB clock will be off a little bit
#define SYSCLK168MHZ

#ifdef USE_STM32F429I_DISCO
#define LCD_DISPLAY_POSITION
#endif

USBD_HandleTypeDef USBD_Device;

osThreadId MainThreadHandle;

// reads lins from CDC
osThreadId CDCThreadHandle;
// sends lines to be parsed
osThreadId CommandHandlerThreadHandle;

osThreadId moveCompletedThreadHandle;
//TaskHandle_t moveCompletedThreadHandle;

#ifdef USE_STM32F429I_DISCO
osMutexDef(lcdMutex);
osMutexId lcdMutex;
#endif

#define MAXLINELEN 132
typedef struct {
	char buf[MAXLINELEN];
	uint8_t len;
} T_MEAS;

osPoolDef(mpool, 16, T_MEAS);                    // Define memory pool
osPoolId  mpool;
osMessageQDef(MsgBox, 16, T_MEAS);               // Define message queue
osMessageQId  MsgBox;

static void cdcThread(void const *argument);
static void commandThread(void const *argument);
static void mainThread(void const *argument);

static void SystemClock_Config(void);
static void Error_Handler(void);
static void Timer_Config(void);

TIM_HandleTypeDef PerformanceTimHandle;
TIM_HandleTypeDef StepTickerTimHandle;
TIM_HandleTypeDef UnStepTickerTimHandle;

volatile uint32_t delta_time= 0;
extern volatile uint32_t adc_ave_time;
extern volatile bool running;

#define BUTTON_BIT 0x01
#define MOVE_BIT 0x02


// extern defined in maincpp mostly
extern int os_started;
extern bool commandLineHandler(const char*);
extern void TimingTests();
extern int maincpp();
extern bool issueTicks(void);
extern void moveCompletedThread(void const *argument);
extern void issueUnstep();
extern void kickQueue();

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

	/* Configure the system clock */
	SystemClock_Config();

#ifdef USE_STM32F429I_DISCO
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
	//BSP_LCD_DisplayStringAtLine(2, (uint8_t*)"This is text X");

#if USELCDLOG
	LCD_LOG_Init();
	LCD_LOG_ClearTextZone();
	LCD_LOG_SetHeader((uint8_t *)"Wolf3dWare");
	LCD_UsrLog("System clock: %1.1f MHz\n", SystemCoreClock/1000000.0F);
#endif // USELCDLOG
#endif // USE_STM32F429I_DISCO

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

	osThreadDef(Tick, moveCompletedThread, osPriorityRealtime, 0, 1000);
	moveCompletedThreadHandle = osThreadCreate (osThread(Tick), NULL);

	// xTaskCreate( moveCompletedThread, "MoveCompleted", 1000, NULL, 0, &moveCompletedThreadHandle );
	//  	configASSERT( moveCompletedThreadHandle );

	mpool = osPoolCreate(osPool(mpool));                 // create memory pool
	MsgBox = osMessageCreate(osMessageQ(MsgBox), NULL);  // create msg queue

	// start threads
	osThreadDef(Main, mainThread, osPriorityLow, 0, 1000);
	MainThreadHandle = osThreadCreate (osThread(Main), NULL);

	osThreadDef(CDC, cdcThread, osPriorityNormal, 0, 1000);
	CDCThreadHandle = osThreadCreate (osThread(CDC), NULL);

	osThreadDef(Commands, commandThread, osPriorityNormal, 0, 1000);
	CommandHandlerThreadHandle = osThreadCreate (osThread(Commands), NULL);

	maincpp(); // any cpp setup needed, including assigning pins to the actuators

	Timer_Config(); // setup a 1us counter for performance tests, and stepticker at 10us

	// use thread safe malloc after this
	os_started = 1;

	osKernelStart();

	/* Infinite loop */
	while (1) { }
}

const char * taskName= 0;
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
	// stack overflow
	taskName= pcTaskName;
	__debugbreak();
}


// called to send replies back to USB Serial
bool serial_reply(const char *buf, size_t len)
{
	int n= VCP_write(buf, len);
	return n == len;
}

extern void getPosition(float *, float *, float *, float *);
//static int lx=0, ly= 0;
extern bool host_connected;
extern bool testGpio();
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
				#ifdef USE_STM32F429I_DISCO
				//BSP_LED_Toggle(LED4);
				//LCD_UsrLog("stepticker: %lu us\n", delta_time);
				//LCD_UsrLog("\nADC time: %lu\n", adc_ave_time);
				BSP_LCD_Clear(LCD_COLOR_WHITE);
				#endif
			}

			if(ulNotifiedValue & MOVE_BIT) {
				// int x, y;
				// getPosition(&x, &y);
				// if(lx != x || ly != y){
				// 	BSP_LCD_DrawLine(lx, ly, x, y);
				// 	lx= x; ly= y;
				// }
			}

		} else {

			BSP_LED_Toggle(LED3);
			if(running) BSP_LED_On(LED4);
			else BSP_LED_Off(LED4);

			#ifdef LCD_DISPLAY_POSITION
			float x, y, z, e;
			getPosition(&x, &y, &z, &e);
			char buf[16];
			snprintf(buf, sizeof(buf), "X %8.4f", x);
			BSP_LCD_DisplayStringAtLine(2, (uint8_t*)buf);
			snprintf(buf, sizeof(buf), "Y %8.4f", y);
			BSP_LCD_DisplayStringAtLine(3, (uint8_t*)buf);
			snprintf(buf, sizeof(buf), "Z %8.4f", z);
			BSP_LCD_DisplayStringAtLine(4, (uint8_t*)buf);
			snprintf(buf, sizeof(buf), "E %8.4f", e);
			BSP_LCD_DisplayStringAtLine(5, (uint8_t*)buf);
			#endif


			//testGpio();
			// static bool last_connect_status= false;
			// if(!host_connected && last_connect_status) {
			// 	last_connect_status= false;
			// 	LCD_UsrLog("terminal diconnected\n");
			// }else if(host_connected && !last_connect_status) {
			// 	last_connect_status= true;
			// 	LCD_UsrLog("terminal connected\n");
			// }
		}
	}
}

// void vApplicationIdleHook( void )
// {

// }

// Sends a command to the command thread
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

// handles all incoming commands from the USB serial
// is the only context that is allowed to write to the USB Serial
static char  cmd_line[MAXLINELEN+1];
static void commandThread(void const *argument)
{
	const TickType_t xTicksToWait = pdMS_TO_TICKS( 100 );
	T_MEAS  *rptr;
	osEvent  evt;
	for (;;) {
		evt = osMessageGet(MsgBox, xTicksToWait);  // wait for message
		if (evt.status == osEventMessage) {
			rptr = evt.value.p;
			uint8_t cnt = rptr->len;
			memcpy(cmd_line, rptr->buf, cnt);
			osPoolFree(mpool, rptr);                  // free memory allocated for message
			cmd_line[cnt] = 0;
			//LCD_UsrLog("%s\n", cmd_line);
			commandLineHandler(cmd_line);

		}else if(evt.status == osEventTimeout) {
			kickQueue();
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
#ifndef SYSCLK168MHZ
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
	RCC_OscInitStruct.PLL.PLLM = 8;   // 5 (168)
	RCC_OscInitStruct.PLL.PLLN = 360; // 210 (168)
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/* Activate the Over-Drive mode */
	HAL_PWREx_EnableOverDrive(); // disable (168)

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

#else
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */

	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);


	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
	clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
#endif
}

static void Timer_Config()
{
	// setup Performance timer

	/* Compute the prescaler value to have TIMx counter clock equal to 1000 KHz */
	uint32_t uwPrescalerValue = (uint32_t) ((SystemCoreClock / 2) / 1000000) - 1;

	/* Set TIM2 instance */
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

	/* Set TIM3 instance */
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

	// setup the unstepticker timer interrupt

	/* Compute the prescaler value to have TIM4 counter clock equal to 1Mhz */
	uwPrescalerValue = (uint32_t) ((SystemCoreClock / 2) / 1000000) - 1;

	/* Set TIM4 instance the unstep timer */
	UnStepTickerTimHandle.Instance = UNSTEPTICKER_TIMx;
	UnStepTickerTimHandle.Init.Period = 3 - 1; // set period to trigger interrupt at 3us
	UnStepTickerTimHandle.Init.Prescaler = uwPrescalerValue;
	UnStepTickerTimHandle.Init.ClockDivision = 0;
	UnStepTickerTimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;

	if(HAL_TIM_Base_Init(&UnStepTickerTimHandle) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

	// don't start it here, only gets started when a step needs to be unstepped

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
uint32_t xst, xet;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// this is the step ticker
	if(htim->Instance == STEPTICKER_TIMx) {
		// handle stepticker
		xst= start_time();
		if(!issueTicks()) {
			// signal the next block to start, handled in moveCompletedThread
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			vTaskNotifyGiveFromISR( moveCompletedThreadHandle, &xHigherPriorityTaskWoken );
			portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		}

	}else if(htim->Instance == UNSTEPTICKER_TIMx) {
		// handle unstep ticker
		issueUnstep();
		// stop the timer
		HAL_TIM_Base_Stop_IT(&UnStepTickerTimHandle);
		// reset the count for next time
		__HAL_TIM_SET_COUNTER(&UnStepTickerTimHandle, 0);
		// Or this apparently will stop interrupts and reset counter
		//UnStepTickerTimHandle.Instance->CR1 |= (TIM_CR1_UDIS);
		//UnStepTickerTimHandle.Instance->EGR |= (TIM_EGR_UG);
	}
}

// this will start the unstep ticker
void startUnstepTicker()
{
	HAL_TIM_Base_Start_IT(&UnStepTickerTimHandle);
	// Or this apparently will restart interrupts
	//UnStepTickerTimHandle.Instance->CR1 &= ~(TIM_CR1_UDIS);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
	BSP_LED_Toggle(LED4);
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

// Non volatile storage area in flash
/* Base address of the Flash sectors Bank 1 */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */

/* Base address of the Flash sectors Bank 2 */
#define ADDR_FLASH_SECTOR_12     ((uint32_t)0x08100000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_13     ((uint32_t)0x08104000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_14     ((uint32_t)0x08108000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_15     ((uint32_t)0x0810C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_16     ((uint32_t)0x08110000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_17     ((uint32_t)0x08120000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_18     ((uint32_t)0x08140000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_19     ((uint32_t)0x08160000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_20     ((uint32_t)0x08180000) /* Base @ of Sector 8, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_21     ((uint32_t)0x081A0000) /* Base @ of Sector 9, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_22     ((uint32_t)0x081C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_23     ((uint32_t)0x081E0000) /* Base @ of Sector 11, 128 Kbytes */

/**
  * @brief  Gets the sector of a given address
  * @param  None
  * @retval The sector of a given address
  */
static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;

  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
	sector = FLASH_SECTOR_0;
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
	sector = FLASH_SECTOR_1;
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
	sector = FLASH_SECTOR_2;
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
	sector = FLASH_SECTOR_3;
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
	sector = FLASH_SECTOR_4;
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
	sector = FLASH_SECTOR_5;
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
	sector = FLASH_SECTOR_6;
  }
  else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
  {
	sector = FLASH_SECTOR_7;
  }
  else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
  {
	sector = FLASH_SECTOR_8;
  }
  else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
  {
	sector = FLASH_SECTOR_9;
  }
  else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
  {
	sector = FLASH_SECTOR_10;
  }
  #ifdef USE_STM32F429I_DISCO
  else if((Address < ADDR_FLASH_SECTOR_12) && (Address >= ADDR_FLASH_SECTOR_11))
  {
	sector = FLASH_SECTOR_11;
  }
  else if((Address < ADDR_FLASH_SECTOR_13) && (Address >= ADDR_FLASH_SECTOR_12))
  {
	sector = FLASH_SECTOR_12;
  }
  else if((Address < ADDR_FLASH_SECTOR_14) && (Address >= ADDR_FLASH_SECTOR_13))
  {
	sector = FLASH_SECTOR_13;
  }
  else if((Address < ADDR_FLASH_SECTOR_15) && (Address >= ADDR_FLASH_SECTOR_14))
  {
	sector = FLASH_SECTOR_14;
  }
  else if((Address < ADDR_FLASH_SECTOR_16) && (Address >= ADDR_FLASH_SECTOR_15))
  {
	sector = FLASH_SECTOR_15;
  }
  else if((Address < ADDR_FLASH_SECTOR_17) && (Address >= ADDR_FLASH_SECTOR_16))
  {
	sector = FLASH_SECTOR_16;
  }
  else if((Address < ADDR_FLASH_SECTOR_18) && (Address >= ADDR_FLASH_SECTOR_17))
  {
	sector = FLASH_SECTOR_17;
  }
  else if((Address < ADDR_FLASH_SECTOR_19) && (Address >= ADDR_FLASH_SECTOR_18))
  {
	sector = FLASH_SECTOR_18;
  }
  else if((Address < ADDR_FLASH_SECTOR_20) && (Address >= ADDR_FLASH_SECTOR_19))
  {
	sector = FLASH_SECTOR_19;
  }
  else if((Address < ADDR_FLASH_SECTOR_21) && (Address >= ADDR_FLASH_SECTOR_20))
  {
	sector = FLASH_SECTOR_20;
  }
  else if((Address < ADDR_FLASH_SECTOR_22) && (Address >= ADDR_FLASH_SECTOR_21))
  {
	sector = FLASH_SECTOR_21;
  }
  else if((Address < ADDR_FLASH_SECTOR_23) && (Address >= ADDR_FLASH_SECTOR_22))
  {
	sector = FLASH_SECTOR_22;
  }
  else/*(Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_23))*/
  {
	sector = FLASH_SECTOR_23;
  }

  #else

  else
  {
	sector = FLASH_SECTOR_11;
  }

  #endif // USE_STM32F429I_DISCO
  return sector;
}

#ifdef USE_STM32F429I_DISCO
#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_23   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_SECTOR_23   /* End @ of user Flash area */
#else
#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_11   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_SECTOR_11   /* End @ of user Flash area */
#endif

/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;

size_t writeFlash(void *buf, size_t len, uint32_t offset)
{
	__disable_irq();
	uint32_t FirstSector = 0, NbOfSectors = 0, Address = 0;
	uint32_t SectorError = 0;
	bool stat= true;

	 /* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	  /* Erase the user Flash area (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	/* Get the 1st sector to erase */
	FirstSector = GetSector(FLASH_USER_START_ADDR);
	/* Get the number of sector to erase from 1st sector*/
	NbOfSectors = GetSector(FLASH_USER_END_ADDR) - FirstSector + 1;

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector = FirstSector;
	EraseInitStruct.NbSectors = NbOfSectors;

	/* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
	 you have to make sure that these data are rewritten before they are accessed during code
	 execution. If this cannot be done safely, it is recommended to flush the caches by setting the
	 DCRST and ICRST bits in the FLASH_CR register. */
	if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) == HAL_OK) {
		/* Program the user Flash area word by word  ***********/
		Address = FLASH_USER_START_ADDR;
		for (int i = 0; i < len; i+=4) {
			uint32_t data= *((uint32_t*)buf);
			if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, data) == HAL_OK) {
		  		Address += 4;
		  		buf += 4;

			} else {
				/* Error occurred while writing data in Flash memory.
				 User can add here some code to deal with this error */
				/*
				  FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
				*/
				//Error_Handler();
				stat= false;
				break;
			}
		}

	}else{
		/*
		  Error occurred while sector erase.
		  User can add here some code to deal with this error.
		  SectorError will contain the faulty sector and then to know the code error on this sector,
		  user can call function 'HAL_FLASH_GetError()'
		*/
		  /*
			FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
		  */
		//Error_Handler();
		stat= false;
	}

	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();
	__enable_irq();

	if(!stat) return 0;
	return len;
}

size_t readFlash(void *buf, size_t len, uint32_t offset)
{
	// just copy the data
	memcpy(buf, (void *)(FLASH_USER_START_ADDR+offset), len);
	return len;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
