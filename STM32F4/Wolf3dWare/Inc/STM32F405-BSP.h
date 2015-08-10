#pragma once

#ifndef USE_STM32F429I_DISCO

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include <stdbool.h>

/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Exported_Types
  * @{
  */
typedef enum
{
	LED4 = 0,
	LED3 = 1,
	LED5 = 2,
	LED6 = 3
} Led_TypeDef;

typedef enum
{
	BUTTON_KEY = 0,
} Button_TypeDef;

typedef enum
{
	BUTTON_MODE_GPIO = 0,
	BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef;


#ifdef OLIMEX

// Olimex
#define LEDn                             2

#define LED3_PIN                         GPIO_PIN_12
#define LED3_GPIO_PORT                   GPIOC
#define LED3_GPIO_CLK_ENABLE()           __GPIOC_CLK_ENABLE()
#define LED3_GPIO_CLK_DISABLE()          __GPIOC_CLK_DISABLE()

#define LED4_PIN                         GPIO_PIN_1
#define LED4_GPIO_PORT                   GPIOC
#define LED4_GPIO_CLK_ENABLE()           __GPIOC_CLK_ENABLE()
#define LED4_GPIO_CLK_DISABLE()          __GPIOC_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__INDEX__)  do{if((__INDEX__) == 0) LED4_GPIO_CLK_ENABLE(); else \
                                            if((__INDEX__) == 1) LED3_GPIO_CLK_ENABLE(); \
                                            }while(0)
#define LEDx_GPIO_CLK_DISABLE(__INDEX__) do{if((__INDEX__) == 0) LED4_GPIO_CLK_DISABLE(); else \
                                            if((__INDEX__) == 1) LED3_GPIO_CLK_DISABLE(); \
                                            }while(0)

#define BUTTONn                          1

/**
 * @brief Wakeup push-button
 */
#define KEY_BUTTON_PIN                GPIO_PIN_0
#define KEY_BUTTON_GPIO_PORT          GPIOA
#define KEY_BUTTON_GPIO_CLK_ENABLE()  do{ __GPIOA_CLK_ENABLE(); }while(0)
#define KEY_BUTTON_GPIO_CLK_DISABLE() do{ __GPIOA_CLK_DISABLE(); }while(0)
#define KEY_BUTTON_EXTI_IRQn          EXTI0_IRQn

#else

// Stamp
#define LEDn                             2

#define LED3_PIN                         GPIO_PIN_0
#define LED3_GPIO_PORT                   GPIOC
#define LED3_GPIO_CLK_ENABLE()           __GPIOC_CLK_ENABLE()
#define LED3_GPIO_CLK_DISABLE()          __GPIOC_CLK_DISABLE()

#define LED4_PIN                         GPIO_PIN_1
#define LED4_GPIO_PORT                   GPIOC
#define LED4_GPIO_CLK_ENABLE()           __GPIOC_CLK_ENABLE()
#define LED4_GPIO_CLK_DISABLE()          __GPIOC_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__INDEX__)   do{ (((__INDEX__) == 0) ? LED4_GPIO_CLK_ENABLE() : LED3_GPIO_CLK_ENABLE()); }while(0)
#define LEDx_GPIO_CLK_DISABLE(__INDEX__)  do{ (((__INDEX__) == 0) ? LED4_GPIO_CLK_DISABLE() : LED3_GPIO_CLK_DISABLE());}while(0)

#define BUTTONn                          1

/**
 * @brief Wakeup push-button
 */
#define KEY_BUTTON_PIN                GPIO_PIN_0
#define KEY_BUTTON_GPIO_PORT          GPIOA
#define KEY_BUTTON_GPIO_CLK_ENABLE()  __GPIOA_CLK_ENABLE()
#define KEY_BUTTON_GPIO_CLK_DISABLE() __GPIOA_CLK_DISABLE()
#define KEY_BUTTON_EXTI_IRQn          EXTI0_IRQn

#endif

#define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)     do{if((__INDEX__) == 0) KEY_BUTTON_GPIO_CLK_ENABLE(); \
                                                 }while(0)
#define BUTTONx_GPIO_CLK_DISABLE(__INDEX__)    do{if((__INDEX__) == 0) KEY_BUTTON_GPIO_CLK_DISABLE(); \
                                                 }while(0)


/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Exported_Functions
  * @{
  */
void     BSP_LED_Init(Led_TypeDef Led);
void     BSP_LED_On(Led_TypeDef Led);
void     BSP_LED_Off(Led_TypeDef Led);
void     BSP_LED_Toggle(Led_TypeDef Led);
void     BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef Mode);
uint32_t BSP_PB_GetState(Button_TypeDef Button);

int BSP_Init_Encoder();
uint16_t BSP_Read_Encoder();

void I2Cx_Init(void);
bool I2Cx_WriteData(uint8_t Addr, uint8_t *Data, uint16_t Len);
bool I2Cx_ReadData(uint8_t Addr, uint8_t *Data, uint16_t Len);

#ifdef __cplusplus
}
#endif

#endif // ndef USE_STM32F429I_DISCO
