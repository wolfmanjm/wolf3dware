#pragma once

#ifndef USE_STM32F429I_DISCO

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

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


/** @addtogroup STM32F4_DISCOVERY_LOW_LEVEL_LED
  * @{
  */
#define LEDn                             4

#define LED3_PIN                         GPIO_PIN_0
#define LED3_GPIO_PORT                   GPIOC
#define LED3_GPIO_CLK_ENABLE()           __GPIOC_CLK_ENABLE()
#define LED3_GPIO_CLK_DISABLE()          __GPIOC_CLK_DISABLE()

#define LED4_PIN                         GPIO_PIN_1
#define LED4_GPIO_PORT                   GPIOC
#define LED4_GPIO_CLK_ENABLE()           __GPIOC_CLK_ENABLE()
#define LED4_GPIO_CLK_DISABLE()          __GPIOC_CLK_DISABLE()

#define LED5_PIN                         GPIO_PIN_2
#define LED5_GPIO_PORT                   GPIOC
#define LED5_GPIO_CLK_ENABLE()           __GPIOC_CLK_ENABLE()
#define LED5_GPIO_CLK_DISABLE()          __GPIOC_CLK_DISABLE()

#define LED6_PIN                         GPIO_PIN_3
#define LED6_GPIO_PORT                   GPIOC
#define LED6_GPIO_CLK_ENABLE()           __GPIOC_CLK_ENABLE()
#define LED6_GPIO_CLK_DISABLE()          __GPIOC_CLK_DISABLE()


#define LEDx_GPIO_CLK_ENABLE(__INDEX__)   (((__INDEX__) == 0) ? LED4_GPIO_CLK_ENABLE() :\
										   ((__INDEX__) == 1) ? LED3_GPIO_CLK_ENABLE() :\
										   ((__INDEX__) == 2) ? LED5_GPIO_CLK_ENABLE() : LED6_GPIO_CLK_ENABLE())

#define LEDx_GPIO_CLK_DISABLE(__INDEX__)  (((__INDEX__) == 0) ? LED4_GPIO_CLK_DISABLE() :\
										   ((__INDEX__) == 1) ? LED3_GPIO_CLK_DISABLE() :\
										   ((__INDEX__) == 2) ? LED5_GPIO_CLK_DISABLE() : LED6_GPIO_CLK_DISABLE())
#define BUTTONn                          1

/**
 * @brief Wakeup push-button
 */
#define KEY_BUTTON_PIN                GPIO_PIN_0
#define KEY_BUTTON_GPIO_PORT          GPIOA
#define KEY_BUTTON_GPIO_CLK_ENABLE()  __GPIOA_CLK_ENABLE()
#define KEY_BUTTON_GPIO_CLK_DISABLE() __GPIOA_CLK_DISABLE()
#define KEY_BUTTON_EXTI_IRQn          EXTI0_IRQn


#define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)    (KEY_BUTTON_GPIO_CLK_ENABLE())

#define BUTTONx_GPIO_CLK_DISABLE(__INDEX__)    (KEY_BUTTON_GPIO_CLK_DISABLE())


/*############################### SPI1 #######################################*/
#define DISCOVERY_SPIx                              SPI1
#define DISCOVERY_SPIx_CLK_ENABLE()                 __SPI1_CLK_ENABLE()
#define DISCOVERY_SPIx_GPIO_PORT                    GPIOA                      /* GPIOA */
#define DISCOVERY_SPIx_AF                           GPIO_AF5_SPI1
#define DISCOVERY_SPIx_GPIO_CLK_ENABLE()            __GPIOA_CLK_ENABLE()
#define DISCOVERY_SPIx_GPIO_CLK_DISABLE()           __GPIOA_CLK_DISABLE()
#define DISCOVERY_SPIx_SCK_PIN                      GPIO_PIN_5                 /* PA.05 */
#define DISCOVERY_SPIx_MISO_PIN                     GPIO_PIN_6                 /* PA.06 */
#define DISCOVERY_SPIx_MOSI_PIN                     GPIO_PIN_7                 /* PA.07 */

/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */
#define SPIx_TIMEOUT_MAX                            0x1000 /*<! The value of the maximal timeout for BUS waiting loops */


/*############################# I2C1 #########################################*/
/* I2C clock speed configuration (in Hz) */
#ifndef BSP_I2C_SPEED
#define BSP_I2C_SPEED                            100000
#endif /* BSP_I2C_SPEED */

/* I2C peripheral configuration defines (control interface of the audio codec) */
#define DISCOVERY_I2Cx                            I2C1
#define DISCOVERY_I2Cx_CLK_ENABLE()               __I2C1_CLK_ENABLE()
#define DISCOVERY_I2Cx_SCL_SDA_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()
#define DISCOVERY_I2Cx_SCL_SDA_AF                 GPIO_AF4_I2C1
#define DISCOVERY_I2Cx_SCL_SDA_GPIO_PORT          GPIOB
#define DISCOVERY_I2Cx_SCL_PIN                    GPIO_PIN_6
#define DISCOVERY_I2Cx_SDA_PIN                    GPIO_PIN_9

#define DISCOVERY_I2Cx_FORCE_RESET()              __I2C1_FORCE_RESET()
#define DISCOVERY_I2Cx_RELEASE_RESET()            __I2C1_RELEASE_RESET()

/* I2C interrupt requests */
#define DISCOVERY_I2Cx_EV_IRQn                    I2C1_EV_IRQn
#define DISCOVERY_I2Cx_ER_IRQn                    I2C1_ER_IRQn

/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */
#define I2Cx_TIMEOUT_MAX    0x1000 /*<! The value of the maximal timeout for BUS waiting loops */


/*############################# ACCELEROMETER ################################*/
/* Read/Write command */
#define READWRITE_CMD                     ((uint8_t)0x80)
/* Multiple byte read/write command */
#define MULTIPLEBYTE_CMD                  ((uint8_t)0x40)
/* Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device */
#define DUMMY_BYTE                        ((uint8_t)0x00)

/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Exported_Functions
  * @{
  */
void     BSP_LED_Init(Led_TypeDef Led);
void     BSP_LED_On(Led_TypeDef Led);
void     BSP_LED_Off(Led_TypeDef Led);
void     BSP_LED_Toggle(Led_TypeDef Led);
void     BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef Mode);
uint32_t BSP_PB_GetState(Button_TypeDef Button);


#ifdef __cplusplus
}
#endif

#endif // USE_STM32F429I_DISCO
