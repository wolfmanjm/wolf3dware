#pragma once

/*
	An abstraction of an Input pin
	entirely static and expanded inline
*/
#include "stm32f4xx_hal.h"


// needed to get around weird compiler errors
#define GPIO_PORT_A ((uint32_t)GPIOA)
#define GPIO_PORT_B ((uint32_t)GPIOB)
#define GPIO_PORT_C ((uint32_t)GPIOC)
#define GPIO_PORT_D ((uint32_t)GPIOD)
#define GPIO_PORT_E ((uint32_t)GPIOE)
#define GPIO_PORT_F ((uint32_t)GPIOF)
#define GPIO_PORT_G ((uint32_t)GPIOG)
#define GPIO_PORT_H ((uint32_t)GPIOH)
#define GPIO_PORT_I ((uint32_t)GPIOI)
#define GPIO_PORT_J ((uint32_t)GPIOJ)
#define GPIO_PORT_K ((uint32_t)GPIOK)

// defines a pin with the port and pin mask
// NOTE relies on pointer types being 32 bits
// Pin must be initialized using the output() or input() calls
template <uint32_t TPort, uint16_t TPin, uint32_t TClkEnable, bool inv= false>
class GPIOPin {
public:
    static const uint32_t Port = TPort;
    static const uint16_t Pin = TPin;
    static const uint32_t Clk_enable = TClkEnable;
    static void set(bool set) {((GPIO_TypeDef*)Port)->BSRR = (set!=inv) ? Pin : Pin<<16; }
    static bool get(void) { return ((((GPIO_TypeDef*)Port)->IDR & Pin) != 0); }
    static void output(bool s) {
    	/* Configure the GPIO pin for output */
		RCC->AHB1ENR |= Clk_enable; // enable the clock
    	GPIO_InitTypeDef  GPIO_InitStruct;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
		GPIO_InitStruct.Pin = Pin;
		HAL_GPIO_Init((GPIO_TypeDef*)Port, &GPIO_InitStruct);
		set(s);
	}
};

// Use this macro to define a pin, Port is A-G pin is 0-31, the
// optional 3rd parameter is true to invert the pin, default is false not
// inverted
#define GPIO(PORT,PIN,...) GPIOPin<GPIO_PORT_##PORT, GPIO_PIN_##PIN, RCC_AHB1ENR_GPIO##PORT##EN, ##__VA_ARGS__>;
