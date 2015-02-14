#pragma once

/*
	Define all the GPIO pins here
*/
#include "stm32f4xx_hal.h"

template <uint32_t base_addr>
struct STM32F429_PORT {
 	static GPIO_TypeDef volatile *port() { return (GPIO_TypeDef volatile *)base_addr; }
};

using GPIO_PORT_A = STM32F429_PORT<(uint32_t)GPIOA>;
using GPIO_PORT_B = STM32F429_PORT<(uint32_t)GPIOB>;
using GPIO_PORT_C = STM32F429_PORT<(uint32_t)GPIOC>;
using GPIO_PORT_D = STM32F429_PORT<(uint32_t)GPIOD>;
using GPIO_PORT_E = STM32F429_PORT<(uint32_t)GPIOE>;
using GPIO_PORT_F = STM32F429_PORT<(uint32_t)GPIOF>;
using GPIO_PORT_G = STM32F429_PORT<(uint32_t)GPIOG>;
using GPIO_PORT_H = STM32F429_PORT<(uint32_t)GPIOH>;
using GPIO_PORT_I = STM32F429_PORT<(uint32_t)GPIOI>;
using GPIO_PORT_J = STM32F429_PORT<(uint32_t)GPIOJ>;
using GPIO_PORT_K = STM32F429_PORT<(uint32_t)GPIOK>;

template <typename TPort, uint16_t TPin>
class GPIOPin {
public:
    using Port = TPort;
    static const uint16_t Pin = TPin;
    //static void set(bool set) {Port p; HAL_GPIO_WritePin((GPIO_TypeDef*)p.port(), Pin, set?GPIO_PIN_SET:GPIO_PIN_RESET); }
    static void set(bool set) {Port p; p.port()->BSRR = set ? Pin : Pin<<16; }
};
