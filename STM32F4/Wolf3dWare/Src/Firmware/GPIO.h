#pragma once

/*
	Define all the GPIO pins here
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

// defines a pin with the port and pin mask, must use port definition from above
// relies on pointer types being 32 bits
template <uint32_t TPort, uint16_t TPin>
class GPIOPin {
public:
    static const uint32_t Port = TPort;
    static const uint16_t Pin = TPin;
    static void set(bool set) {((GPIO_TypeDef*)Port)->BSRR = set ? Pin : Pin<<16; }
};

