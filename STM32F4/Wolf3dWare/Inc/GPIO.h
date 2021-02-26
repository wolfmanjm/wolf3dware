#pragma once

/*
    An abstraction of an Input pin
    entirely static and expanded inline
*/
#include "stm32f4xx_hal.h"

enum class GPIO_PORT : uint32_t {
    A = GPIOA_BASE,
    B = GPIOB_BASE,
    C = GPIOC_BASE,
    D = GPIOD_BASE,
    #ifdef USE_STM32F429I_DISCO
    E = GPIOE_BASE,
    F = GPIOF_BASE,
    G = GPIOG_BASE,
    H = GPIOH_BASE,
    I = GPIOI_BASE,
    J = GPIOJ_BASE,
    K = GPIOK_BASE,
    #endif
};

// defines a pin with the port and pin mask
// NOTE relies on pointer types being 32 bits
// Pin must be initialized using the output() or input() calls
template <GPIO_PORT TPort, uint16_t TPin, uint32_t TClkEnable, bool inv = false>
class GPIOPin
{
public:
    static const uint16_t Pin = TPin;
    static const uint32_t Clk_enable = TClkEnable;
    static void set(bool set)
    {
        ((GPIO_TypeDef *)TPort)->BSRR = ((set != inv) ? Pin : (Pin << 16));
    }
    static bool get(void)
    {
        return ((((GPIO_TypeDef *)TPort)->IDR & Pin) != 0);
    }
    static void output(bool s)
    {
        /* Configure the GPIO pin for output */
        RCC->AHB1ENR |= Clk_enable; // enable the clock
        GPIO_InitTypeDef  GPIO_InitStruct;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
        GPIO_InitStruct.Pin = Pin;
        HAL_GPIO_Init((GPIO_TypeDef *)TPort, &GPIO_InitStruct);
        set(s);
    }
    static uint16_t input(bool pullup=true, bool irq=false, bool rising=true, uint32_t pri=0x0F)
    {
        /* Configure the GPIO pin for input and possibly generate an interrupt, and pullup */
        RCC->AHB1ENR |= Clk_enable; // enable the clock
        GPIO_InitTypeDef  GPIO_InitStruct;
        if (!irq) {
            /* Configure pin as input */
            GPIO_InitStruct.Pin = Pin;
            GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
            GPIO_InitStruct.Pull = pullup?GPIO_PULLUP:GPIO_NOPULL;
            GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
            HAL_GPIO_Init((GPIO_TypeDef *)TPort, &GPIO_InitStruct);

        }else{
            /* Configure pin as input with External interrupt */
            GPIO_InitStruct.Pin = Pin;
            GPIO_InitStruct.Pull = pullup?GPIO_PULLUP:GPIO_NOPULL;
            GPIO_InitStruct.Mode = rising?GPIO_MODE_IT_RISING:GPIO_MODE_IT_FALLING;
            HAL_GPIO_Init((GPIO_TypeDef *)TPort, &GPIO_InitStruct);

            /* Enable and set Button EXTI Interrupt to the specified priority */
            // TODO need to map the pin to the EXTIn_IRQn
            HAL_NVIC_SetPriority(getIRQ(), pri, 0x00);
            HAL_NVIC_EnableIRQ(getIRQ());
        }
        return Pin;
    }
/*
	returns the IRQn needed for the Pin
    EXTI0_IRQn		EXTI0_IRQHandler	Handler for pins connected to line 0
	EXTI1_IRQn		EXTI1_IRQHandler	Handler for pins connected to line 1
	EXTI2_IRQn		EXTI2_IRQHandler	Handler for pins connected to line 2
	EXTI3_IRQn		EXTI3_IRQHandler	Handler for pins connected to line 3
	EXTI4_IRQn		EXTI4_IRQHandler	Handler for pins connected to line 4
	EXTI9_5_IRQn	EXTI9_5_IRQHandler	Handler for pins connected to line 5 to 9
	EXTI15_10_IRQn	EXTI15_10_IRQHandler	Handler for pins connected to line 10 to 15
*/
	static IRQn_Type getIRQ()
	{
		switch(Pin) {
			case GPIO_PIN_0: return EXTI0_IRQn;
			case GPIO_PIN_1: return EXTI1_IRQn;
			case GPIO_PIN_2: return EXTI2_IRQn;
			case GPIO_PIN_3: return EXTI3_IRQn;
			case GPIO_PIN_4: return EXTI4_IRQn;
			case GPIO_PIN_5:
			case GPIO_PIN_6:
			case GPIO_PIN_7:
			case GPIO_PIN_8:
			case GPIO_PIN_9: return EXTI9_5_IRQn;
			case GPIO_PIN_10:
			case GPIO_PIN_11:
			case GPIO_PIN_12:
			case GPIO_PIN_13:
			case GPIO_PIN_14:
			case GPIO_PIN_15: return EXTI15_10_IRQn;
		}
		return EXTI0_IRQn;
	}
};

// Use this macro to define a pin, Port is A-G pin is 0-15, the
// optional 3rd parameter is true to invert the pin, default is false not
// inverted
#define GPIO(PORT,PIN,...) GPIOPin<GPIO_PORT::PORT, GPIO_PIN_##PIN, RCC_AHB1ENR_GPIO##PORT##EN, ##__VA_ARGS__>;
