#pragma once

#define MBED_RTOS
//#define FREE_RTOS

#if defined(MBED_RTOS) && defined(FREE_RTOS)
#error "Only one of FREE_RTOS or MBED_RTOS can be defined"
#endif

//#define STEP_TICKER_FREQUENCY 100000.0F
#define STEP_TICKER_FREQUENCY 20000.0F

#define SMOOTHIEBOARD
//#define AZTEEGX5_MINI

//#define PRINTER3D
//#define USE_PANEL
