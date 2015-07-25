#pragma once

#define MBED_RTOS
//#define FREE_RTOS

#if defined(MBED_RTOS) && defined(FREE_RTOS)
#error "Only one of FREE_RTOS or MBED_RTOS can be defined"
#endif

//#define STEP_TICKER_FREQUENCY 100000.0F
#define STEP_TICKER_FREQUENCY 20000.0F

#define TEMP_READINGS_PER_SECOND 5 // 20 slow down for testing
#define OVERSAMPLE_ADC 4 // number of bits of extra resolution required
#define OVERSAMPLE_SAMPLES (256+256) // the number of samples required is 4^OVERSAMPLE_ADC double it to filter out spikes
#define OVERSAMPLE_SAMPLERATE 40000 // sample rate for ADC
// we are sampling two channels so twice the amount of samples
#if ((OVERSAMPLE_SAMPLES*2*1000) / OVERSAMPLE_SAMPLERATE) >= (50-10)
	#warning "Sample rate is too low needs to take less than 50ms"
#endif

#define SMOOTHIEBOARD
//#define AZTEEGX5_MINI

#define PRINTER3D
