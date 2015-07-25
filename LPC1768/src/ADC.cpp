#include "config.h"

#include "mbed.h"
#include "MODDMA.h"

/*
    DMA the ADC from P0.23 and P0.24 into a buffer continuously
    it will be filtered and averaged later
*/

// we multiply by two as we are sampling two channels
#define SAMPLE_BUFFER_LENGTH OVERSAMPLE_SAMPLES*2

// externally accesible buffer TODO needs to go into AHB1
static uint16_t adc_buffer[2][SAMPLE_BUFFER_LENGTH/2] __attribute__ ((section ("AHBSRAM0")));
static MODDMA dma;
static MODDMA_Config *conf;
static volatile bool adc_running= true;

// two channels are interleaved, 32 bits each channel
static uint32_t adcInputBuffer[SAMPLE_BUFFER_LENGTH] __attribute__ ((section ("AHBSRAM0")));

extern uint32_t start_time();
extern uint32_t stop_time();

uint32_t adc_t1, adc_t2;
uint32_t adc_actual_sample_rate;

bool isADCReady()
{
    return !adc_running;
}

void startADC()
{
    adc_running= true;
    adc_t1= start_time();
    // Schedule another grab.
    dma.Setup( conf );
    dma.Enable( conf );

    // Enable ADC irq flag (to DMA).
    LPC_ADC->ADINTEN = 0x100;
    // Enable burst mode on inputs 0 and 1.
    LPC_ADC->ADCR |= (1UL << 16);
}

uint16_t *getADC(uint8_t ch)
{
    uint16_t *p= nullptr;
    switch(ch) {
        case 0: p= adc_buffer[0]; break;
        case 1: p= adc_buffer[1]; break;
        default: startADC(); // kick off the next sample
    }
    return p;
}

// Configuration callback on TC
void TC0_callback(void)
{
    adc_t2= stop_time();
    MODDMA_Config *config = dma.getConfig();

    // Disable burst mode and switch off the IRQ flag.
    LPC_ADC->ADCR &= ~(1UL << 16);
    LPC_ADC->ADINTEN = 0;

    // Finish the DMA cycle by shutting down the channel.
    dma.haltAndWaitChannelComplete( (MODDMA::CHANNELS)config->channelNum());
    dma.Disable( (MODDMA::CHANNELS)config->channelNum() );

    // Clear DMA IRQ flags.
    if (dma.irqType() == MODDMA::TcIrq) dma.clearTcIrq();
    if (dma.irqType() == MODDMA::ErrIrq) dma.clearErrIrq();

 	// copy results into result buffer
    for (int i = 0; i < SAMPLE_BUFFER_LENGTH; i++) {
        int channel = (adcInputBuffer[i] >> 24) & 0x7;
        if(channel >= 0 && channel <= 1) {
            adc_buffer[channel][i>>1] = (adcInputBuffer[i] >> 4) & 0xFFF;
        }
    }

    adc_running= false;
}

// Configuration callback on Error
void ERR0_callback(void)
{
    // Switch off burst conversions.
    LPC_ADC->ADCR |= ~(1UL << 16);
    LPC_ADC->ADINTEN = 0;
    error("Oh no! My Mbed EXPLODED! :( Only kidding, go find the problem");
}
/*
    LPC_SC->PLL0CFG   = 0x0000000E; // 120Mhz
                        0x00010018; // 100Mhz

    adc_clk_freq=CLKS_PER_SAMPLE*sample_rate; == 64 * sample_rate
    int m = (LPC_SC->PLL0CFG & 0xFFFF) + 1; == 25 / == 15
    int n = (LPC_SC->PLL0CFG >> 16) + 1;  == 2 / == 1
    int cclkdiv = LPC_SC->CCLKCFG + 1;  == 3
    int Fcco = (2 * m * XTAL_FREQ) / n; == 300MHz / == 360MHz
    int cclk = Fcco / cclkdiv; == 100Mhz / == 120MHz

    //Power up the ADC
    LPC_SC->PCONP |= (1 << 12);
    //Set clock at cclk / 1.
    LPC_SC->PCLKSEL0 &= ~(0x3 << 24);
    switch (cclk_div) {
        case 1:
            LPC_SC->PCLKSEL0 |= 0x1 << 24;
            break;
        case 2:
            LPC_SC->PCLKSEL0 |= 0x2 << 24;
            break;
        case 4:
            LPC_SC->PCLKSEL0 |= 0x0 << 24;
            break;
        case 8:
            LPC_SC->PCLKSEL0 |= 0x3 << 24;
            break;
        default:
            fprintf(stderr, "Warning: ADC CCLK clock divider must be 1, 2, 4 or 8. %u supplied.\n",
                cclk_div);
            fprintf(stderr, "Defaulting to 1.\n");
            LPC_SC->PCLKSEL0 |= 0x1 << 24;
            break;
    }
    pclk = cclk / cclk_div; == /4 25MHz, /8 12.5MHz / /4 30MHz, /8 15Mhz

    clock_div=pclk / adc_clk_freq; == 25Mhz / (64 * samplerate) 1,280,000

    if (clock_div > 0xFF) {
        fprintf(stderr, "Warning: Clock division is %u which is above 255 limit. Re-Setting at limit.\n",
            clock_div);
        clock_div=0xFF;
    }
    if (clock_div == 0) {
        fprintf(stderr, "Warning: Clock division is 0. Re-Setting to 1.\n");
        clock_div=1;
    }

    _adc_clk_freq=pclk / clock_div;
    if (_adc_clk_freq > MAX_ADC_CLOCK) {
        fprintf(stderr, "Warning: Actual ADC sample rate of %u which is above %u limit\n",
            _adc_clk_freq / CLKS_PER_SAMPLE, MAX_ADC_CLOCK / CLKS_PER_SAMPLE);
        while ((pclk / max_div) > MAX_ADC_CLOCK) max_div++;
        fprintf(stderr, "Maximum recommended sample rate is %u\n", (pclk / max_div) / CLKS_PER_SAMPLE);
    }
*/

void InitializeADC(int sample_rate)
{
    memset(adcInputBuffer, 0, sizeof(adcInputBuffer));

    // We use the ADC irq to trigger DMA and the manual says
    // that in this case the NVIC for ADC must be disabled.
    NVIC_DisableIRQ(ADC_IRQn);

    // Power up the ADC and set PCLK
    LPC_SC->PCONP    |=  (1UL << 12);
    LPC_SC->PCLKSEL0 &= ~(3UL << 24);
	//LPC_SC->PCLKSEL0 |= 0x00 << 24; // PCLK = CCLK/4 100MHz/4 = 25MHz 120MHz/4= 30MHz

    // PCLK / adc sample rate
    uint32_t pclk= SystemCoreClock / 4;
    uint32_t clock_div= pclk / (64*sample_rate);
    if (clock_div > 0xFF) {
        fprintf(stderr, "Warning: Clock division is %lu which is above 255 limit. Re-Setting at limit.\n", clock_div);
        clock_div= 0xFF;
    }
    if (clock_div == 0) {
        fprintf(stderr, "Warning: Clock division is 0. Re-Setting to 1.\n");
        clock_div= 1;
    }
    adc_actual_sample_rate= (pclk/clock_div) / 64;

    // Enable the ADC, sample_rate (20KHz),  ADC0.0 & .1
    LPC_ADC->ADCR  = (1UL << 21) | ((clock_div-1) << 8) | (3UL << 0);

    // Set the pin functions to ADC
    LPC_PINCON->PINSEL1 &= ~(3UL << 14);  /* P0.23, Mbed p15. */
    LPC_PINCON->PINSEL1 |=  (1UL << 14);
    LPC_PINCON->PINSEL1 &= ~(3UL << 16);  /* P0.24, Mbed p16. */
    LPC_PINCON->PINSEL1 |=  (1UL << 16);

    // Prepare an ADC configuration.
    conf = new MODDMA_Config;
    conf
    ->channelNum    ( MODDMA::Channel_0 )
    ->srcMemAddr    ( 0 )
    ->dstMemAddr    ( (uint32_t)adcInputBuffer )
    ->transferSize  ( SAMPLE_BUFFER_LENGTH )
    ->transferType  ( MODDMA::p2m )
    ->transferWidth ( MODDMA::word ) // 4 bytes
    ->srcConn       ( MODDMA::ADC )
    ->dstConn       ( 0 )
    ->dmaLLI        ( 0 )
    ->attach_tc     ( &TC0_callback )
    ->attach_err    ( &ERR0_callback )
    ; // end conf.

    // Prepare configuration.
    dma.Setup( conf );

    // Enable configuration.
    dma.Enable( conf );

    // Enable ADC irq flag (to DMA).
    // Note, don't set the individual flags,
    // just set the global flag.
    LPC_ADC->ADINTEN = 0x100;

    // Enable burst mode on inputs 0 and 1.
    LPC_ADC->ADCR |= (1UL << 16);
}
