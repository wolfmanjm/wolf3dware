#include "mbed.h"
#include "rtos.h"
#include "us_ticker_api.h"

extern int setup();
extern int commsSetup();

extern void stage2_setup();
extern void setLed(int, bool);

static osThreadId mainThreadID;


uint32_t start_time()
{
    return us_ticker_read();
}

uint32_t stop_time()
{
    return us_ticker_read();
}

void mainThread()
{
	// call any init stuff that has to happen after RTOS is started
	stage2_setup();

	// just toggle a led to show we are alive
	bool toggle= false;
	mainThreadID= osThreadGetId();
	osThreadSetPriority(mainThreadID, osPriorityIdle);
	while(1) {
		setLed(1, toggle);
		toggle= !toggle;
		Thread::wait(1000);
	}
}

int main() {
	//HAL_NVIC_SetPriorityGrouping( NVIC_PRIORITYGROUP_4 );

	commsSetup();
	setup();
	mainThread();
}
