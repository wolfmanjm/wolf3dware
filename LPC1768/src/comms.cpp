#include "mbed.h"
#include "rtos.h"

#include "USBSerial.h"

#include <stdbool.h>
#include <string.h>

// usb CDC serial
static USBSerial *usb_serial= nullptr;

// reads lines from CDC and dispatched full lines to parser
static Thread *CDCThreadHandle= nullptr;

static void cdcThread(void const *argument);

extern bool commandLineHandler(const char*);
extern void kickQueue();
extern void setLed(int, bool);

static void serialReceived()
{
	if(CDCThreadHandle != nullptr){
		CDCThreadHandle->signal_set(0x1);
	}
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int commsSetup(void)
{
	usb_serial= new USBSerial();

	// setup USB CDC
	usb_serial->attach(serialReceived);

	// start coomunication threads
	CDCThreadHandle = new Thread(cdcThread, nullptr,  osPriorityNormal);

	return 1;
}

// called to send replies back to USB Serial
// Note must not exceed 64 bytes for USB
bool serial_reply(const char *buf, size_t len)
{
	return usb_serial->writeBlock((uint8_t*)buf, len);
}

#define MAXLINELEN 132
static char line[MAXLINELEN];
static uint16_t cnt = 0;

static void cdcThread(void const *argument)
{
	bool toggle= false;
	uint8_t c;
	for (;;) {

		// TODO handle timeout and kick queue every now and then
		Thread::signal_wait(0x1);
		setLed(2, toggle); toggle= !toggle;

		while(usb_serial->available() > 0) {
			c= usb_serial->_getc();

			if(c == '\n') {
				if(cnt == 0) continue; //ignore empty lines

				// dispatch on NL. This blocks if queue is full etc
				line[cnt]= 0;
				commandLineHandler(line);
				cnt= 0;

			}else if(c == '\r'){
				// ignore CR
				continue;

			}else if(c == 8 || c == 127) { // BS or DEL
				if(cnt > 0) --cnt;

			}else if(cnt >= sizeof(line)) {
				// discard the excess of long lines
				continue;

			}else{
				line[cnt++]= c;
			}
		}
	}
}
