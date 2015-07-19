#include "mbed.h"
#include "rtos.h"
#include "USBSerial.h"

//Virtual serial port over USB
USBSerial serial;


void serial_thread(void const *args) {

	serial.printf("I am a virtual serial port\r\n");
	while(1)
	{
		char buf[128];
		if(serial.available() > 0){
			serial.gets(buf, sizeof(buf)-1);
			serial.printf("%s\n", buf);
		}else{
			Thread::wait(100);
		}
	}
}

DigitalOut led1(LED1);
DigitalOut led2(LED2);

void led2_thread(void const *args) {
	while (true) {
		led2 = !led2;
		Thread::wait(1000);
	}
}

extern int setup();
int main() {
	Thread thread1(led2_thread);
	Thread thread2(serial_thread);
	
	setup();
	
	while (true) {
		led1 = !led1;
		Thread::wait(500);
	}
}
