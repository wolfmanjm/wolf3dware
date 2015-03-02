#include "Firmware/Kernel.h"
#include "Firmware/GCodeProcessor.h"
#include "Firmware/Dispatcher.h"
#include "Firmware/GCode.h"
#include "Firmware/MotionControl.h"
#include "Firmware/Block.h"
#include "Firmware/Planner.h"
#include "Firmware/Actuator.h"
#include "Firmware/Tools/TemperatureControl.h"
#include "Firmware/Tools/Thermistor.h"
#include "Firmware/Tools/Extruder.h"

#include "Lock.h"
#include "GPIO.h"

#include "cmsis_os.h"

#include <map>
#include <vector>
#include <iostream>
#include <stdint.h>
#include <ctype.h>
#include <cmath>
#include <assert.h>
#include <malloc.h>
#include <string.h>
#include <algorithm>

using namespace std;

#define PRINTER3D

// global
SemaphoreHandle_t READY_Q_MUTEX;
SemaphoreHandle_t TEMPERATURE_MUTEX;

uint32_t xdelta= 0;

// local
static volatile bool execute_mode= true;
// signals that the ticks can start to be issued to the actuators
static volatile bool move_issued= false;
// count of ticks missed while setting up next move
static volatile uint32_t waiting_ticks= 0;
static uint32_t overflow= 0;
// set to true as long as we are processing the queue
volatile bool running= false;

static size_t maxqsize= 0;

#define __debugbreak()  { __asm volatile ("bkpt #0"); }


// define specific pins, set 3rd parameter to true if inverted
static const bool INVERTPIN=false;

#ifdef USE_STM32F429I_DISCO
using X_StepPin = GPIO(A, 5,INVERTPIN);	// PA5   P2-21
using X_DirPin  = GPIO(A, 9,INVERTPIN); // PA9   P1-52
using X_EnbPin  = GPIO(A,10,INVERTPIN); // PA10  P1-51
using Y_StepPin = GPIO(B, 4,INVERTPIN); // PB4   P1-25
using Y_DirPin  = GPIO(B, 7,INVERTPIN); // PB7   P1-24
using Y_EnbPin  = GPIO(D, 7,INVERTPIN); // PD7   P1-35
using Z_StepPin = GPIO(C, 8,INVERTPIN); // PC8   P1-55
using Z_DirPin  = GPIO(C,11,INVERTPIN); // PC11  P1-44
using Z_EnbPin  = GPIO(C,12,INVERTPIN); // PC12  P1-43
using E_StepPin = GPIO(C,13,INVERTPIN); // PC13  P1-12
using E_DirPin  = GPIO(D, 2,INVERTPIN); // PD2   P1-40
using E_EnbPin  = GPIO(D, 4,INVERTPIN); // PD4   P1-39

using LED3Pin   = GPIO(G,13);           // PG13  LED3
using LED4Pin   = GPIO(G,14);           // PG14  LED4

using TriggerPin= GPIO(D, 5);           // PD5
// 11 Spare
// PC3   P2-15 - ADC1-IN13 adc channel13 DMA channel 0 stream 0 or 4
//- PD5 P1-37
// PD7 P1-35
// PE2 -> PE6 P1-15,P1-16,P1-13,P1-14 ,P1-11
// PE5      - PWM TIM9 PP2 Channel1
// PE6      - PWM TIM9 PP2 Channel2
// PF6 P2-3 - ADC3-IN4 adc channel4 DMA channel 2 stream 0 or 1
// PG2 P2-62
// PG3 P2-61
// PG9 P1-33

#else
// STM32F405
using X_StepPin = GPIO(B, 7,INVERTPIN);	//
using X_DirPin  = GPIO(B, 6,INVERTPIN); //
using Y_StepPin = GPIO(B, 5,INVERTPIN); //
using Y_DirPin  = GPIO(B, 4,INVERTPIN); //
using Z_StepPin = GPIO(B, 3,INVERTPIN); //
using Z_DirPin  = GPIO(D, 2,INVERTPIN); //
using E_StepPin = GPIO(C,12,INVERTPIN); //
using E_DirPin  = GPIO(C,11,INVERTPIN); //

using X_EnbPin  = GPIO(B,12,INVERTPIN); //
using Y_EnbPin  = GPIO(B,13,INVERTPIN); //
using Z_EnbPin  = GPIO(B,14,INVERTPIN); //
using E_EnbPin  = GPIO(B,15,INVERTPIN); //

using LED3Pin   = GPIO(C,0);            // PC0 LED3
using LED4Pin   = GPIO(C,1);            // PC1 LED4

using TriggerPin= GPIO(C, 2);           // PC2
/*
	PA0  - 				: button
	PA1  - 		:free
	PA2  -              : PWM TIM9 ch1
	PA3  -              : PWM TIM9 ch2
	PA4  -				: ADC HE  Ch4 ADC1
	PA5  -				: ADC Bed Ch5 ADC1
	PA6 - PA8 - :free
	PA9  - 				: VBUS
	PA10 - 				: USB-ID
	PA11 - 				: USB_DM
	PA12 - 				: USB_DP

	PA13 - PA15 :free

	PB2  - 				: Boot1
	PB3 - PB7			: Motor

	PB8 -       :free / I2C
	PB9 - 	    :free / I2C / SPI2 nss
	PB10  -     :free       / SPI2 sck
	PB11  -     :free
	PB12 - PB15			: motor

	PC0 - 				: LED3
	PC1 - 				: LED4
	PC2 - 				: LED5/Trigger Pin also SPI2 miso
	PC3 -				: LED6             also SPI2 mosi

	PC4 - PC10   :free

	PC11 - PC12			: motor

	PC13 - PC15  : free

	PD2         		: motor

*/

#endif // USE_STM32F429I_DISCO

static void initializePins()
{
	// set pins to output, and initially low
	X_StepPin::output(false);
	X_DirPin::output(false);
	X_EnbPin::output(false);
	Y_StepPin::output(false);
	Y_DirPin::output(false);
	Y_EnbPin::output(false);
	Z_StepPin::output(false);
	Z_DirPin::output(false);
	Z_EnbPin::output(false);
	E_StepPin::output(false);
	E_DirPin::output(false);
	E_EnbPin::output(false);

	TriggerPin::output(false);

	//LED4Pin::output(false);
}

extern "C" bool testGpio()
{
	static bool tog= false;
	LED4Pin::set(tog);
	tog= !tog;
	return LED4Pin::get();
}

extern "C" void getPosition(float *x, float *y, float *z, float *e)
{
	*x= THEKERNEL.getMotionControl().getActuator('X').getCurrentPositionInmm();
	*y= THEKERNEL.getMotionControl().getActuator('Y').getCurrentPositionInmm();
	*z= THEKERNEL.getMotionControl().getActuator('Z').getCurrentPositionInmm();
	*e= THEKERNEL.getMotionControl().getActuator('E').getCurrentPositionInmm();
}
extern "C" osThreadId MainThreadHandle;

// example of reading the DMA filled ADC buffer and taking the 4 middle values as average

static size_t doDelay(void *, size_t, uint32_t ms)
{
	const TickType_t xDelay = pdMS_TO_TICKS(ms);
	vTaskDelay( xDelay );
	return 0;
}

extern "C" size_t writeFlash(void *, size_t, uint32_t);
extern "C" size_t readFlash(void *, size_t, uint32_t);
extern "C" void setPWM(uint8_t channel, float percent);
extern "C" uint16_t* getADC(uint8_t ch);
extern "C" void InitializePWM();
extern "C" void InitializeADC();
extern "C" void moveCompletedThread(void const *argument);
extern "C" void startUnstepTicker();

static uint16_t readADC()
{
	uint16_t *adc_buf= getADC(0);
	// grab the dma buffer
	std::deque<uint16_t> buf(adc_buf, adc_buf+8);
	// sort
	std::sort (buf.begin(), buf.end());
	// eliminate first and last two
	buf.pop_back(); buf.pop_back();
	buf.pop_front(); buf.pop_front();
	uint16_t sum= std::accumulate(buf.begin(), buf.end(), 0);
	return roundf(sum/4.0F); // return the average
}

extern "C" int maincpp()
{
	READY_Q_MUTEX= xSemaphoreCreateMutex();
	TEMPERATURE_MUTEX= xSemaphoreCreateMutex();

	// creates Kernel singleton and other singletons and Initializes MotionControl
	MotionControl& mc= THEKERNEL.getMotionControl();

	// assign the HAL function for Non volatile memory in the Kernel
	//THEKERNEL.assignHALFunction(Kernel::NV_INIT, [](void*,size_t)  { HAL_FLASH_INIT(); });
	THEKERNEL.assignHALFunction(Kernel::NV_WRITE, writeFlash);
	THEKERNEL.assignHALFunction(Kernel::NV_READ, readFlash);

	// also a HAL independent task delay
	THEKERNEL.assignHALFunction(Kernel::DELAY, doDelay);

	// initialize Kernel and its modules
	THEKERNEL.initialize();

	// setup the pins
	initializePins();

	// Setup pins for each Actuator
	mc.getActuator('X').assignHALFunction(Actuator::SET_STEP, [](bool on)  { X_StepPin::set(on); });
	mc.getActuator('X').assignHALFunction(Actuator::SET_DIR, [](bool on)   { X_DirPin::set(on); });
	mc.getActuator('X').assignHALFunction(Actuator::SET_ENABLE, [](bool on){ X_EnbPin::set(on); });
	mc.getActuator('Y').assignHALFunction(Actuator::SET_STEP, [](bool on)  { Y_StepPin::set(on); });
	mc.getActuator('Y').assignHALFunction(Actuator::SET_DIR, [](bool on)   { Y_DirPin::set(on); });
	mc.getActuator('Y').assignHALFunction(Actuator::SET_ENABLE, [](bool on){ Y_EnbPin::set(on); });
	mc.getActuator('Z').assignHALFunction(Actuator::SET_STEP, [](bool on)  { Z_StepPin::set(on); });
	mc.getActuator('Z').assignHALFunction(Actuator::SET_DIR, [](bool on)   { Z_DirPin::set(on); });
	mc.getActuator('Z').assignHALFunction(Actuator::SET_ENABLE, [](bool on){ Z_EnbPin::set(on); });
	mc.getActuator('E').assignHALFunction(Actuator::SET_STEP, [](bool on)  { E_StepPin::set(on); });
	mc.getActuator('E').assignHALFunction(Actuator::SET_DIR, [](bool on)   { E_DirPin::set(on); });
	mc.getActuator('E').assignHALFunction(Actuator::SET_ENABLE, [](bool on){ E_EnbPin::set(on);  });

#ifdef PRINTER3D
	// needed for hotend
	InitializePWM(); // PWM control
	InitializeADC(); // ADC control

	// Setup the Temperature Control and sensors
	static Thermistor thermistor0(0);
	thermistor0.assignHALFunction(Thermistor::GET_ADC, getADC);

	// static Thermistor thermistor1(1);
	// thermistor1.assignHALFunction(Thermistor::GET_ADC, getADC);

	static TemperatureControl tc("T", 0, thermistor0);
	tc.assignHALFunction(TemperatureControl::SET_PWM, setPWM);
	tc.initialize();

	// static TemperatureControl bc("B", 1, thermistor1);
	// bc.assignHALFunction(TemperatureControl::SET_PWM, setPWM);
	// bc.initialize();

	// use same index as the associated temperature control
	// specify which axis is the extruder
	static Extruder ex('E', 0);
	ex.initialize();
#endif

	move_issued= false;
	waiting_ticks= 0;
	running= false;

	// load configuration from non volatile storage
	THEDISPATCHER.loadConfiguration(); // same as M501
	//sendReply(THEDISPATCHER.getResult());

	return 0;
}

void executeNextBlock()
{
	// queue needs to be protected with a Mutex
	Planner::Queue_t& q= THEKERNEL.getPlanner().getReadyQueue();
	Lock l(READY_Q_MUTEX);
	l.lock();
	if(!q.empty()) {
		Block block= q.back(); // incurs a copy as we destroy it in next instruction
		q.pop_back();
		l.unlock();
		// sets up the move with all the actuators involved in this block
		move_issued= THEKERNEL.getMotionControl().issueMove(block);
		running= true;

	}else{
		l.unlock();
		running= false;
	}
	// this lets main thread know we moved to plot the movements
	//xTaskNotify( MainThreadHandle, 0x02, eSetBits);
}

// TODO add a task to write responses to host as different tasks may need access
extern "C" bool serial_reply(const char*, size_t);
static void sendReply(const std::string& str)
{
	if(!str.empty()) {
		if(str.size() < 64) {
			serial_reply(str.c_str(), str.size());
		}else{
			//hack before we fix the cdc out
			int n= str.size();
			int off= 0;
			while(n > 0) {
				int s= min(63, n);
				serial_reply(str.substr(off, s).c_str(), s);
				off+=s;
				n-=s;
			}
		}
	}
}

extern "C" char _end;
extern "C" caddr_t _sbrk(int incr);
void free_memory(std::ostringstream& oss)
{
	uint32_t chunk_curr= (unsigned int)&_end;
	//if((chunk_curr % 8) != 0) chunk_curr += (8 - (chunk_curr % 8)); // align to next 8 byte boundary
	uint32_t chunk_number= 1;
	uint32_t used_space= 0;
	uint32_t free_space= 0;
	uint32_t heap_end= (uint32_t) _sbrk(0);
	//if ((heap_end % 8) != 0) heap_end -= (heap_end % 8); // round down to next 8 byte boundary

	while (chunk_curr < heap_end) {
		uint32_t chunk_size= *(unsigned int*)(chunk_curr+4) & ~1;
		uint32_t chunk_next= chunk_curr + chunk_size;
		int chunk_inuse= *(unsigned int*)(chunk_next+4) & 1;
		//oss << "Allocation: " << chunk_number << ", Address: " << chunk_curr+8 << ", Size: " << chunk_size-8 << "\n";
		if (chunk_inuse) {
			used_space += (chunk_size-8);
		} else {
			free_space += (chunk_size-8);
		}
		chunk_curr= chunk_next;
		chunk_number++;
	}
	uint32_t sp= (uint32_t)__get_MSP();
	oss << "Used malloc: " << used_space << " bytes\n";
	oss << "Free malloc: " << free_space << " bytes\n";
	oss << "Used heap: " <<  heap_end - (uint32_t)&_end << " bytes\n";
	oss << "Unused heap: " << sp - heap_end << " bytes\n";
	oss << "Total free: " << (sp - heap_end) + free_space << " bytes\n";
}

// runs in the commandThread context
bool handleCommand(const char *line)
{
	bool handled= true;
	std::ostringstream oss;

	if(strcmp(line, "dump") == 0) {
		// dump planned block queue
		THEKERNEL.getPlanner().dump(oss);

	}else if(strcmp(line, "connected") == 0) {
		oss << "Welcome to Wolf3dWare\r\nok\r\n";

	}else if(strcmp(line, "version") == 0) {
		oss << "Wolf3dWare V0.5, Clock speed: " << SystemCoreClock/1000000.0F << " MHz\n";
		#ifdef USE_STM32F429I_DISCO
		oss << "Running on STM32F429I_DISCO\n";
		#else
		oss << "Running on STM32F405 Stamp\n";
		#endif

	}else if(strcmp(line, "run") == 0) {
		THEKERNEL.getPlanner().moveAllToReady();
		executeNextBlock();
		execute_mode= true;
		oss << "ok\n";

	}else if(strcmp(line, "hold") == 0) {
		execute_mode= false;
		oss << "ok\n";

	}else if(strcmp(line, "kill") == 0) {
		execute_mode= false;
		THEKERNEL.getPlanner().purge();
		THEKERNEL.getMotionControl().resetAxisPositions();
		running= false;
		oss << "ok\n";

	}else if(strcmp(line, "stats") == 0) {
		oss << "Worst time: " << xdelta << "uS\n";
		oss << "worst overflow: " << overflow << " ticks\n";
		oss << "max q size: " << maxqsize << "\n";
		oss << "ok\n";

	}else if(strcmp(line, "mem") == 0) {
		free_memory(oss);

	}else if(strcmp(line, "adc") == 0) {
		for (int i = 0; i < 10; ++i) {
			uint16_t adc= readADC();
			oss << adc << ", ";
			THEKERNEL.delay(50);
		}
		oss << "\n";

	}else if(strncmp(line, "parse", 5) == 0) {
		GCodeProcessor& gp= THEKERNEL.getGCodeProcessor();
		GCodeProcessor::GCodes_t gcodes;
		bool r= gp.parse(&line[6], gcodes);
		oss << "returned: " << r << ": " << gp.getLineNumber()+1 << "\n";
		for(auto& i : gcodes) {
			oss << i;
		}
		oss << "\n";

	}else{
		oss << "Unknown command: " << line << "\n";
		handled= false;
	}

	sendReply(oss.str());

	return handled;
}


// gets called for each received line from USB serial port
// runs in the commandThread context
extern "C" bool commandLineHandler(const char *line)
{
	if(*line == 24) {
		// ^X means this is a command not gcode
		handleCommand(&line[1]);
		return true;
	}

	// Handle Gcode
	GCodeProcessor& gp= THEKERNEL.getGCodeProcessor();
	GCodeProcessor::GCodes_t gcodes;

	// Parse gcode
	if(!gp.parse(line, gcodes)){
		// line failed checksum, send resend request
		std::ostringstream oss;
		oss << "rs N" << gp.getLineNumber()+1 << "\r\n";
		sendReply(oss.str());
		return true;

	}else if(gcodes.empty()) {
		// if gcodes is empty then was a M110, just send ok
		sendReply("ok\r\n");
		return true;
	}

	// dispatch gcode to MotionControl and Planner
	for(auto& i : gcodes) {
		if(THEDISPATCHER.dispatch(i)) {
			// send the result to the place it came from
			sendReply(THEDISPATCHER.getResult());

		}else{
			// no handler for this gcode, return ok - nohandler
			const char *str= "ok - nohandler\r\n";
			sendReply(std::string(str));
		}
	}

	// check for large queue size, stall until it gets smaller
	const size_t MAX_Q= 100;
	Planner::Queue_t& q= THEKERNEL.getPlanner().getReadyQueue();
	Lock l(READY_Q_MUTEX);
	l.lock();
	size_t n= q.size();
	l.unlock();
	n += THEKERNEL.getPlanner().getLookAheadQueue().size();
	if(n > maxqsize) maxqsize= n;

	if(n > MAX_Q) {
		// we force it to start executing and if not currently running we start off the first block
		if(!execute_mode) execute_mode= true;
		if(!running) executeNextBlock();

		// wait for it to empty halfway
		l.lock();
		while(q.size() > MAX_Q/2) {
			l.unlock();
			THEKERNEL.delay(100);
			l.lock();
		}
		l.unlock();
	}
	return true;
}

// we have not recieved any comamnds for a while see if we can kickstart the queue running
extern "C" void kickQueue()
{
	if(execute_mode && !running) {
		Planner::Queue_t& q= THEKERNEL.getPlanner().getReadyQueue();
		Lock l(READY_Q_MUTEX);
		l.lock();
		size_t n=  q.size();
		l.unlock();
		if(n > 0) {
			// we have somethign in the queue we can execute
			executeNextBlock();
			return;
		}
		// check lookahead queue, no need to lock it as it is only manipulated in this thread
		Planner::Queue_t& lq= THEKERNEL.getPlanner().getLookAheadQueue();
		if(lq.size() > 0) {
			// move it into ready and execute it (probably a single jog command)
			THEKERNEL.getPlanner().moveAllToReady();
			executeNextBlock();
		}
	}
}

extern uint32_t xst, xet;
extern "C" uint32_t stop_time();
// run ticks in tick ISR, but the pri needs to be 5 otherwise we can't use signals
// worst case with 4 axis stepping is 8uS so far
extern "C" bool issueTicks()
{
	static uint32_t current_tick= 0;
	MotionControl& mc= THEKERNEL.getMotionControl();
	if(!execute_mode) return true;

	// TODO need to count missed ticks while moveCompletedThread is running
	if(!move_issued){
		// nothing asked to move so we don't need to do anything
		if(waiting_ticks > 0) waiting_ticks++; // this gets incremented if we are waiting for the next move to get setup
		return true;
	}
	if(waiting_ticks > overflow) overflow= waiting_ticks;

	// if we missed some ticks while processing the next move issue them here
	// if waiting_ticks == 0 then nothing was setup to move so we have not missed any ticks
	// if waiting_ticks == 1 then we may have setup a move so we count hown many ticxks we have missed
	// if waiting_ticks > 1 then we have missed waiting_ticks-1 ticks while the next blockj was being setup to move
	while(waiting_ticks > 1) {
		// we issue the number of ticks we missed while setting up the next move
		if(!mc.issueTicks(++current_tick)){
			// too many waiting ticks, we finished all the moves
			move_issued= false;
			current_tick= 0;
			waiting_ticks= 1;
			return false;
		}
		--waiting_ticks;
	}

	// a move was issued to the actuators, tick them until all moves are done

	bool all_moves_finished= !mc.issueTicks(++current_tick);

	if(mc.isStepped()) {
		// if a step or steps were set then start the unstep ticker
		startUnstepTicker();
	}

	bool moves_left= true;
	if(all_moves_finished) {
		// all moves finished
		TriggerPin::set(true);
		// need to protect against getting called again if this takes longer than 10uS
		move_issued= false; // this will not get set again until all the actuators have been setup for the next move
		current_tick= 0;
		waiting_ticks= 1; // setup to count any missed ticks
		moves_left= false;  // signals ISR to yield to the moveCompletedThread
	}

	xet= stop_time();
	uint32_t d= xet-xst;
	if(d > xdelta) xdelta= d;
	return moves_left;
}

extern "C" void issueUnstep()
{
	THEKERNEL.getMotionControl().issueUnsteps();
}

// run the block change in this thread when signaled
void moveCompletedThread(void const *argument)
{
	for(;;) {
		// wait until we have something to process
		uint32_t ulNotifiedValue= ulTaskNotifyTake( pdTRUE, portMAX_DELAY);
		if(ulNotifiedValue > 0) {
			// get next block, and setup the next move
			executeNextBlock();

			if(!move_issued) {
				// no moves were setup so disable the waiting tick count
				// if a move was issued then waiting_ticks would have been keeping count of how many we missed
				waiting_ticks= 0;
			}
			TriggerPin::set(false);
		}
	}
}

extern "C" void tests()
{
#if 0
	//malloc_stats();
	GCodeProcessor& gp= THEKERNEL.getGCodeProcessor();

	// Parse gcode - This would come from Serial port
	GCodeProcessor::GCodes_t gcodes;
	gp.parse("G1 X100 Y0 E1.0 F6000 G1 X100 Y100 E2.0 G1 X0 Y100 E3.0 G1 X0 Y0 E4.0 G1 X100 Y50 E4.75 M114", gcodes);

	// dispatch gcode to MotionControl and Planner
	for(auto& i : gcodes) {
		if(THEDISPATCHER.dispatch(i)) {
			// TODO send the result to the place it came from
			std::cout << THEDISPATCHER.getResult();

		}else{
			// no handler for this gcode, return ok - nohandler
		}
	}

	// dump planned block queue
	THEKERNEL.getPlanner().dump(cout);

	//malloc_stats();

	const Actuator& xact= THEKERNEL.getMotionControl().getActuator('X');
	const Actuator& yact= THEKERNEL.getMotionControl().getActuator('Y');
	const Actuator& eact= THEKERNEL.getMotionControl().getActuator('E');
	const float xpos[]{100,100,0,0,100};
	const float ypos[]{0,100,100,0,50};
	const float epos[]{1,2,3,4,4.75F};

	int cnt= 0;

	// iterate over block queue and setup steppers
	Planner::Queue_t& q= THEKERNEL.getPlanner().getQueue();
	while(!q.empty()) {
		Block block= q.back();
		q.pop_back();
		std::cout << "Playing Block: " << block.id << "\n";
		THEKERNEL.getMotionControl().issueMove(block);

		// simulate step ticker
		uint32_t current_tick= 0;
		bool r= true;
		while(r) {
			++current_tick;
			// sends ticks to all actuators, returns false when all have finished all steps
			r= THEKERNEL.getMotionControl().issueTicks(current_tick);
		}

		// check we got where we requested to go
		if(xact.getCurrentPositionInmm() != xpos[cnt]){ std::cout << "Error " << xact.getCurrentPositionInmm() << " != " << xpos[cnt] << "\n"; }
		if(yact.getCurrentPositionInmm() != ypos[cnt]){ std::cout << "Error " << yact.getCurrentPositionInmm() << " != " << ypos[cnt] << "\n"; }
		if(eact.getCurrentPositionInmm() != epos[cnt]){ std::cout << "Error " << eact.getCurrentPositionInmm() << " != " << epos[cnt] << "\n"; }

		++cnt;
		std::cout << "Done\n";
	}

	//malloc_stats();
#endif
}
