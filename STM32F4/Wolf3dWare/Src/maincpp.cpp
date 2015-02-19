#include "Firmware/Kernel.h"
#include "Firmware/GCodeProcessor.h"
#include "Firmware/Dispatcher.h"
#include "Firmware/GCode.h"
#include "Firmware/MotionControl.h"
#include "Firmware/Block.h"
#include "Firmware/Planner.h"
#include "Firmware/Actuator.h"

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

using namespace std;

// global
SemaphoreHandle_t READY_Q_MUTEX;
bool execute_mode= false;
uint32_t xdelta= 0;

// local
// signals that the ticks can start to be issued to the actuators
static volatile bool move_issued= false;
// count of ticks missed while setting up next move
static volatile uint32_t waiting_ticks= 0;

#define __debugbreak()  { __asm volatile ("bkpt #0"); }
static const bool INVERTPIN=false;

// define specific pins, test 3rd parameter to true if inverted
using X_StepPin = GPIOPin<GPIO_PORT_A,GPIO_PIN_5,INVERTPIN>; 	// PA5   P2-21
using X_DirPin  = GPIOPin<GPIO_PORT_A,GPIO_PIN_9,INVERTPIN>;    // PA9   P1-52
using X_EnbPin  = GPIOPin<GPIO_PORT_A,GPIO_PIN_10,INVERTPIN>;   // PA10  P1-51
using Y_StepPin = GPIOPin<GPIO_PORT_B,GPIO_PIN_4,INVERTPIN>; 	// PB4   P1-25
using Y_DirPin  = GPIOPin<GPIO_PORT_B,GPIO_PIN_7,INVERTPIN>;    // PB7   P1-24
using Y_EnbPin  = GPIOPin<GPIO_PORT_C,GPIO_PIN_3,INVERTPIN>;    // PC3   P2-15
using Z_StepPin = GPIOPin<GPIO_PORT_C,GPIO_PIN_8,INVERTPIN>; 	// PC8   P1-55
using Z_DirPin  = GPIOPin<GPIO_PORT_C,GPIO_PIN_11,INVERTPIN>;   // PC11  P1-44
using Z_EnbPin  = GPIOPin<GPIO_PORT_C,GPIO_PIN_12,INVERTPIN>;   // PC12  P1-43
using E_StepPin = GPIOPin<GPIO_PORT_C,GPIO_PIN_13,INVERTPIN>;   // PC13  P1-12
using E_DirPin  = GPIOPin<GPIO_PORT_D,GPIO_PIN_2,INVERTPIN>;    // PD2   P1-40
using E_EnbPin  = GPIOPin<GPIO_PORT_D,GPIO_PIN_4,INVERTPIN>;    // PD4   P1-39

using LED3Pin   = GPIOPin<GPIO_PORT_G,GPIO_PIN_13>;     // LED3
using LED4Pin   = GPIOPin<GPIO_PORT_G,GPIO_PIN_14>;     // LED4

using TriggerPin= GPIOPin<GPIO_PORT_D,GPIO_PIN_5>;      // PD5

// 11 Spare
//- PD5 P1-37
// PD7 P1-35
// PE2 -> PE6 P1-15,P1-16,P1-13,P1-14 ,P1-11
// PF6 P2-3
// PG2 P2-62
// PG3 P2-61
// PG9 P1-33

static void initializePins()
{
	// There maybe a better way to do this it is a little inconvenient this way

	GPIO_InitTypeDef  GPIO_InitStruct;

  	/* Enable the GPIO Clocks */
	__GPIOA_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();
	__GPIOC_CLK_ENABLE();
	__GPIOD_CLK_ENABLE();

	/* Configure the GPIO pins for output */
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

	// PA
	GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_9|GPIO_PIN_10;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	// PB
	GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_7;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	// PC
	GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	// PD
	GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	// init all to low
	// HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
	{X_StepPin::set(false);}
	{X_DirPin::set(false);}
	{X_EnbPin::set(false);}
	{Y_StepPin::set(false);}
	{Y_DirPin::set(false);}
	{Y_EnbPin::set(false);}
	{Z_StepPin::set(false);}
	{Z_DirPin::set(false);}
	{Z_EnbPin::set(false);}
	{E_StepPin::set(false);}
	{E_DirPin::set(false);}
	{E_EnbPin::set(false);}
	{TriggerPin::set(false);}
}

extern "C" void testGpio()
{
	static bool tog= false;
	LED4Pin::set(tog);
	tog= !tog;
}

extern "C" size_t writeFlash(void *, size_t, uint32_t);
extern "C" size_t readFlash(void *, size_t, uint32_t);

extern "C" void moveCompletedThread(void const *argument);

extern "C" int maincpp()
{
	READY_Q_MUTEX= xSemaphoreCreateMutex();

	// creates Kernel singleton and other singletons and Initializes MotionControl
	MotionControl& mc= THEKERNEL.getMotionControl();

	// assign the HAL function for Non volatile memory in the Kernel
	//THEKERNEL.assignHALFunction(Kernel::NV_INIT, [](void*,size_t)  { HAL_FLASH_INIT(); });
	THEKERNEL.assignHALFunction(Kernel::NV_WRITE, writeFlash);
	THEKERNEL.assignHALFunction(Kernel::NV_READ, readFlash);

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

	move_issued= false;
	waiting_ticks= 0;
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
		l.unLock();
		// sets up the move with all the actuators involved in this block
		move_issued= THEKERNEL.getMotionControl().issueMove(block);

	}else{
		l.unLock();
	}
}

extern "C" bool serial_reply(const char*, size_t);
void sendReply(const std::string& str)
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
		oss << "Wolf3dWare V0.1, Clock speed: " << SystemCoreClock/1000000.0F << " MHz\n";

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
		oss << "ok\n";

	}else if(strcmp(line, "stats") == 0) {
		oss << "Worst time: " << xdelta << "uS\nok\n";

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

	// Parse gcode
	GCodeProcessor::GCodes_t gcodes= gp.parse(line);

	// dispatch gcode to MotionControl and Planner
	for(auto i : gcodes) {
		if(THEDISPATCHER.dispatch(i)) {
			// send the result to the place it came from
			sendReply(THEDISPATCHER.getResult());

		}else{
			// no handler for this gcode, return ok - nohandler
			const char *str= "ok - nohandler\n";
			sendReply(std::string(str));
		}
	}
	return true;
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
	if(!mc.issueTicks(++current_tick)){
		// all moves finished
		TriggerPin::set(true);
		// need to protect against getting called again if this takes longer than 10uS
		move_issued= false; // this will not get set again until all the actuators have been setup for the next move
		current_tick= 0;
		waiting_ticks= 1; // setup to count any missed ticks
		return false;  // signals ISR to yield to the moveCompletedThread
	}
	xet= stop_time();
	uint32_t d= xet-xst;
	if(d > xdelta) xdelta= d;
	return true;
}

// run the block change in this thread when signaled
static uint32_t overflow= 0;
void moveCompletedThread(void const *argument)
{
	for(;;) {
		// wait until we have something to process
		uint32_t ulNotifiedValue= ulTaskNotifyTake( pdTRUE, portMAX_DELAY);
		if(ulNotifiedValue > 0) {
			if(ulNotifiedValue > 1) overflow++; // NOTE this cannot happen so remove it FIXME

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
	GCodeProcessor::GCodes_t gcodes= gp.parse("G1 X100 Y0 E1.0 F6000 G1 X100 Y100 E2.0 G1 X0 Y100 E3.0 G1 X0 Y0 E4.0 G1 X100 Y50 E4.75 M114");

	// dispatch gcode to MotionControl and Planner
	for(auto i : gcodes) {
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
