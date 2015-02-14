#include "Firmware/Kernel.h"
#include "Firmware/GCodeProcessor.h"
#include "Firmware/Dispatcher.h"
#include "Firmware/GCode.h"
#include "Firmware/MotionControl.h"
#include "Firmware/Block.h"
#include "Firmware/Planner.h"
#include "Firmware/Actuator.h"

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

#include "Firmware/GPIO.h"

#define __debugbreak()  { __asm volatile ("bkpt #0"); }

// define specific pins
using X_StepPin = GPIOPin<GPIO_PORT_A,GPIO_PIN_5>; // PA5   P2-21
using X_DirPin  = GPIOPin<GPIO_PORT_A,GPIO_PIN_9>; // PA9   P1-52
using X_EnbPin  = GPIOPin<GPIO_PORT_A,GPIO_PIN_10>;// PA10  P1-51
using Y_StepPin = GPIOPin<GPIO_PORT_B,GPIO_PIN_4>; // PB4   P1-25
using Y_DirPin  = GPIOPin<GPIO_PORT_B,GPIO_PIN_7>; // PB7   P1-24
using Y_EnbPin  = GPIOPin<GPIO_PORT_C,GPIO_PIN_3>; // PC3   P2-15
using Z_StepPin = GPIOPin<GPIO_PORT_C,GPIO_PIN_8>; // PC8   P1-55
using Z_DirPin  = GPIOPin<GPIO_PORT_C,GPIO_PIN_11>;// PC11  P1-44
using Z_EnbPin  = GPIOPin<GPIO_PORT_C,GPIO_PIN_12>;// PC12  P1-43
using E_StepPin = GPIOPin<GPIO_PORT_C,GPIO_PIN_13>;// PC13  P1-12
using E_DirPin  = GPIOPin<GPIO_PORT_D,GPIO_PIN_2>; // PD2   P1-40
using E_EnbPin  = GPIOPin<GPIO_PORT_D,GPIO_PIN_4>; // PD4   P1-39

using LED3Pin   = GPIOPin<GPIO_PORT_G,GPIO_PIN_13>;// LED3
using LED4Pin   = GPIOPin<GPIO_PORT_G,GPIO_PIN_14>;// LED4

// 11 Spare
// PD5 P1-37
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
	GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	// init all to low
	// HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
	{X_StepPin p; p.set(false);}
	{X_DirPin p;  p.set(false);}
	{X_EnbPin p;  p.set(false);}
	{Y_StepPin p; p.set(false);}
	{Y_DirPin p;  p.set(false);}
	{Y_EnbPin p;  p.set(false);}
	{Z_StepPin p; p.set(false);}
	{Z_DirPin p;  p.set(false);}
	{Z_EnbPin p;  p.set(false);}
	{E_StepPin p; p.set(false);}
	{E_DirPin p;  p.set(false);}
	{E_EnbPin p;  p.set(false);}
}

extern "C" void testGpio()
{
	static bool tog= false;
	LED4Pin pin;
	pin.set(tog);
	tog= !tog;
}

#include "cmsis_os.h"
osThreadId TickThreadHandle;
extern "C" void tickThread(void const *argument);

extern "C" int maincpp()
{
	osThreadDef(Tick, tickThread, osPriorityRealtime, 0, 1000);
	TickThreadHandle = osThreadCreate (osThread(Tick), NULL);
	if(TickThreadHandle == NULL) {
		__debugbreak();
	}

	// creates Kernel singleton and other singletoms and Initializes MotionControl
	MotionControl& mc= THEKERNEL.getMotionControl();

	initializePins();

	// Setup pins for each Actuator
	mc.getActuator('X').assignHALFunction(Actuator::SET_STEP, [](bool on)  { X_StepPin p; p.set(on); });
	mc.getActuator('X').assignHALFunction(Actuator::SET_DIR, [](bool on)   { X_DirPin p; p.set(on); });
	mc.getActuator('X').assignHALFunction(Actuator::SET_ENABLE, [](bool on){ X_EnbPin p; p.set(on); });
	mc.getActuator('Y').assignHALFunction(Actuator::SET_STEP, [](bool on)  { Y_StepPin p; p.set(on); });
	mc.getActuator('Y').assignHALFunction(Actuator::SET_DIR, [](bool on)   { Y_DirPin p; p.set(on); });
	mc.getActuator('Y').assignHALFunction(Actuator::SET_ENABLE, [](bool on){ Y_EnbPin p; p.set(on); });
	mc.getActuator('Z').assignHALFunction(Actuator::SET_STEP, [](bool on)  { Z_StepPin p; p.set(on); });
	mc.getActuator('Z').assignHALFunction(Actuator::SET_DIR, [](bool on)   { Z_DirPin p; p.set(on); });
	mc.getActuator('Z').assignHALFunction(Actuator::SET_ENABLE, [](bool on){ Z_EnbPin p; p.set(on); });
	mc.getActuator('E').assignHALFunction(Actuator::SET_STEP, [](bool on)  { E_StepPin p; p.set(on); });
	mc.getActuator('E').assignHALFunction(Actuator::SET_DIR, [](bool on)   { E_DirPin p; p.set(on); });
	mc.getActuator('E').assignHALFunction(Actuator::SET_ENABLE, [](bool on){ E_EnbPin p; p.set(on);  });
	return 0;
}

void executeMotion()
{
	// iterate over block queue and setup steppers
	Planner::Queue_t& q= THEKERNEL.getPlanner().getQueue();
	while(!q.empty()) {
		Block block= q.back();
		//if(!block.ready()) wait longer
		q.pop_back();
		THEKERNEL.getMotionControl().issueMove(block);

		// wait until move is finished then execute next block

	}
}

extern "C" bool serial_reply(const char*, size_t);

bool handleCommand(const char *line)
{
	bool handled= true;
	std::ostringstream oss;

	if(strcmp(line, "dump") == 0) {
		// dump planned block queue
		THEKERNEL.getPlanner().dump(oss);

	}else if(strcmp(line, "version") == 0) {
		oss << "Wolf3dWare V0.1\n";

	}else if(strcmp(line, "play") == 0) {
		executeMotion();
		oss << "ok\n";

	}else{
		oss << "Unknown command: " << line << "\n";
		handled= false;
	}

	std::string str= oss.str();
	if(!str.empty()) {
		serial_reply(str.c_str(), str.size());
	}

	return handled;
}


// gets called for each received line from USB serial port
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
			serial_reply(THEDISPATCHER.getResult().c_str(), THEDISPATCHER.getResult().size());

		}else{
			// no handler for this gcode, return ok - nohandler
			const char *str= "ok - nohandler\n";
			serial_reply(str, strlen(str));
		}
	}
	return true;
}

// Do not want this to run in stepticker ISR, so we signal a high priority Thread to run it
extern "C" void issueTicks()
{
	BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    vTaskNotifyGiveFromISR( TickThreadHandle, &xHigherPriorityTaskWoken );

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

static uint32_t currentTick= 0;
void tickThread(void const *argument)
{
	const TickType_t xTicksToWait = pdMS_TO_TICKS( 100000 );
	for(;;) {
		// wait until we have something to process
		uint32_t ulNotifiedValue= ulTaskNotifyTake( pdTRUE, xTicksToWait);
		while(ulNotifiedValue > 0) {
			if(!THEKERNEL.getMotionControl().issueTicks(++currentTick)){
				currentTick= 0;
				// finished all moves
				// signal the next block to start
			}

			--ulNotifiedValue;
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
		//if(!block.ready()) wait longer
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
