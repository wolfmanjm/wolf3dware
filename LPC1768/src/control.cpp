#include "Firmware/Kernel.h"
#include "Firmware/GCodeProcessor.h"
#include "Firmware/Dispatcher.h"
#include "Firmware/GCode.h"
#include "Firmware/MotionControl.h"
#include "Firmware/Block.h"
#include "Firmware/Planner.h"
#include "Firmware/Actuator.h"

#include "mbed.h"
#include "rtos.h"

#include "Lock.h"

#include "mri.h"

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


// global
Mutex READY_Q_MUTEX;
Mutex TEMPERATURE_MUTEX;

static volatile bool execute_mode= true;
// signals that the ticks can start to be issued to the actuators
static volatile bool move_issued= false;
// count of ticks missed while setting up next move
static volatile uint32_t waiting_ticks= 0;
static uint32_t overflow= 0;
// set to true as long as we are processing the queue
volatile bool running= false;

static size_t maxqsize= 0;
static uint32_t xdelta= 0;
static Thread *moveCompletedThreadHandle= nullptr;

uint16_t readADC();
void setPWM(uint8_t channel, float percent);
uint16_t* getADC(uint8_t ch);
void InitializePWM();
void InitializeADC();

void startUnstepTicker();

// external references
bool serial_reply(const char*, size_t);
void setLed(int led, bool on);



// forward references
void kickQueue();

void initControl()
{
	move_issued= false;
	waiting_ticks= 0;
	running= false;
}

static void executeNextBlock()
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
	#ifdef USE_STM32F429I_DISCO
	// this lets main thread know we moved to plot the movements
	xTaskNotify( MainThreadHandle, 0x02, eSetBits);
	#endif
}

// TODO add a task to write responses to host as different tasks may need access
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

extern "C" char __end__;
extern "C" caddr_t _sbrk(int incr);
static void free_memory(std::ostringstream& oss)
{
	uint32_t chunk_curr= (unsigned int)&__end__;
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
	oss << "Used heap: " <<  heap_end - (uint32_t)&__end__ << " bytes\n";
	oss << "Unused heap: " << sp - heap_end << " bytes\n";
	oss << "Total free: " << (sp - heap_end) + free_space << " bytes\n";
}

// runs in the dcd thread context
static bool handleCommand(const char *line)
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
		#ifdef SMOOTHIEBOARD
		oss << "Running on SmoothieBoard\n";
		#elif defined(AZTEEGX5_MINI)
		oss << "Running on Azteeg X5 mini\n";
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

	}else if(strcmp(line, "tx") == 0) {
		// test USB tx
		for (int i = 0; i < 1000; ++i) {
			oss << "This is Line: " << i << "\n";
			sendReply(oss.str());
			THEKERNEL.delay(10);
			oss.str("");
			oss.clear();
		}

	}else{
		oss << "Unknown command: " << line << "\n";
		handled= false;
	}

	sendReply(oss.str());

	return handled;
}

// we have not recieved any commands for a while see if we can kickstart the queue running
void kickQueue()
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

// gets called for each received line from USB serial port
// runs in the commandThread context
bool commandLineHandler(const char *line)
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
		std::string ret= THEDISPATCHER.dispatch(i);
		if(!ret.empty()) {
			// send the result to the place it came from
			sendReply(ret);

		}else{
			// no handler for this gcode, return ok - nohandler
			const char *str= "ok - nohandler\r\n";
			sendReply(std::string(str));
		}
	}

	// check for large queue size, stall until it gets smaller
	const size_t MAX_Q= 32;
	Planner::Queue_t& q= THEKERNEL.getPlanner().getReadyQueue();
	Lock l(READY_Q_MUTEX);
	l.lock();
	size_t n= q.size() + THEKERNEL.getPlanner().getLookAheadQueue().size();
	l.unlock();
	if(n > maxqsize) maxqsize= n;

	if(n > MAX_Q) {
		// we force it to start executing and if not currently running we start off the first block
		if(!execute_mode) execute_mode= true;

		// wait for it to empty
		do{
			// we need to kick it in case the lookahead is full and ready is empty which can happen in certain cases
			kickQueue();
			THEKERNEL.delay(100);
			l.lock();
			n= q.size() + THEKERNEL.getPlanner().getLookAheadQueue().size();
			l.unlock();
		} while(n > MAX_Q-4);
	}
	return true;
}


uint32_t xst, xet;
uint32_t stop_time();

// run ticks in tick ISR
static bool issueTicks()
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
		setLed(10, true); // trigger pin

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

static void issueUnstep()
{
	THEKERNEL.getMotionControl().issueUnsteps();
}

// run the block change in this thread when signaled
static void moveCompletedThread(void const *argument)
{
	for(;;) {
		// wait until we have something to process
		uint32_t ulNotifiedValue= 0; // ulTaskNotifyTake( pdTRUE, portMAX_DELAY);
		if(ulNotifiedValue > 0) {
			// get next block, and setup the next move
			executeNextBlock();

			if(!move_issued) {
				// no moves were setup so disable the waiting tick count
				// if a move was issued then waiting_ticks would have been keeping count of how many we missed
				waiting_ticks= 0;
			}
			setLed(10, false); // trigger pin low
		}
	}
}
/*
uint32_t xst, xet;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// this is the step ticker
	if(htim->Instance == STEPTICKER_TIMx) {
		// handle stepticker
		xst= start_time();
		if(!issueTicks()) {
			// signal the next block to start, handled in moveCompletedThread
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			vTaskNotifyGiveFromISR( moveCompletedThreadHandle, &xHigherPriorityTaskWoken );
			portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		}

	}else if(htim->Instance == UNSTEPTICKER_TIMx) {
		// handle unstep ticker
		issueUnstep();
		// stop the timer
		HAL_TIM_Base_Stop_IT(&UnStepTickerTimHandle);
		// reset the count for next time
		__HAL_TIM_SET_COUNTER(&UnStepTickerTimHandle, 0);
		// Or this apparently will stop interrupts and reset counter
		//UnStepTickerTimHandle.Instance->CR1 |= (TIM_CR1_UDIS);
		//UnStepTickerTimHandle.Instance->EGR |= (TIM_EGR_UG);
	}
}

// this will start the unstep ticker
void startUnstepTicker()
{
	HAL_TIM_Base_Start_IT(&UnStepTickerTimHandle);
	// Or this apparently will restart interrupts
	//UnStepTickerTimHandle.Instance->CR1 &= ~(TIM_CR1_UDIS);
}

*/
#include "Firmware/Kernel.h"
#include "Firmware/GCodeProcessor.h"
#include "Firmware/Dispatcher.h"
#include "Firmware/GCode.h"
#include "Firmware/MotionControl.h"
#include "Firmware/Block.h"
#include "Firmware/Planner.h"
#include "Firmware/Actuator.h"

#include "mbed.h"
#include "rtos.h"

#include "Lock.h"

#include "mri.h"

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


