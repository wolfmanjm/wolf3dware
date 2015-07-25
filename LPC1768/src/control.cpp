#include "config.h"

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
static uint32_t ydelta= 0;
static Thread *moveCompletedThreadHandle= nullptr;
static void setupTimers();
static void moveCompletedThread(void const *argument);

extern void startADC();
extern void setPWM(uint8_t channel, float percent);
extern void commsDisconnect();

// external references
extern bool serial_reply(const char*, size_t);
extern void setLed(int led, bool on);

// forward references
void kickQueue();
void startUnstepTicker();

extern uint16_t* getADC(uint8_t ch);
extern uint32_t adc_t1, adc_t2;
extern uint32_t adc_actual_sample_rate;

#ifndef OVERSAMPLE_ADC
// example of reading the DMA filled ADC buffer and taking the 4 middle values as average
// This version gives about 0.2% variation
static uint16_t readADC(int channel, bool verbose= false)
{
	uint16_t *adc_buf= getADC(channel);

	// grab the dma buffer
	std::deque<uint16_t> buf(adc_buf, adc_buf+32);
	// sort
	std::sort (buf.begin(), buf.end());
	// eliminate first and last 8, and take average of mid 16
	uint16_t sum= std::accumulate(buf.begin()+8, buf.end()-8, 0);
	return roundf((float)sum / (buf.size()-16)); // return the average
}

#else

// to oversample to get 4 extra bits (16bits from 12bit ADC) you need to sample 4^4 = 256 samples,
// sum them then shift right 4 bits to get the 16bit result
#if 0
// we sort the samples and average the two sets
static uint16_t readADC(int channel, bool verbose= false)
{
	int samples= OVERSAMPLE_SAMPLES; // the number of samples required
	uint32_t acc= 0;
	uint16_t *adc_buf= getADC(channel);
	// get oversampled 16 bit value
	uint16_t a, b;
	for (int i = 0; i < samples/2; ++i) {
		// accumulate the samples
		acc += adc_buf[i];
	}
	a= acc >> OVERSAMPLE_ADC;
	for (int i = samples/2; i < samples; ++i) {
		// accumulate the samples
		acc += adc_buf[i];
	}
	b= acc >> OVERSAMPLE_ADC;

	return (a+b)/2; // return the 16bit result
}
#else
// we eliminate the top and bottom 128 after sorting
static uint16_t readADC(int channel, bool verbose= false)
{
	int samples= OVERSAMPLE_SAMPLES; // the number of samples required
	uint32_t acc= 0;
	uint16_t *adc_buf= getADC(channel);
	// sort the buffer
	std::sort(adc_buf, adc_buf+samples);
	if(verbose) {
		for (int i = 0; i < samples; ++i) {
			printf("%u, ", adc_buf[i]);
			if((i % 32) == 0) printf("\n");
		}
		printf("\n");
	}
	// eliminate the top and bottom 128 (OVERSAMPLE_SAMPLES/4)
	for (int i = 128; i < samples-128; ++i) {
		// accumulate the samples
		acc += adc_buf[i];
	}

	return acc >> OVERSAMPLE_ADC; // return the 16bit result
}
#endif
#endif
// Prepares and executes a watchdog reset for dfu or reboot
void systemReset( bool dfu )
{
    if(dfu) {
        LPC_WDT->WDCLKSEL = 0x1;                // Set CLK src to PCLK
        uint32_t clk = SystemCoreClock / 16;    // WD has a fixed /4 prescaler, PCLK default is /4
        LPC_WDT->WDTC = 1 * (float)clk;         // Reset in 1 second
        LPC_WDT->WDMOD = 0x3;                   // Enabled and Reset
        LPC_WDT->WDFEED = 0xAA;                 // Kick the dog!
        LPC_WDT->WDFEED = 0x55;
    } else {
        NVIC_SystemReset();
    }
}

void initControl()
{
	move_issued= false;
	waiting_ticks= 0;
	running= false;
	moveCompletedThreadHandle = new Thread(moveCompletedThread, nullptr, osPriorityRealtime);

	// setup tick and untick timers
	setupTimers();
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
		setLed(5, true); // play led on

	}else{

		l.unlock();
		running= false;
		setLed(5, false); // play led off
	}
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

#if 0
// for newlib
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

#else

// for newlib-nano
extern "C" uint32_t  __end__;
extern "C" uint32_t  __malloc_free_list;
extern "C" uint32_t  _sbrk(int size);
static void free_memory(std::ostringstream& oss)
{
    uint32_t chunkNumber = 1;
    // The __end__ linker symbol points to the beginning of the heap.
    uint32_t chunkCurr = (uint32_t)&__end__;
    // __malloc_free_list is the head pointer to newlib-nano's link list of free chunks.
    uint32_t freeCurr = __malloc_free_list;
    // Calling _sbrk() with 0 reserves no more memory but it returns the current top of heap.
    uint32_t heapEnd = _sbrk(0);
    // accumulate totals
    uint32_t freeSize = 0;
    uint32_t usedSize = 0;

    oss << "Used Heap Size: " << heapEnd - chunkCurr << "\n";

    // Walk through the chunks until we hit the end of the heap.
    while (chunkCurr < heapEnd) {
        // Assume the chunk is in use.  Will update later.
        int      isChunkFree = 0;
        // The first 32-bit word in a chunk is the size of the allocation.  newlib-nano over allocates by 8 bytes.
        // 4 bytes for this 32-bit chunk size and another 4 bytes to allow for 8 byte-alignment of returned pointer.
        uint32_t chunkSize = *(uint32_t *)chunkCurr;
        // The start of the next chunk is right after the end of this one.
        uint32_t chunkNext = chunkCurr + chunkSize;

        // The free list is sorted by address.
        // Check to see if we have found the next free chunk in the heap.
        if (chunkCurr == freeCurr) {
            // Chunk is free so flag it as such.
            isChunkFree = 1;
            // The second 32-bit word in a free chunk is a pointer to the next free chunk (again sorted by address).
            freeCurr = *(uint32_t *)(freeCurr + 4);
        }

        // Skip past the 32-bit size field in the chunk header.
        chunkCurr += 4;
        // 8-byte align the data pointer.
        chunkCurr = (chunkCurr + 7) & ~7;
        // newlib-nano over allocates by 8 bytes, 4 bytes for the 32-bit chunk size and another 4 bytes to allow for 8
        // byte-alignment of the returned pointer.
        chunkSize -= 8;
        // if (verbose)
        //     stream->printf("  Chunk: %lu  Address: 0x%08lX  Size: %lu  %s\n", chunkNumber, chunkCurr, chunkSize, isChunkFree ? "CHUNK FREE" : "");

        if (isChunkFree) freeSize += chunkSize;
        else usedSize += chunkSize;

        chunkCurr = chunkNext;
        chunkNumber++;
    }
    oss << "Allocated: " << usedSize << ", Free: " << freeSize << "\n";
    unsigned long m = (uint32_t)__get_MSP() - heapEnd;
    oss << "Unused Heap: " << m << " bytes\r\n";
    oss << "Stack size: " << 0x10008000 - (uint32_t)__get_MSP() << "\n";
	#if 0
    oss << "Free AHB0: " << AHB0.free();
    oss << " Free AHB1: " << AHB1.free() << "\n";
	#endif
}
#endif

// runs in the cdc thread context
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

	}else if(strcmp(line, "br") == 0) {
		__debugbreak();

	}else if(strcmp(line, "kill") == 0) {
		execute_mode= false;
		THEKERNEL.getPlanner().purge();
		THEKERNEL.getMotionControl().resetAxisPositions();
		running= false;
		setLed(5, false); // play led off
		oss << "ok\n";

	}else if(strcmp(line, "stats") == 0) {
		oss << "Worst step time: " << xdelta << "uS\n";
		oss << "Worst issuetick time: " << ydelta << "uS\n";
		oss << "worst overflow: " << overflow << " ticks\n";
		oss << "max q size: " << maxqsize << "\n";
		oss << "ok\n";

	}else if(strcmp(line, "mem") == 0) {
		free_memory(oss);

	}else if(strcmp(line, "reset") == 0) {
		sendReply("Reset in 5 seconds...\n");
		Thread::wait(5000);
		systemReset(false);

	}else if(strcmp(line, "dfu") == 0) {
		commsDisconnect();
		Thread::wait(1000);
		systemReset(true);

	}else if(strcmp(line, "adc") == 0) {
		oss << "Actual sample rate= " << adc_actual_sample_rate << "\n";
		for (int j = 0; j < 100; ++j) {
			uint16_t max= 0, min= 0xFFFF;
			for (int i = 0; i < 10; ++i) {
				getADC(255); // kicks off DMA
				Thread::wait(50); // give it some time (simulate 20hz)
				uint16_t a1= readADC(0);
				//uint16_t a2= readADC(1);
				if(a1 > max) max= a1;
				if(a1 < min) min= a1;
				// oss << "ADC0: " << a1 << ", ADC1: " << a2 << "\n";
				// oss << "took: " << adc_t2-adc_t1 << "uS\n";
				sendReply(oss.str());
				oss.str(""); oss.clear();
			}
			float d= (max-min);
			oss << "Variation: " << d << ", " << d*100/max << "%\n";
		}

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

static uint32_t xst, xet;
//static uint32_t yst, yet;
uint32_t start_time();
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
			// too many waiting ticks, we finished all the moves,
			move_issued= false;
			current_tick= 0;
			waiting_ticks= 1;
			return false;
		}
		--waiting_ticks;
	}

	// a move was issued to the actuators, tick them until all moves are done
	uint32_t yst= start_time();
	bool all_moves_finished= !mc.issueTicks(++current_tick);
		{
			uint32_t yet= stop_time();
			uint32_t d= yet-yst;
			if(d > ydelta) ydelta= d;
		}
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

// run the block change in this thread when signaled
static void moveCompletedThread(void const *argument)
{
	for(;;) {
		// wait until we have something to process
		Thread::signal_wait(0x01);

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

static void steptick_isr()
{
	xst= start_time();
	if(!issueTicks()) {
		// signal the next block to start, handled in moveCompletedThread
		moveCompletedThreadHandle->signal_set(0x01);
	}
}

static Ticker step_ticker;
static void setupTimers()
{
	uint32_t us= roundf(1000000.0F/STEP_TICKER_FREQUENCY);
    step_ticker.attach_us(steptick_isr, us);
}

static void unsteptick_isr()
{
	THEKERNEL.getMotionControl().issueUnsteps();
}

// this will start the unstep ticker
static Timeout unstep_ticker;
void startUnstepTicker()
{
    unstep_ticker.attach_us(unsteptick_isr, 1);
}


