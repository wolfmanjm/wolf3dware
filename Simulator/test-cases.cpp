#include "Kernel.h"
#include "GCodeProcessor.h"
#include "Dispatcher.h"
#include "GCode.h"
#include "MotionControl.h"
#include "Block.h"
#include "Planner.h"
#include "Actuator.h"

#include <map>
#include <vector>
#include <iostream>
#include <stdint.h>
#include <ctype.h>
#include <cmath>
#include <assert.h>
#include <malloc.h>

using namespace std;

#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "catch.hpp"

TEST_CASE( "Process and parse gcodes", "[GCodeProcessor]" ) {
	// malloc_stats();
	GCodeProcessor gp;

	SECTION( "Single GCode with parameters" ) {
		const char *gc= "G0 X1 Y2";
		INFO( "gcode is " << gc );
		GCodeProcessor::GCodes_t gcodes;
		bool ok= gp.parse(gc, gcodes);
		REQUIRE(ok);
		REQUIRE( gcodes.size() == 1);
		for(auto i : gcodes) {
			INFO( "dump is " << i );
			REQUIRE(i.hasG());
			REQUIRE(i.getCode() == 0);
			REQUIRE(i.hasArg('X')); REQUIRE(i.getArg('X') == 1);
			REQUIRE(i.hasArg('Y')); REQUIRE(i.getArg('Y') == 2);
			REQUIRE_FALSE(i.hasArg('Z'));
			REQUIRE_THROWS(	i.getArg('Z') );
		}
	}

	SECTION( "Multiple commands on line and no spaces" ) {
		const char *gc= "M123X1Y2G1X10Y20Z0.634";
		INFO( "gcode is " << gc );
		GCodeProcessor::GCodes_t gcodes;
		bool ok= gp.parse(gc, gcodes);
		REQUIRE(ok);
		REQUIRE( gcodes.size() == 2);
		auto a= gcodes[0];
		INFO( "gcode[0] is " << a);
		REQUIRE(a.hasM());
		REQUIRE(a.getCode() == 123);
		REQUIRE(a.hasArg('X')); REQUIRE(a.getArg('X') == 1);
		REQUIRE(a.hasArg('Y')); REQUIRE(a.getArg('Y') == 2);
		auto b= gcodes[1];
		INFO( "gcode[1] is " << b);
		REQUIRE(b.hasG());
		REQUIRE(b.getCode() == 1);
		REQUIRE(b.hasArg('X')); REQUIRE(b.getArg('X') == 10);
		REQUIRE(b.hasArg('Y')); REQUIRE(b.getArg('Y') == 20);
		REQUIRE(b.hasArg('Z')); REQUIRE(b.getArg('Z') == 0.634f);
	}

	SECTION( "Modal G1 and comments" ) {
		GCodeProcessor::GCodes_t gcodes;
		bool ok= gp.parse("G1 X0", gcodes);
		REQUIRE(ok);
		const char *gc= "( this is a comment )X100Y200 ; G23 X0";
		INFO( "gcode is " << gc );
		gcodes.clear();
		ok= gp.parse(gc, gcodes);
		REQUIRE(ok);
		REQUIRE( gcodes.size() == 1);
		auto a = gcodes[0];
		INFO( "gcode[0] is " << a);
		REQUIRE(a.hasG());
		REQUIRE(a.getCode() == 1);
		REQUIRE(a.hasArg('X'));
		REQUIRE(a.hasArg('Y'));
		REQUIRE_FALSE(a.hasArg('Z'));
	}

	SECTION( "Line numbers and checksums" ) {
		GCodeProcessor::GCodes_t gcodes;
		bool ok= gp.parse("N10 G1 X0", gcodes);
		REQUIRE_FALSE(ok);
		REQUIRE(gcodes.empty());

		gcodes.clear();
		ok= gp.parse("N10 M110*123", gcodes);
		REQUIRE(ok);
		REQUIRE(gp.getLineNumber() == 10);
		REQUIRE(gcodes.empty());

		INFO("Bad line number");
		gcodes.clear();
		ok= gp.parse("N95 G1 X-4.992 Y-14.792 F12000.000*97", gcodes);
		REQUIRE_FALSE(ok);
		REQUIRE(gcodes.empty());

		gcodes.clear();
		ok= gp.parse("N94 M110*123", gcodes);
		REQUIRE(ok);
		REQUIRE(gp.getLineNumber() == 94);
		ok= gp.parse("N95 G1 X-4.992 Y-14.792 F12000.000*97", gcodes);
		REQUIRE(ok);
		REQUIRE(gcodes.size() == 1);

		INFO("Bad checksum");
		gcodes.clear();
		ok= gp.parse("N94 M110*123", gcodes);
		REQUIRE(ok);
		REQUIRE(gp.getLineNumber() == 94);
		ok= gp.parse("N95 G1 X-4.992 Y-14.792 F12000.000*98", gcodes);
		REQUIRE_FALSE(ok);
		REQUIRE(gcodes.empty());
	}
}

bool cb1= false;
bool cb2= false;
bool cb3= false;
auto fnc1= [](GCode& gc) { INFO("G1 handler: " << gc); cb1= true; return true; };
auto fnc2= [](GCode& gc) { INFO("M1 handler: " << gc); cb2= true; return true; };
auto fnc3= [](GCode& gc) { INFO("Second G1 handler: " << gc); cb3= true; return true; };


TEST_CASE( "Dispatch GCodes", "[Dispatcher]" ) {
	GCodeProcessor gp;
	GCodeProcessor::GCodes_t gcodes;
	bool ok= gp.parse("G1 X1 Y2 M1 G4 S10", gcodes);
	REQUIRE(ok);
	REQUIRE( gcodes.size() == 3 );
	cb1= false;
	cb2= false;
	cb3= false;
	THEDISPATCHER.clearHandlers();
	THEDISPATCHER.addHandler(Dispatcher::GCODE_HANDLER, 1, fnc1);
	THEDISPATCHER.addHandler(Dispatcher::MCODE_HANDLER, 1, fnc2);
	auto h3= THEDISPATCHER.addHandler(Dispatcher::GCODE_HANDLER, 1, fnc3);

	REQUIRE_FALSE(cb1);
	REQUIRE_FALSE(cb2);
	REQUIRE_FALSE(cb3);

	SECTION( "check callbacks" ) {
		REQUIRE( THEDISPATCHER.dispatch(gcodes[0]) );
		REQUIRE ( cb1 );
		REQUIRE ( cb3 );
		REQUIRE( THEDISPATCHER.dispatch(gcodes[1]) );
		REQUIRE ( cb2 );
		REQUIRE_FALSE( THEDISPATCHER.dispatch(gcodes[2]) );
	}

	SECTION( "Remove second G1 handler" ) {
		THEDISPATCHER.removeHandler(Dispatcher::GCODE_HANDLER, h3);
		REQUIRE( THEDISPATCHER.dispatch(gcodes[0]) );
		REQUIRE ( cb1 );
		REQUIRE_FALSE ( cb3 );
	}
}



TEST_CASE( "Planning", "[planner]" ) {
	// initialize Kernel and its modules
	THEKERNEL.initialize();
	GCodeProcessor& gp= THEKERNEL.getGCodeProcessor();

	SECTION("plan one axis three moves") {
		// Parse gcode
		GCodeProcessor::GCodes_t gcodes;
		bool ok= gp.parse("G92 G1 X0.1 F6000 G1 X0.2 G1 X100", gcodes);
		REQUIRE(ok);
		REQUIRE(gcodes.size() == 4);

		// dispatch gcode to Planner
		for(auto& i : gcodes) {
			THEDISPATCHER.dispatch(i);
		}

		Planner::Queue_t& lq= THEKERNEL.getPlanner().getLookAheadQueue();
		REQUIRE(lq.size() == 1);
		Planner::Queue_t& rq= THEKERNEL.getPlanner().getReadyQueue();
		REQUIRE(rq.size() == 2);

		// dump planned block queue
		THEKERNEL.getPlanner().moveAllToReady();
		THEKERNEL.getPlanner().dump(cout);

		// iterate over block queue and check it
		REQUIRE(rq.size() == 3);
		Block block= rq.back();
		REQUIRE(block.total_move_ticks == 1000);
		REQUIRE(block.accelerate_until == 1000);
		REQUIRE(block.decelerate_after == 1000);
		REQUIRE(block.entry_speed == 0);
		REQUIRE(block.exit_speed == 20);
		rq.pop_back();

		block= rq.back();
		REQUIRE(block.total_move_ticks == 414);
		REQUIRE(block.accelerate_until == 414);
		REQUIRE(block.decelerate_after == 414);
		REQUIRE(block.entry_speed == 20);
		REQUIRE(block.exit_speed == Approx(28.2843F).epsilon(0.0001F));
		rq.pop_back();

		block= rq.back();
		REQUIRE(block.total_move_ticks == 103585);
		REQUIRE(block.accelerate_until == 3585);
		REQUIRE(block.decelerate_after == 98585);
		REQUIRE(block.entry_speed == Approx(28.2843F).epsilon(0.0001F));
		REQUIRE(block.exit_speed == 0);
		rq.pop_back();
	}

	SECTION("plan one axis") {
		// Parse gcode
		GCodeProcessor::GCodes_t gcodes;
		bool ok= gp.parse("G92 G1 X100 F6000 G1 X200 G1 X300 G1 X400 G1 X500", gcodes);
		REQUIRE(ok);

		// dispatch gcode to Planner
		for(auto i : gcodes) {
			THEDISPATCHER.dispatch(i);
		}

		// dump planned block queue
		THEKERNEL.getPlanner().dump(cout);

		const float entryspeed[]{0,100,100,100,100};
		const float exitspeed[]{100,100,100,100,0};
		int cnt= 0;

		// iterate over block queue and check it
		Planner::Queue_t& q= THEKERNEL.getPlanner().getLookAheadQueue();
		Planner::Queue_t& rq= THEKERNEL.getPlanner().getReadyQueue();
		REQUIRE(q.size() == 1);
		REQUIRE(rq.size() == 4);

		while(!rq.empty()) {
			INFO( "Cnt: " << cnt);
			Block block= rq.back();
			rq.pop_back();
			REQUIRE(block.entry_speed == entryspeed[cnt]);
			REQUIRE(block.exit_speed == exitspeed[cnt]);
			REQUIRE(block.steps_to_move[0] == 10000);
			++cnt;
		}

		Block block= q.back();
		q.pop_back();
		REQUIRE(block.entry_speed == entryspeed[cnt]);
		REQUIRE(block.exit_speed == exitspeed[cnt]);
		REQUIRE(block.steps_to_move[0] == 10000);
		++cnt;

		REQUIRE(cnt == 5);
		REQUIRE(rq.empty());
		REQUIRE(q.empty());
	}

	SECTION("plan one axis, different speeds") {
		Planner::Queue_t& q= THEKERNEL.getPlanner().getLookAheadQueue();
		Planner::Queue_t& rq= THEKERNEL.getPlanner().getReadyQueue();
		q.clear();
		rq.clear();
		// Parse gcode
		GCodeProcessor::GCodes_t gcodes;
		bool ok= gp.parse("G92 G1 X100 F6000 G1 X200 F600 G1 X300 F6000 G1 X400 F12000 G1 X500 F12000", gcodes);
		REQUIRE(ok);
		// dispatch gcode to Planner
		for(auto i : gcodes) {
			THEDISPATCHER.dispatch(i);
		}

		// dump planned block queue
		THEKERNEL.getPlanner().dump(cout);

		const float entryspeed[]{0,10,10,100,200};
		const float exitspeed[]{10,10,100,200,0};
		int cnt= 0;

		// iterate over block queue and check it
		REQUIRE(q.size() == 1);
		REQUIRE(rq.size() == 4);

		while(!rq.empty()) {
			INFO( "Block is " << cnt);
			Block block= rq.back();
			rq.pop_back();
			REQUIRE(block.entry_speed == entryspeed[cnt]);
			REQUIRE(block.exit_speed == exitspeed[cnt]);
			REQUIRE(block.steps_to_move[0] == 10000);
			++cnt;
		}

		Block block= q.back();
		q.pop_back();
		REQUIRE(block.entry_speed == entryspeed[cnt]);
		REQUIRE(block.exit_speed == exitspeed[cnt]);
		REQUIRE(block.steps_to_move[0] == 10000);
		++cnt;

		REQUIRE(cnt == 5);
		REQUIRE(q.empty());
		REQUIRE(rq.empty());
	}

	SECTION("plan one axis, very short segments") {
		// Parse gcode
		GCodeProcessor::GCodes_t gcodes;
		bool ok= gp.parse("G92 G91 G1 X0.1 F6000", gcodes);
		REQUIRE(ok);
		// dispatch gcodes to Planner
		for(auto i : gcodes) {
			THEDISPATCHER.dispatch(i);
		}
		gcodes.clear();
		const char* seg= "G1 X0.1";
		const char *last= "G1 X100 G1 X100 G90";
		for (int i = 0; i < 100; ++i) {
		    gp.parse(seg, gcodes);
			THEDISPATCHER.dispatch(gcodes[0]);
		}
		gcodes.clear();
		gp.parse(last, gcodes);
		for(auto i : gcodes) {
			THEDISPATCHER.dispatch(i);
		}

		// dump planned block queue
		THEKERNEL.getPlanner().dump(cout);

		// iterate over block queue and check it
		Planner::Queue_t& q= THEKERNEL.getPlanner().getLookAheadQueue();
		Planner::Queue_t& rq= THEKERNEL.getPlanner().getReadyQueue();
		REQUIRE(q.size() == 1);
		REQUIRE(rq.size() == 102);
		rq.clear();
		q.clear();
	}
}

TEST_CASE( "Planning and Stepping", "[stepper]" ) {
	GCodeProcessor& gp= THEKERNEL.getGCodeProcessor();
    MotionControl& mc= THEKERNEL.getMotionControl();

    // Setup pins for each Actuator
    mc.getActuator('X').assignHALFunction(Actuator::SET_STEP,   [](bool ) { /*cout << "X_StepPin::set(" << on << ")\n"; */ });
    mc.getActuator('X').assignHALFunction(Actuator::SET_DIR,    [](bool ) { /*cout << "X_DirPin::set(" << on << ")\n";  */ });
    mc.getActuator('X').assignHALFunction(Actuator::SET_ENABLE, [](bool ) { /*cout << "X_EnbPin::set(" << on << ")\n";  */ });
    mc.getActuator('Y').assignHALFunction(Actuator::SET_STEP,   [](bool ) { /*cout << "Y_StepPin::set(" << on << ")\n"; */ });
    mc.getActuator('Y').assignHALFunction(Actuator::SET_DIR,    [](bool ) { /*cout << "Y_DirPin::set(" << on << ")\n";  */ });
    mc.getActuator('Y').assignHALFunction(Actuator::SET_ENABLE, [](bool ) { /*cout << "Y_EnbPin::set(" << on << ")\n";  */ });
    mc.getActuator('Z').assignHALFunction(Actuator::SET_STEP,   [](bool ) { /*cout << "Z_StepPin::set(" << on << ")\n"; */ });
    mc.getActuator('Z').assignHALFunction(Actuator::SET_DIR,    [](bool ) { /*cout << "Z_DirPin::set(" << on << ")\n";  */ });
    mc.getActuator('Z').assignHALFunction(Actuator::SET_ENABLE, [](bool ) { /*cout << "Z_EnbPin::set(" << on << ")\n";  */ });
    mc.getActuator('E').assignHALFunction(Actuator::SET_STEP,   [](bool ) { /*cout << "E_StepPin::set(" << on << ")\n"; */ });
    mc.getActuator('E').assignHALFunction(Actuator::SET_DIR,    [](bool ) { /*cout << "E_DirPin::set(" << on << ")\n";  */ });
    mc.getActuator('E').assignHALFunction(Actuator::SET_ENABLE, [](bool ) { /*cout << "E_EnbPin::set(" << on << ")\n";  */ });

	SECTION("Generate Steps, one axis") {
		// Parse gcode
		GCodeProcessor::GCodes_t gcodes;
		bool ok= gp.parse("G92 G1 X100 F6000 G1 X200 G1 X300 G1 X400 G1 X500", gcodes);
		REQUIRE(ok);

		// dispatch gcode to MotionControl and Planner
		for(auto i : gcodes) {
			THEDISPATCHER.dispatch(i);
		}

		// dump planned block queue
		THEKERNEL.getPlanner().dump(cout);

		const Actuator& xact= THEKERNEL.getMotionControl().getActuator('X');
		const float pos[]{100,200,300,400,500};
		int cnt= 0;

		// iterate over block queue and setup steppers
		THEKERNEL.getPlanner().moveAllToReady();
		Planner::Queue_t& q= THEKERNEL.getPlanner().getReadyQueue();
		while(!q.empty()) {
			Block block= q.back();
			q.pop_back();
			INFO("Playing Block: " << block.id);
			THEKERNEL.getMotionControl().issueMove(block);
			// simulate step ticker
			uint32_t current_tick= 0;
			bool r= true;
			while(r) {
		  		++current_tick;
				r= THEKERNEL.getMotionControl().issueTicks(current_tick);
			}
			// check we got where we requested to go
			REQUIRE(xact.getCurrentPositionInmm() == pos[cnt++]);
			INFO("Done");
		}

		REQUIRE(xact.getCurrentPositionInmm() == 500);
		REQUIRE(q.empty());
	}

	SECTION( "Generate Steps, two axis" ) {
		// Parse gcode
		GCodeProcessor::GCodes_t gcodes;
		bool ok= gp.parse("G92 G1 X100 Y0 F6000 G1 X100 Y100 G1 X0 Y100 G1 X0 Y0 G1 X100 Y50", gcodes);
		REQUIRE(ok);

		// dispatch gcode to MotionControl and Planner
		for(auto i : gcodes) {
			THEDISPATCHER.dispatch(i);
		}

		// dump planned block queue
		THEKERNEL.getPlanner().dump(cout);

		const Actuator& xact= THEKERNEL.getMotionControl().getActuator('X');
		const Actuator& yact= THEKERNEL.getMotionControl().getActuator('Y');
		const float xpos[]{100,100,0,0,100};
		const float ypos[]{0,100,100,0,50};
		int cnt= 0;

		// iterate over block queue and setup steppers
		THEKERNEL.getPlanner().moveAllToReady();
		Planner::Queue_t& q= THEKERNEL.getPlanner().getReadyQueue();
		while(!q.empty()) {
			Block block= q.back();
			q.pop_back();
			INFO("Playing Block: " << block.id);
			THEKERNEL.getMotionControl().issueMove(block);
			// simulate step ticker
			uint32_t current_tick= 0;
			bool r= true;
			while(r) {
		  		++current_tick;
				r= THEKERNEL.getMotionControl().issueTicks(current_tick);
			}
			// check we got where we requested to go
			REQUIRE(xact.getCurrentPositionInmm() == xpos[cnt]);
			REQUIRE(yact.getCurrentPositionInmm() == ypos[cnt]);
			++cnt;
			INFO("Done");
		}

		REQUIRE(xact.getCurrentPositionInmm() == 100);
		REQUIRE(yact.getCurrentPositionInmm() == 50);
		REQUIRE(q.empty());
	}

	SECTION( "Generate Steps, three axis XYE" ) {

		// Parse gcode
		GCodeProcessor::GCodes_t gcodes;
		bool ok= gp.parse("G92 G1 X100 Y0 E1.0 F6000 G1 X100 Y100 E2.0 G1 X0 Y100 E3.0 G1 X0 Y0 E4.0 G1 X100 Y50 E4.75", gcodes);
		REQUIRE(ok);

		// dispatch gcode to MotionControl and Planner
		for(auto i : gcodes) {
			THEDISPATCHER.dispatch(i);
		}

		// dump planned block queue
		THEKERNEL.getPlanner().dump(cout);

		// malloc_stats();

		const Actuator& xact= THEKERNEL.getMotionControl().getActuator('X');
		const Actuator& yact= THEKERNEL.getMotionControl().getActuator('Y');
		const Actuator& eact= THEKERNEL.getMotionControl().getActuator('E');
		const float xpos[]{100,100,0,0,100};
		const float ypos[]{0,100,100,0,50};
		const float epos[]{1,2,3,4,4.75F};

		int cnt= 0;

		// iterate over block queue and setup steppers
		THEKERNEL.getPlanner().moveAllToReady();
		Planner::Queue_t& q= THEKERNEL.getPlanner().getReadyQueue();
		while(!q.empty()) {
			Block block= q.back();
			q.pop_back();
			INFO("Playing Block: " << block.id);
			THEKERNEL.getMotionControl().issueMove(block);
			// simulate step ticker
			uint32_t current_tick= 0;
			bool r= true;
			while(r) {
		  		++current_tick;
				r= THEKERNEL.getMotionControl().issueTicks(current_tick);
			}
			// check we got where we requested to go
			REQUIRE(xact.getCurrentPositionInmm() == xpos[cnt]);
			REQUIRE(yact.getCurrentPositionInmm() == ypos[cnt]);
			REQUIRE(eact.getCurrentPositionInmm() == Approx(epos[cnt]).epsilon(0.001F));
			++cnt;
			INFO("Done");
		}

		REQUIRE(xact.getCurrentPositionInmm() == 100);
		REQUIRE(yact.getCurrentPositionInmm() == 50);
		REQUIRE(eact.getCurrentPositionInmm() == Approx(4.75F).epsilon(0.001F));
	}
}
TEST_CASE( "Stream Output", "[streamoutput]" ) {
	SECTION("basic output") {
		// dispatch gcode to MotionControl and Planner
		THEDISPATCHER.dispatch('M', 220, 0);

		REQUIRE(THEDISPATCHER.getResult().size() > 0);
		std::cout << THEDISPATCHER.getResult();

		THEDISPATCHER.dispatch('M', 220, 'S', 123, 0);
		REQUIRE(THEDISPATCHER.getResult() == "ok\r\n");

		THEDISPATCHER.dispatch('M', 220, 0);
		REQUIRE(THEDISPATCHER.getResult().size() > 0);
		std::cout << THEDISPATCHER.getResult();

		THEDISPATCHER.dispatch('M', 114, 0);
		REQUIRE(THEDISPATCHER.getResult().size() > 0);
		std::cout << THEDISPATCHER.getResult();

		THEDISPATCHER.dispatch('M', 114, 1, 0);
		REQUIRE(THEDISPATCHER.getResult().size() > 0);
		std::cout << THEDISPATCHER.getResult();
	}
}

#include "RingBuffer.hpp"
TEST_CASE( "RingBuffer", "[ringbuffer]" ) {

	SECTION("Basic") {
		RingBuffer<uint8_t, 10> rb;
		REQUIRE(rb.empty());
		REQUIRE_FALSE(rb.full());
		REQUIRE(rb.put(1));
		REQUIRE_FALSE(rb.empty());
		REQUIRE_FALSE(rb.full());
		for (int i = 2; i <= 9; ++i) {
		    REQUIRE(rb.put(i));
			if(i < 9) REQUIRE_FALSE(rb.full());
			else REQUIRE(rb.full());
		}
		REQUIRE_FALSE(rb.empty());
		REQUIRE_FALSE(rb.put(10));
		REQUIRE(rb.full());

		uint8_t x;
		for (uint8_t i = 1; i <= 8; ++i) {
			REQUIRE_FALSE(rb.empty());
		    REQUIRE(rb.get(x));
			REQUIRE_FALSE(rb.full());
			REQUIRE(x == i);
		}
		REQUIRE_FALSE(rb.empty());
		REQUIRE(rb.get(x));
		REQUIRE(x == 9);
		REQUIRE(rb.empty());
	}
}

