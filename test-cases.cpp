#include "GCodeProcessor.h"
#include "Dispatcher.h"
#include "GCode.h"
#include "MotionControl.h"

#include <map>
#include <vector>
#include <iostream>
#include <stdint.h>
#include <ctype.h>
#include <cmath>
#include <assert.h>

using namespace std;

#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "catch.hpp"

TEST_CASE( "Process and parse gcodes", "[GCodeProcessor]" ) {
	GCodeProcessor gp;

	SECTION( "Single GCode with parameters" ) {
		const char *gc= "G0 X1 Y2";
		INFO( "gcode is " << gc );
		GCodeProcessor::GCodes_t gcodes= gp.parse(gc);
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
		auto gcodes= gp.parse(gc);
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
		gp.parse("G1 X0");
		const char *gc= "( this is a comment )X100Y200 ; G23 X0";
		INFO( "gcode is " << gc );
		auto gcodes= gp.parse(gc);
		REQUIRE( gcodes.size() == 1);
		auto a = gcodes[0];
		INFO( "gcode[0] is " << a);
		REQUIRE(a.hasG());
		REQUIRE(a.getCode() == 1);
		REQUIRE(a.hasArg('X'));
		REQUIRE(a.hasArg('Y'));
		REQUIRE_FALSE(a.hasArg('Z'));
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
	GCodeProcessor::GCodes_t gcodes= gp.parse("G1 X1 Y2 M1 G4 S10");
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


TEST_CASE( "Motion Control", "[MotionControl]" ) {
	MotionControl mc;
	mc.initialize();
	GCodeProcessor gp;
	GCodeProcessor::GCodes_t gcodes= gp.parse("G1 X1 Y2 F6000 G0 X1 G92 G1 X1 G91 G1 X1 G1 X1 G92 G1 X0");
	for(auto i : gcodes) {
		THEDISPATCHER.dispatch(i);
	}
}
