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
			REQUIRE(i.has_g());
			REQUIRE(i.get_code() == 0);
			REQUIRE(i.has_arg('X')); REQUIRE(i.get_arg('X') == 1);
			REQUIRE(i.has_arg('Y')); REQUIRE(i.get_arg('Y') == 2);
			REQUIRE_FALSE(i.has_arg('Z'));
			REQUIRE_THROWS(	i.get_arg('Z') );
		}
	}

	SECTION( "Multiple commands on line and no spaces" ) {
		const char *gc= "M123X1Y2G1X10Y20Z0.634";
		INFO( "gcode is " << gc );
		auto gcodes= gp.parse(gc);
		REQUIRE( gcodes.size() == 2);
		auto a= gcodes[0];
		INFO( "gcode[0] is " << a);
		REQUIRE(a.has_m());
		REQUIRE(a.get_code() == 123);
		REQUIRE(a.has_arg('X')); REQUIRE(a.get_arg('X') == 1);
		REQUIRE(a.has_arg('Y')); REQUIRE(a.get_arg('Y') == 2);
		auto b= gcodes[1];
		INFO( "gcode[1] is " << b);
		REQUIRE(b.has_g());
		REQUIRE(b.get_code() == 1);
		REQUIRE(b.has_arg('X')); REQUIRE(b.get_arg('X') == 10);
		REQUIRE(b.has_arg('Y')); REQUIRE(b.get_arg('Y') == 20);
		REQUIRE(b.has_arg('Z')); REQUIRE(b.get_arg('Z') == 0.634f);
	}

	SECTION( "Modal G1 and comments" ) {
		gp.parse("G1 X0");
		const char *gc= "( this is a comment )X100Y200 ; G23 X0";
		INFO( "gcode is " << gc );
		auto gcodes= gp.parse(gc);
		REQUIRE( gcodes.size() == 1);
		auto a = gcodes[0];
		INFO( "gcode[0] is " << a);
		REQUIRE(a.has_g());
		REQUIRE(a.get_code() == 1);
		REQUIRE(a.has_arg('X'));
		REQUIRE(a.has_arg('Y'));
		REQUIRE_FALSE(a.has_arg('Z'));
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
	THEDISPATCHER.clear_handlers();
	THEDISPATCHER.add_handler(Dispatcher::GCODE_HANDLER, 1, fnc1);
	THEDISPATCHER.add_handler(Dispatcher::MCODE_HANDLER, 1, fnc2);
	auto h3= THEDISPATCHER.add_handler(Dispatcher::GCODE_HANDLER, 1, fnc3);

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
		THEDISPATCHER.remove_handler(Dispatcher::GCODE_HANDLER, h3);
		REQUIRE( THEDISPATCHER.dispatch(gcodes[0]) );
		REQUIRE ( cb1 );
		REQUIRE_FALSE ( cb3 );
	}
}


TEST_CASE( "Motion Control", "[MotionControl]" ) {
	MotionControl mc;
	mc.initialize();
	GCodeProcessor gp;
	GCodeProcessor::GCodes_t gcodes= gp.parse("G1 X1 Y2 G92 G92 X0");
	for(auto i : gcodes) {
		THEDISPATCHER.dispatch(i);
	}
}
