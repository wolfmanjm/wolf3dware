#include "MotionControl.h"
#include "Kernel.h"
#include "Dispatcher.h"
#include "GCode.h"
#include "Planner.h"
#include "Actuator.h"


#include <string.h>
#include <functional>
#include <algorithm>
#include <iostream>
using namespace std;


MotionControl::MotionControl()
{
	inch_mode= false;
	absolute_mode= true;
	seek_rate= 1000;
	feed_rate= 5000;
}

MotionControl::~MotionControl()
{
}

void MotionControl::addActuator(Actuator& actuator, bool primary)
{
	int i= actuators.size();
	char axis= actuator.getAxis();
	actuators.push_back(actuator);
	actuator_axis_lut.push_back(axis);
	axis_actuator_map[axis]= i;
	last_milestone.push_back(0);
	primary_axis.push_back(primary);
}

void MotionControl::initialize()
{
	// register the gcodes this class handles
	using std::placeholders::_1;

	// G codes
	THEDISPATCHER.addHandler( Dispatcher::GCODE_HANDLER, 0,  std::bind( &MotionControl::handleG0G1, this, _1) );
	THEDISPATCHER.addHandler( Dispatcher::GCODE_HANDLER, 1,  std::bind( &MotionControl::handleG0G1, this, _1) );
	THEDISPATCHER.addHandler( Dispatcher::GCODE_HANDLER, 20, std::bind( &MotionControl::handleSettings, this, _1) );
	THEDISPATCHER.addHandler( Dispatcher::GCODE_HANDLER, 21, std::bind( &MotionControl::handleSettings, this, _1) );
	THEDISPATCHER.addHandler( Dispatcher::GCODE_HANDLER, 90, std::bind( &MotionControl::handleSettings, this, _1) );
	THEDISPATCHER.addHandler( Dispatcher::GCODE_HANDLER, 91, std::bind( &MotionControl::handleSettings, this, _1) );
	THEDISPATCHER.addHandler( Dispatcher::GCODE_HANDLER, 92, std::bind( &MotionControl::handleSetAxisPosition, this, _1) );

	// M codes
	THEDISPATCHER.addHandler( Dispatcher::MCODE_HANDLER, 17, std::bind( &MotionControl::handleEnable, this, _1) );
	THEDISPATCHER.addHandler( Dispatcher::MCODE_HANDLER, 18, std::bind( &MotionControl::handleEnable, this, _1) );
	THEDISPATCHER.addHandler( Dispatcher::MCODE_HANDLER, 84, std::bind( &MotionControl::handleEnable, this, _1) );
	THEDISPATCHER.addHandler( Dispatcher::MCODE_HANDLER, 114, std::bind( &MotionControl::handleGetPosition, this, _1) );
	THEDISPATCHER.addHandler( Dispatcher::MCODE_HANDLER, 220, std::bind( &MotionControl::handleSetSpeedOverride, this, _1) );

	Actuator xactuator('X'), yactuator('Y'), zactuator('Z'), eactuator('E');
	xactuator.setStepsPermm(100);
	yactuator.setStepsPermm(100);
	zactuator.setStepsPermm(400);
	eactuator.setStepsPermm(750);
	addActuator(xactuator, true);
	addActuator(yactuator, true);
	addActuator(zactuator, true);
	addActuator(eactuator, false);
}

bool MotionControl::handleEnable(GCode& gc)
{
	switch(gc.getCode()) {
		case 17: // all on
			for(auto& a : actuators) a.enable(true);
			break;
		case 18: // all off
			for(auto& a : actuators) a.enable(false);
				break;
		case 84:
			if(gc.getSubcode() == 0){
				for(auto& a : actuators) a.enable(false);
			}else if(gc.getSubcode() == 1){
				// selective axis off
				for(auto& args : gc.getArgs()) {
					auto a= axis_actuator_map.find(args.first);
					if(a != axis_actuator_map.end()) actuators[a->second].enable(false);
				}
			}else return false;
			break;
	}
	return true;
}

bool MotionControl::handleSetSpeedOverride(GCode& gc)
{
    if (gc.hasArg('S')) {
        float factor = gc.getArg('S');
        factor= std::max(factor, 10.0F);
        factor= std::min(factor, 1000.0F);
        seconds_per_minute = 6000.0F / factor;
    }else{
    	THEDISPATCHER.getOS().printf("Current speed override: %f\n", 6000.0F / seconds_per_minute);
    }

    return true;
}

bool MotionControl::handleG0G1(GCode& gc)
{
	const int n_axis= actuators.size();
    float target[n_axis];

	// for each axis get the target, defaults to last_target
	for (int i = 0; i < n_axis; ++i) {
		char c= actuator_axis_lut[i];
        if( gc.hasArg(c) ) {
            float d= toMillimeters(gc.getArg(c));
            // TODO absolute mode may need to be per actuator, as E can be in relative
            target[i] = (absolute_mode) ? d : last_milestone[i] + d;
        }else{
        	target[i] = last_milestone[i];
        }
    }

    if( gc.hasArg('F') ) {
    	float f= toMillimeters(gc.getArg('F'));
        if( gc.getCode() == 0 ) seek_rate = f;
        else feed_rate = f;
    }

    // submit to planner
	THEKERNEL.getPlanner().plan(last_milestone.data(), target, n_axis, actuators.data(), (gc.getCode() == 0 ? seek_rate : feed_rate) / seconds_per_minute );

	// update last_target
	std::copy(target, target+n_axis, last_milestone.begin());
 	return true;
}

bool MotionControl::handleSettings(GCode& gc)
{
	switch(gc.getCode()) {
		case 20: inch_mode = true; break;
		case 21: inch_mode = false; break;
		case 90: absolute_mode = true; break;
		case 91: absolute_mode = false; break;
        default: return false;
	}

	return true;
}


bool MotionControl::handleGetPosition(GCode& gc)
{
	bool raw= (gc.getSubcode() == 1);
  	THEDISPATCHER.getOS().printf("C: ");
	for (size_t i = 0; i < actuators.size(); ++i) {
		char c= actuator_axis_lut[i];
  	    THEDISPATCHER.getOS().printf("%c:%1.3f ", c, raw ? actuators[i].getCurrentPositionInSteps() : fromMillimeters(last_milestone[i]));
  	}
    THEDISPATCHER.getOS().setPrependOK();
    return true;
}

bool MotionControl::handleSetAxisPosition(GCode& gc)
{
	if(gc.hasNoArgs()) {
		resetAxisPositions();
    } else {
        for (auto& i : gc.getArgs()) {
            resetAxisPosition(i.first, toMillimeters(i.second));
        }
    }
    return true;
}

void MotionControl::resetAxisPositions() {
	std::fill(last_milestone.begin(), last_milestone.end(), 0.0F);
	for(auto& a : actuators) a.resetPositionInSteps(0);
}

void MotionControl::resetAxisPosition(char axis, float pos){
	auto i= axis_actuator_map.find(axis);
	if(i != axis_actuator_map.end()){
		last_milestone[i->second]= pos;
		actuators[i->second].resetPositionInmm(pos);
	}
}

// sets up each axis to move
bool MotionControl::issueMove(const Block& block)
{
	auto i= std::max_element(block.steps_to_move.begin(), block.steps_to_move.end());
	float inv= 1.0F / *i ;
	for (size_t i = 0; i < block.steps_to_move.size(); ++i) {
	    if(block.steps_to_move[i] == 0) continue;
	    //std::cout << "moving axis: " << actuators[i].getAxis() << "\n";
	    actuators[i].move(block.direction[i], block.steps_to_move[i], block.steps_to_move[i]*inv, block);
	}
	return true;
}

bool MotionControl::issueTicks(uint32_t current_tick)
{
	bool done= true;
	for (auto& a : actuators) {
		if(a.tick(current_tick)) done= false;
	}

	// this will toggle the step pin if it was set
	for (auto& a : actuators) {
		a.unstep();
	}
	return !done;
}

Actuator& MotionControl::getActuator(char axis)
{
	return actuators[getAxisActuator(axis)];
}
