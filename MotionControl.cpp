#include "MotionControl.h"
#include "Dispatcher.h"
#include "GCode.h"
#include "Planner.h"

#include <string.h>
#include <functional>
#include <iostream>
using namespace std;


MotionControl::MotionControl()
{
	inch_mode= false;
	absolute_mode= true;
	planner= new Planner();
}

MotionControl::~MotionControl()
{
	delete planner;
}

void MotionControl::addActuator(Actuator *actuator, char axis)
{
	int i= actuators.size();
	actuators.push_back(actuator);
	actuator_axis_lut.push_back(axis);
	axis_actuator_map[axis]= i;
	last_milestone.push_back(0);
}

void MotionControl::initialize()
{
	// register the gcodes this class handles
	using std::placeholders::_1;
	THEDISPATCHER.addHandler( Dispatcher::GCODE_HANDLER, 0, std::bind( &MotionControl::handleG0G1, this, _1) );
	THEDISPATCHER.addHandler( Dispatcher::GCODE_HANDLER, 1, std::bind( &MotionControl::handleG0G1, this, _1) );
	THEDISPATCHER.addHandler( Dispatcher::GCODE_HANDLER, 20, std::bind( &MotionControl::handleSettings, this, _1) );
	THEDISPATCHER.addHandler( Dispatcher::GCODE_HANDLER, 21, std::bind( &MotionControl::handleSettings, this, _1) );
	THEDISPATCHER.addHandler( Dispatcher::GCODE_HANDLER, 90, std::bind( &MotionControl::handleSettings, this, _1) );
	THEDISPATCHER.addHandler( Dispatcher::GCODE_HANDLER, 91, std::bind( &MotionControl::handleSettings, this, _1) );
	THEDISPATCHER.addHandler( Dispatcher::GCODE_HANDLER, 92, std::bind( &MotionControl::handleSetAxisPosition, this, _1) );

	THEDISPATCHER.addHandler( Dispatcher::MCODE_HANDLER, 220, std::bind( &MotionControl::handleSetSpeedOverride, this, _1) );
}

bool MotionControl::handleSetSpeedOverride(GCode& gc)
{
    if (gc.hasArg('S')) {
        float factor = gc.getArg('S');
        factor= std::min(factor, 10.0F);
        factor= std::max(factor, 1000.0F);
        seconds_per_minute = 6000.0F / factor;
    }else{
    	// printf("Current speed override: %f\n", 6000.0F / seconds_per_minute);
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
	planner->plan(gc, last_milestone.data(), target, n_axis, (gc.getCode() == 0 ? seek_rate : feed_rate) / seconds_per_minute );

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
}

void MotionControl::resetAxisPosition(char axis, float pos){
	auto i= axis_actuator_map.find(axis);
	if(i != axis_actuator_map.end()) last_milestone[i->second]= pos;
}
