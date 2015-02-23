#include "Extruder.h"
#include "../Kernel.h"
#include "../Actuator.h"
#include "../MotionControl.h"
#include "../GCode.h"
#include "../Dispatcher.h"

#include <cmath>
/*
	Handles specialized Extruder gcodes and Extruder related things
*/

void Extruder::initialize()
{
	// register gcode handlers
	// register the gcodes this class handles
	using std::placeholders::_1;

	// G codes
	THEDISPATCHER.addHandler( Dispatcher::GCODE_HANDLER,  0, std::bind( &Extruder::handleG0G1, this, _1) );
	THEDISPATCHER.addHandler( Dispatcher::GCODE_HANDLER,  1, std::bind( &Extruder::handleG0G1, this, _1) );
	THEDISPATCHER.addHandler( Dispatcher::GCODE_HANDLER, 10, std::bind( &Extruder::handleRetract, this, _1) );
	THEDISPATCHER.addHandler( Dispatcher::GCODE_HANDLER, 11, std::bind( &Extruder::handleRetract, this, _1) );

	// M codes
	THEDISPATCHER.addHandler( Dispatcher::MCODE_HANDLER, 200, std::bind( &Extruder::handleFilamentDiameter, this, _1) );
	THEDISPATCHER.addHandler( Dispatcher::MCODE_HANDLER, 207, std::bind( &Extruder::handleRetractSettings, this, _1) );
	THEDISPATCHER.addHandler( Dispatcher::MCODE_HANDLER, 208, std::bind( &Extruder::handleUnRetractSettings, this, _1) );
	THEDISPATCHER.addHandler( Dispatcher::MCODE_HANDLER, 221, std::bind( &Extruder::handleFlowRateSetting, this, _1) );

	// set up intial scale if filament diameter is set
	if(filament_diameter > 0.01F) {
		volumetric_multiplier= calculateVolumetricMultiplier(filament_diameter);
		setScale();
	}
}

float Extruder::calculateVolumetricMultiplier(float dia)
{
	if(dia > 0) {
		return(1.0F / (powf(dia / 2, 2) * (float)M_PI));
	}

	return 1.0F;
}

void Extruder::setScale()
{
	// apply to the Extruder E axis
	Actuator& a= THEKERNEL.getMotionControl().getActuator('E');
	a.setScale(volumetric_multiplier*extruder_multiplier);
}

// G0 or G1
bool Extruder::handleG0G1(GCode& gc)
{
	if(retracted && !in_retract && gc.hasArg('Z')) {
		// NOTE we cancel the zlift restore for the following G11 as we have moved to an absolute Z which we need to stay at
		// but we will get this for the Z lift the G10 issued  so make sure we have finished that
		cancel_zlift_restore= true;
	}
	return true;
}

// G10 & G11 firmware retract
bool Extruder::handleRetract(GCode& gc)
{
	in_retract= true; // marks it to detect recursion

	// check we are in the correct state of retract or unretract
	if(gc.getCode() == 10 && !retracted) {
		retracted= true;
		cancel_zlift_restore= false;

	} else if(gc.getCode() == 11 && retracted){
		retracted= false;

	} else {
		in_retract= false;
		return true; // ignore duplicates
	}

	// we inject the moves into the queue

	{
		// push state
		GCode newgc;
		newgc.setCommand('M', 120);
		THEDISPATCHER.dispatch(newgc);
	}
	{
		// set relative mode
		GCode newgc;
		newgc.setCommand('G', 91);
		THEDISPATCHER.dispatch(newgc);
	}

	// handle zlift restore which happens before the unretract
	if(retract_zlift_length > 0 && gc.getCode() == 11 && !cancel_zlift_restore) {
		// NOTE we do not do this if cancel_zlift_restore is set to true, which happens if there is an absolute Z move inbetween G10 and G11
		GCode newgc;
		newgc.setCommand('G', 0).addArg('Z', -retract_zlift_length).addArg('F', retract_zlift_feedrate);
		THEDISPATCHER.dispatch(newgc);
	}

	{
		// NOTE we want these to be in mm not mm³ so we use G0
		GCode newgc;
		if(gc.getCode() == 10) { // G10 retract
			newgc.setCommand('G', 0).addArg('E', retract_length).addArg('F', retract_feedrate);

		}else{ // G11 unretract
			newgc.setCommand('G', 0).addArg('E', -(retract_length+retract_recover_length)).addArg('F', retract_recover_feedrate);
		}
		THEDISPATCHER.dispatch(newgc);
	}

	// handle zlift which happens after retract
	if(retract_zlift_length > 0 && gc.getCode() == 10) {
		GCode newgc;
		newgc.setCommand('G', 0).addArg('Z', retract_zlift_length).addArg('F', retract_zlift_feedrate);
		THEDISPATCHER.dispatch(newgc);
	}

	{
		// pop state, restores feedrates and absolute mode
		GCode newgc;
		newgc.setCommand('M', 121);
		THEDISPATCHER.dispatch(newgc);
	}

	in_retract= false;
	return true;
}

// M200 Dnnn set filament diameter and enable volumetric extrusion
bool Extruder::handleFilamentDiameter(GCode& gc)
{
	if(gc.hasArg('D')) {
		// this waits until all the previous moves have completed
		THEKERNEL.getMotionControl().waitForMoves();

		filament_diameter = gc.getArg('D');
		if(filament_diameter > 0.01F) {
			volumetric_multiplier = calculateVolumetricMultiplier(filament_diameter);
		}else{
			volumetric_multiplier = 1.0F;
		}

		setScale();

	}else {
		if(filament_diameter > 0.01F) {
			THEDISPATCHER.getOS().printf("Filament Diameter: %f\n", filament_diameter);
		}else{
			THEDISPATCHER.getOS().printf("Volumetric extrusion is disabled\n");
		}
	}
	return true;
}

// M207 - set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop] Q[zlift feedrate mm/min]
bool Extruder::handleRetractSettings(GCode& gc)
{
	if(gc.hasNoArgs()) {
		THEDISPATCHER.getOS().printf("retract length: %1.4fmm, feedrate: %1.4fmm/sec, z lift length: %1.4fmm, zlift feedrate: %1.4fmm/sec\n",
			retract_length, retract_feedrate, retract_zlift_length, retract_zlift_feedrate);
		return true;
	}

	if(gc.hasArg('S')) retract_length = gc.getArg('S');
	if(gc.hasArg('F')) retract_feedrate = gc.getArg('F')/60.0F; // specified in mm/min converted to mm/sec
	if(gc.hasArg('Z')) retract_zlift_length = gc.getArg('Z');
	if(gc.hasArg('Q')) retract_zlift_feedrate = gc.getArg('Q');
	return true;
}

// M208 - set retract recover length S[positive mm surplus to the M207 S*] F[feedrate mm/min]
bool Extruder::handleUnRetractSettings(GCode& gc)
{
	if(gc.hasNoArgs()) {
		THEDISPATCHER.getOS().printf("recover extra length: %1.4fmm, recover feedrate: %1.4fmm/sec\n",
			retract_recover_length, retract_recover_feedrate);
		return true;
	}

	if(gc.hasArg('S')) retract_recover_length = gc.getArg('S');
	if(gc.hasArg('F')) retract_recover_feedrate = gc.getArg('F')/60.0F; // specified in mm/min converted to mm/sec
	return true;
}

// M221 S100 change flow rate by percentage
bool Extruder::handleFlowRateSetting(GCode& gc)
{
	if(gc.hasArg('S')) {
		extruder_multiplier= gc.getArg('S')/100.0F;
		setScale();

	}else{
		THEDISPATCHER.getOS().printf("Extruder Multiplier: %f\n", extruder_multiplier);
	}

	return true;
}
