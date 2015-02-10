#include "Kernel.h"
#include "MotionControl.h"
#include "Planner.h"
#include "GCodeProcessor.h"

Kernel::Kernel()
{
	motion_control= new MotionControl;
	motion_control->initialize();
	planner= new Planner();
	gcode_processor= new GCodeProcessor();
}

Kernel::~Kernel()
{
	delete gcode_processor;
	delete planner;
	delete motion_control;
}
