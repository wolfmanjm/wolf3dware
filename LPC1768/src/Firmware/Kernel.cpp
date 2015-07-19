#include "Kernel.h"
#include "MotionControl.h"
#include "Planner.h"
#include "GCodeProcessor.h"

Kernel::Kernel()
{
	motion_control= new MotionControl;
	planner= new Planner();
	gcode_processor= new GCodeProcessor();
	// clear functions
	for (int i = 0; i < N_HAL_FUNCTIONS; ++i) {
		hal_functions[i]= nullptr;
	}
}

Kernel::~Kernel()
{
	delete gcode_processor;
	delete planner;
	delete motion_control;
}

void Kernel::initialize()
{
	if(!initialized) {
		if(hal_functions[NV_INIT]) hal_functions[NV_INIT](nullptr, 0, 0);
		motion_control->initialize();
		planner->initialize();
		initialized= true;
	}
}
