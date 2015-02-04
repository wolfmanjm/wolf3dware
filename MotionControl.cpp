#include "MotionControl.h"
#include "Dispatcher.h"
#include "GCode.h"

#include <functional>
#include <iostream>
using namespace std;


MotionControl::MotionControl()
{
	inch_mode= false;
	absolute_mode= true;
}

void MotionControl::add_actuator(Actuator *actuator, char axis)
{
	int i= actuators.size();
	actuators.push_back(actuator);
	actuator_axis_lut.push_back(axis);
	axis_actuator_map[axis]= i;
}

void MotionControl::initialize()
{
	// register the gcodes this class handles
	using std::placeholders::_1;
	THEDISPATCHER.add_handler( Dispatcher::GCODE_HANDLER, 0, std::bind( &MotionControl::handle_g0_g1, this, _1) );
	THEDISPATCHER.add_handler( Dispatcher::GCODE_HANDLER, 1, std::bind( &MotionControl::handle_g0_g1, this, _1) );
	THEDISPATCHER.add_handler( Dispatcher::GCODE_HANDLER, 20, std::bind( &MotionControl::handle_settings, this, _1) );
	THEDISPATCHER.add_handler( Dispatcher::GCODE_HANDLER, 21, std::bind( &MotionControl::handle_settings, this, _1) );
	THEDISPATCHER.add_handler( Dispatcher::GCODE_HANDLER, 90, std::bind( &MotionControl::handle_settings, this, _1) );
	THEDISPATCHER.add_handler( Dispatcher::GCODE_HANDLER, 91, std::bind( &MotionControl::handle_settings, this, _1) );
	THEDISPATCHER.add_handler( Dispatcher::GCODE_HANDLER, 92, std::bind( &MotionControl::handle_settings, this, _1) );
}
bool MotionControl::handle_g0_g1(GCode& gc)
{
	cout << "MotionControl::handle_g0_g1 " << gc << endl;
	return true;
}

bool MotionControl::handle_settings(GCode& gc)
{
	switch(gc.get_code()) {
		case 20: this->inch_mode = true; break;
		case 21: this->inch_mode = false; break;
		case 90: this->absolute_mode = true; break;
		case 91: this->absolute_mode = false; break;
		case 92:
			if(gc.has_no_args()) {
				reset_axis_positions();
            } else {
                for (auto& i : gc.get_args()) {
		            reset_axis_position(this->to_millimeters(i.second), i.first);
                }
            }
            break;
        default: return false;
	}

	return true;
}

void MotionControl::reset_axis_positions() {
	cout << "MotionControl::reset_axis_positions" << endl;
	// for(auto i : actuators) {
	// 	i->set_position(0.0F);
	// }
}

void MotionControl::reset_axis_position(float pos, char axis){
	cout << "MotionControl::reset_axis_position " << axis << " " << pos << endl;
	// auto i= axis_actuator_map.find(axis);
	// if(i != axis_actuator_map.end()) actuators[i->second]->set_position(0.0F);
}
