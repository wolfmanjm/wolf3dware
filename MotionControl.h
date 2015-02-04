#pragma once

#include <vector>
#include <map>
#include <stdint.h>

class GCode;
class Actuator;

class MotionControl
{
public:
	MotionControl();
	~MotionControl(){};
	void initialize();
	void reset_axis_positions();
	void reset_axis_position(float, char);
	void add_actuator(Actuator *, char);

private:
	bool handle_g0_g1(GCode&);
	bool handle_settings(GCode&);
	float to_millimeters( float value ){ return this->inch_mode ? value * 25.4F : value; }
	float from_millimeters( float value){ return this->inch_mode ? value/25.4F : value; }

	std::vector<Actuator*> actuators;
	std::map<char, uint8_t> axis_actuator_map;
	std::vector<char> actuator_axis_lut;

	struct {
		bool absolute_mode:1;
		bool inch_mode:1;
	};
};
