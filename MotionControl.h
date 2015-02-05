#pragma once

#include <vector>
#include <map>
#include <stdint.h>

class GCode;
class Actuator;
class Planner;

class MotionControl
{
public:
	MotionControl();
	~MotionControl();
	void initialize();
	void resetAxisPositions();
	void resetAxisPosition(char, float);
	void addActuator(Actuator *, char);

private:
	bool handleG0G1(GCode&);
	bool handleSettings(GCode&);
	bool handleSetAxisPosition(GCode& gc);
	bool handleSetSpeedOverride(GCode& gc);

	float toMillimeters( float value ){ return this->inch_mode ? value * 25.4F : value; }
	float fromMillimeters( float value){ return this->inch_mode ? value/25.4F : value; }

	float seek_rate, feed_rate;
	float seconds_per_minute{0.0F};

	Planner *planner;

	std::vector<Actuator*> actuators;
	std::map<char, uint8_t> axis_actuator_map;
	std::vector<char> actuator_axis_lut;
	std::vector<float> last_milestone;

	struct {
		bool absolute_mode:1;
		bool inch_mode:1;
	};
};
