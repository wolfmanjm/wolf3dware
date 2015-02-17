#pragma once

#include <vector>
#include <map>
#include <bitset>
#include <stdint.h>

class GCode;
class Actuator;
class Planner;
class Block;

class MotionControl
{
public:
	MotionControl();
	~MotionControl();
	void initialize();
	void resetAxisPositions();
	void resetAxisPosition(char, float);
	void addActuator(Actuator&, bool);
	char getActuatorAxis(uint8_t i) const { return actuator_axis_lut[i]; }
	uint8_t getAxisActuator(char a) const { return axis_actuator_map.at(a); }
	bool isPrimaryAxis(uint8_t i) const { return primary_axis[i]; }
	bool issueMove(const Block& block);
	bool issueTicks(uint32_t current_tick);
	bool isAnythingMoving() const { return moving_mask != 0; }
	void setNothingMoving() { moving_mask= 0; }

	Actuator& getActuator(char axis);

private:
	bool handleG0G1(GCode&);
	bool handleSettings(GCode&);
	bool handleSetAxisPosition(GCode& gc);
	bool handleSetSpeedOverride(GCode& gc);
	bool handleGetPosition(GCode& gc);
	bool handleEnable(GCode& gc);

	float toMillimeters( float value ){ return this->inch_mode ? value * 25.4F : value; }
	float fromMillimeters(float value){ return this->inch_mode ? value / 25.4F : value; }

	float seek_rate, feed_rate;
	float seconds_per_minute{60.0F};
	uint32_t moving_mask{0};

	std::vector<Actuator> actuators;
	std::map<char, uint8_t> axis_actuator_map;
	std::vector<char> actuator_axis_lut;
	std::vector<bool> primary_axis;

	std::vector<float> last_milestone;

	struct {
		bool absolute_mode:1;
		bool inch_mode:1;
	};
};
