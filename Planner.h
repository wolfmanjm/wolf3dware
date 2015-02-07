#pragma once

#include "Block.h"

#include <stdint.h>
#include <deque>

class GCode;
class MotionControl;
class Actuator;

class Planner
{
public:
	Planner(const MotionControl& mc) : motion_control(mc) {};
	~Planner(){};
	bool plan(const float *last_target, const float *target, int n_axis, Actuator *actuators, float rate_mms);
	void dump(std::ostream& o) const;

private:
	void calculateTrapezoid(Block& block, float entryspeed, float exitspeed);
  	float maxAllowableSpeed( float acceleration, float target_velocity, float distance) const;
	float maxExitSpeed(const Block& b) const;
	float reversePass(Block &b, float exit_speed);
	float forwardPass(Block &b, float prev_max_exit_speed);
    void recalculate();

	const MotionControl& motion_control;

	std::deque<Block> block_queue;

    float previous_unit_vec[3];

    float acceleration{2000};
    float z_acceleration{100};
    float junction_deviation{0.05F};
    float z_junction_deviation{0};
    float minimum_planner_speed{0};
};
