#pragma once

#include <stdint.h>
#include <tuple>

class Block;

class Actuator
{
public:
	Actuator(){};
	~Actuator(){};
	void move( bool direction, uint32_t steps_to_move, const Block& block );
	float mm2steps(float mm) const { return mm*steps_per_mm; }
	float steps2mm(float steps) const { return steps/steps_per_mm; }
	void setStepsPermm(float spmm) { steps_per_mm= spmm; }
	std::tuple<bool,uint32_t> stepsToTarget(float target);

private:
	uint32_t next_accel_change;
	float acceleration_change;
	uint32_t next_accel_event;
	float steps_per_tick;
	bool direction;

	float max_rate;
	float steps_per_mm;
	uint32_t last_milestone_steps{0};
};
