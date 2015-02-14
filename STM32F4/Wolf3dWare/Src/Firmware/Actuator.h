#pragma once

#include "Block.h"

#include <stdint.h>
#include <tuple>
#include <functional>

class Actuator
{
public:
	Actuator(char axis) : axis(axis), moving(false), enabled(false) {};
	~Actuator(){};
	void move( bool direction, uint32_t steps_to_move, float axis_ratio, const Block& block);
	float mm2steps(float mm) const { return mm*steps_per_mm; }
	float steps2mm(float steps) const { return steps/steps_per_mm; }
	void setStepsPermm(float spmm) { steps_per_mm= spmm; }
	std::tuple<bool,uint32_t> stepsToTarget(float target);
	bool tick(uint32_t current_tick);
	void step();
	void unstep();
	char getAxis() const { return axis; }
	uint32_t getCurrentPositionInSteps() const { return current_step_position; }
	float getCurrentPositionInmm() const { return steps2mm(current_step_position); }
	void resetPositionInmm(float mm) { current_step_position= last_milestone_steps= mm2steps(mm); }
	void resetPositionInSteps(uint32_t s) { current_step_position= last_milestone_steps= s; }
	void enable(bool);

	enum HAL_FUNCTION_INDEX
	{
		SET_STEP,
		SET_DIR,
		SET_ENABLE,

		N_HAL_FUNCTIONS
	};
	using HAL_function_t = std::function<void(bool)>;
	void assignHALFunction(HAL_FUNCTION_INDEX i, HAL_function_t fnc) { hal_functions[i] = fnc; }

private:
	Block block;
	float counter;
	float acceleration_change;
	uint32_t steps_to_move;
	uint32_t step_count;
	uint32_t next_accel_event;
	float steps_per_tick;
	float axis_ratio;
	float max_rate;
	float steps_per_mm;
	uint32_t last_milestone_steps{0};
	uint32_t current_step_position{0};
	HAL_function_t hal_functions[N_HAL_FUNCTIONS] ;
	char axis;
	struct {
		bool direction:1;
		bool moving: 1;
		bool stepped:1;
		bool enabled:1;
	};
};
