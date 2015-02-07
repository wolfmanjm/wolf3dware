#include "Actuator.h"
#include "Block.h"

#include <cmath>

void Actuator::move( bool direction, uint32_t steps_to_move, const Block& block )
{
    // set direction pin
    this->direction = direction;

    next_accel_change = steps_to_move + 1;  // Do nothing by default ( cruising/plateau )
    acceleration_change = 0;
    if(block.accelerate_until != 0) { // If the next accel event is the end of accel
        next_accel_event = block.accelerate_until;
        acceleration_change = block.acceleration_per_tick;
    }

    if(block.decelerate_after != block.total_move_ticks && block.accelerate_until == 0) { // If the next accel even is the start of decel ( don't set this if the next accel event is accel end )
        next_accel_event = block.decelerate_after;
        acceleration_change = -block.deceleration_per_tick;
    }

    steps_per_tick = block.initial_rate / STEP_TICKER_FREQUENCY; // steps/sec / tick frequency to get steps per tick
    //puts "acceleration change: #{@acceleration_change}"
}

std::tuple<bool,uint32_t> Actuator::stepsToTarget(float target)
{
	uint32_t target_steps = lround(target * steps_per_mm);
	bool dir= (target_steps >= last_milestone_steps);
	uint32_t delta_steps;
	if(dir){
		delta_steps= target_steps - last_milestone_steps;
		last_milestone_steps += delta_steps;
	}else{
		delta_steps= last_milestone_steps - target_steps;
		last_milestone_steps -= delta_steps;
    }
	return std::make_tuple(dir, delta_steps);
}
