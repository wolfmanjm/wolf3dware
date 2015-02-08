#include "Actuator.h"
#include "Block.h"

#include <cmath>
#include <iostream>

void Actuator::move( bool direction, uint32_t steps_to_move, float axis_ratio, const Block &block )
{
    this->block = block; // take a copy if this block
    this->steps_to_move = steps_to_move;
    // set direction pin
    this->direction = direction;
    // need to scale by the axis ratio
    this->axis_ratio = axis_ratio;

    next_accel_event = block.total_move_ticks + 1;  // Do nothing by default ( cruising/plateau )
    acceleration_change = 0;
    if(block.accelerate_until != 0) { // If the next accel event is the end of accel
        next_accel_event = block.accelerate_until;
        acceleration_change = block.acceleration_per_tick;

    }else if(block.decelerate_after == 0 /*&& block.accelerate_until == 0*/) {
        // we start off decelerating
        acceleration_change = -block.deceleration_per_tick;

    }else if(block.decelerate_after != block.total_move_ticks /*&& block.accelerate_until == 0*/) {
        // If the next event is the start of decel ( don't set this if the next accel event is accel end )
        next_accel_event = block.decelerate_after;
    }

    acceleration_change *= axis_ratio;
    steps_per_tick = (block.initial_rate * axis_ratio) / STEP_TICKER_FREQUENCY; // steps/sec / tick frequency to get steps per tick
    //puts "acceleration change: #{@acceleration_change}"
    counter = 0.0F;
    step_count = 0;
    moving= true;
}

std::tuple<bool, uint32_t> Actuator::stepsToTarget(float target)
{
    uint32_t target_steps = lround(target * steps_per_mm);
    bool dir = (target_steps >= last_milestone_steps);
    uint32_t delta_steps;
    if(dir) {
        delta_steps = target_steps - last_milestone_steps;
        last_milestone_steps += delta_steps;
    } else {
        delta_steps = last_milestone_steps - target_steps;
        last_milestone_steps -= delta_steps;
    }
    return std::make_tuple(dir, delta_steps);
}

bool Actuator::tick(uint32_t current_tick)
{
    if(!moving) return false;

    steps_per_tick += acceleration_change;

    if(current_tick == next_accel_event) {
        if(current_tick == block.accelerate_until) { // We are done accelerating, decceleration becomes 0 : plateau
            acceleration_change = 0;
            if(block.decelerate_after < block.total_move_ticks) {
                next_accel_event = block.decelerate_after;
                if(current_tick != block.decelerate_after) { // We start decelerating
                    steps_per_tick = (axis_ratio * block.maximum_rate) / STEP_TICKER_FREQUENCY; // steps/sec / tick frequency to get steps per tick
                }
            }
        }

        if(current_tick == block.decelerate_after) { // We start decelerating
            acceleration_change = -block.deceleration_per_tick * axis_ratio;
        }
    }

    // protect against rounding errors and such
    if(steps_per_tick <= 0) {
        // if @counter < 0.9
        //   puts "#{@axis} - ERROR: finished too fast still have #{@steps_to_move-@current_step} steps to go at tick: #{current_tick}, steps_per_tick: #{@steps_per_tick}, counter: #{@counter}"
        //   return false
        // end
        counter = 1.0F; // we complete this step
        steps_per_tick = 0;
    }

    counter += steps_per_tick;

    if(counter >= 1.0F) { // step time
        counter -= 1.0F;
        ++step_count;
        //puts "#{@axis} - #{current_tick}: Do STEP #{@current_step}, steps_per_tick: #{@steps_per_tick}, #{@steps_per_tick*STEP_TICKER_FREQUENCY/STEPS_PER_MM} mm/sec"

        // std::cout << axis << " Step: " << step_count << " " <<  current_tick << "\n";
        step();

        if(step_count == steps_to_move) {
            //puts "#{@axis} - stepping finished"
            moving= false;
            return false;
        }
    }
    return true;
}

void Actuator::step()
{
    // TODO issue step pulse

    // keep track of real time position in steps
    uint32_t dir= direction?1:-1;
    current_step_position += dir;
}
