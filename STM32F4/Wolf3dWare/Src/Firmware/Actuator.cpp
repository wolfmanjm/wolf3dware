#include "Actuator.h"
#include "Block.h"

#include <cmath>
#include <iostream>

// static instance shared by all Actuators, saves memory
Block Actuator::current_block;

// Note Actuator::setCurerntBlock() must be called before this gets called
void Actuator::move( bool direction, uint32_t steps_to_move, float axis_ratio)
{
    this->steps_to_move = steps_to_move;
    // set direction pin
    this->direction = direction;
    // enable the stepper motor
    if(!enabled) enable(true);
    // set the actual direction pin now so it has lots of time before the first step pulse
    hal_functions[SET_DIR](direction);

    // need to scale by the axis ratio
    this->axis_ratio = axis_ratio;

    next_accel_event = current_block.total_move_ticks + 1;  // Do nothing by default ( cruising/plateau )
    acceleration_change = 0;
    if(current_block.accelerate_until != 0) { // If the next accel event is the end of accel
        next_accel_event = current_block.accelerate_until;
        acceleration_change = current_block.acceleration_per_tick;

    }else if(current_block.decelerate_after == 0 /*&& current_block.accelerate_until == 0*/) {
        // we start off decelerating
        acceleration_change = -current_block.deceleration_per_tick;

    }else if(current_block.decelerate_after != current_block.total_move_ticks /*&& current_block.accelerate_until == 0*/) {
        // If the next event is the start of decel ( don't set this if the next accel event is accel end )
        next_accel_event = current_block.decelerate_after;
    }

    acceleration_change *= axis_ratio;
    steps_per_tick = (current_block.initial_rate * axis_ratio) / STEP_TICKER_FREQUENCY; // steps/sec / tick frequency to get steps per tick
    counter = 0.0F;
    step_count = 0;
    moving= true;
}

bool Actuator::checkMaxSpeed()
{
    float step_freq= max_speed * steps_per_mm;
    if(step_freq > STEP_TICKER_FREQUENCY) {
        max_speed= floorf(STEP_TICKER_FREQUENCY / steps_per_mm);
        return false;
    }
    return true;
}

// returns steps to given target in mm, and sets the milestone for steps
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

// called by step ticker at 100KHz (or faster)
// Runs in ISR context, so NO memory allocation allowed
bool Actuator::tick(uint32_t current_tick)
{
    if(!moving) return false;

    steps_per_tick += acceleration_change;

    if(current_tick == next_accel_event) {
        if(current_tick == current_block.accelerate_until) { // We are done accelerating, deceleration becomes 0 : plateau
            acceleration_change = 0;
            if(current_block.decelerate_after < current_block.total_move_ticks) {
                next_accel_event = current_block.decelerate_after;
                if(current_tick != current_block.decelerate_after) { // We start decelerating
                    steps_per_tick = (axis_ratio * current_block.maximum_rate) / STEP_TICKER_FREQUENCY; // steps/sec / tick frequency to get steps per tick
                }
            }
        }

        if(current_tick == current_block.decelerate_after) { // We start decelerating
            acceleration_change = -current_block.deceleration_per_tick * axis_ratio;
        }
    }

    // protect against rounding errors and such
    if(steps_per_tick <= 0) {
        counter = 1.0F; // we complete this step
        steps_per_tick = 0;
    }

    counter += steps_per_tick;

    if(counter >= 1.0F) { // step time
        counter -= 1.0F;
        ++step_count;

        // std::cout << axis << " Step: " << step_count << " " <<  current_tick << "\n";
        step();

        if(step_count == steps_to_move) {
            moving= false;
            return false;
        }
    }
    return true;
}

void Actuator::enable(bool on)
{
    hal_functions[SET_ENABLE](on);
    enabled= on;
}

void Actuator::step()
{
    // issue step pulse
    hal_functions[SET_STEP](true);

    // keep track of real time position in steps
    uint32_t dir= direction?1:-1;
    current_step_position += dir;

    stepped= true;
}

void Actuator::unstep()
{
    // reset the step pulse if it stepped
    // currently takes about 1us from the set
    if(stepped) {
        hal_functions[SET_STEP](false);
        stepped= false;
    }
}
