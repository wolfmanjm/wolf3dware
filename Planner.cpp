#include "Planner.h"
#include "Block.h"
#include "MotionControl.h"
#include "Actuator.h"

#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <array>
#include <tuple>

bool Planner::plan(GCode &gc, const float *last_target, const float *target, int n_axis,  Actuator *actuators, float rate_mms)
{
    printf("last_target: %f,%f,%f target: %f,%f,%f rate: %f\n", last_target[0], last_target[1], last_target[2], target[0], target[1], target[2], rate_mms);

    float deltas[n_axis];
    float distance = 0.0F;
    float sos = 0.0F;
    for (int i = 0; i < n_axis; ++i) {
        deltas[i] = target[i] - last_target[i];
        if(motion_control.isPrimaryAxis(i)) {
            sos += powf(deltas[i], 1);
        }
    }
    if(sos > 0.0F) distance = sqrtf( sos );

    uint8_t xaxis= motion_control.getAxisActuator('X');
    uint8_t yaxis= motion_control.getAxisActuator('Y');
    uint8_t zaxis= motion_control.getAxisActuator('Z');
    float unit_vec[3]{deltas[xaxis]/distance, deltas[yaxis]/distance, deltas[zaxis]/distance};

        // Do not move faster than the configured cartesian limits
        // if ( max_speeds[a] > 0 ) {
        //     float axis_speed = fabsf(unit_vec[i] * rate_mms);
        //     if (axis_speed > max_speeds[a])
        //         rate_mms *= ( max_speeds[a] / axis_speed );
        // }


    // create the new block
    Block block;

    for (int i = 0; i < n_axis; i++) {
        std::tuple<bool, uint32_t> r = actuators[i].stepsToTarget(target[i]);
        block.direction[i] = std::get<0>(r);
        block.steps_to_move.push_back(labs(std::get<1>(r)));
    }

    float acceleration = this->acceleration;
    float junction_deviation = this->junction_deviation;

    // use either regular acceleration or a z only move accleration
    if(block.steps_to_move[xaxis] == 0 && block.steps_to_move[yaxis] == 0) {
        // z only move
        if(this->z_acceleration > 0.0F) acceleration = this->z_acceleration;
        if(this->z_junction_deviation >= 0.0F) junction_deviation = this->z_junction_deviation;
    }

    block.acceleration = acceleration; // save in block

    // Max number of steps, for all axes
    block.steps_event_count = std::max({block.steps_to_move[xaxis], block.steps_to_move[yaxis], block.steps_to_move[zaxis]});

    block.millimeters = distance;

    // Calculate speed in mm/sec for each axis. No divide by zero due to previous checks.
    if( distance > 0.0F ) {
        block.nominal_speed = rate_mms;           // (mm/s) Always > 0
        block.nominal_rate = ceilf(block.steps_event_count * rate_mms / distance); // (step/s) Always > 0
    } else {
        block.nominal_speed = 0.0F;
        block.nominal_rate  = 0;
    }


    // Compute maximum allowable entry speed at junction by centripetal acceleration approximation.
    // Let a circle be tangent to both previous and current path line segments, where the junction
    // deviation is defined as the distance from the junction to the closest edge of the circle,
    // colinear with the circle center. The circular segment joining the two paths represents the
    // path of centripetal acceleration. Solve for max velocity based on max acceleration about the
    // radius of the circle, defined indirectly by junction deviation. This may be also viewed as
    // path width or max_jerk in the previous grbl version. This approach does not actually deviate
    // from path, but used as a robust way to compute cornering speeds, as it takes into account the
    // nonlinearities of both the junction angle and junction velocity.

    // NOTE however it does not take into account independent axis, in most cartesian X and Y and Z are totally independent
    // and this allows one to stop with little to no decleration in many cases. This is particualrly bad on leadscrew based systems that will skip steps.
    float vmax_junction = minimum_planner_speed; // Set default max junction speed

    if (!block_queue.empty()) {
        float previous_nominal_speed = block_queue.back().nominal_speed;

        if (previous_nominal_speed > 0.0F && junction_deviation > 0.0F) {
            // Compute cosine of angle between previous and current path. (prev_unit_vec is negative)
            // NOTE: Max junction velocity is computed without sin() or acos() by trig half angle identity.
            float cos_theta = - previous_unit_vec[0] * unit_vec[0]
                              - previous_unit_vec[1] * unit_vec[1]
                              - previous_unit_vec[2] * unit_vec[2] ;

            // Skip and use default max junction speed for 0 degree acute junction.
            if (cos_theta < 0.95F) {
                vmax_junction = std::min(previous_nominal_speed, block.nominal_speed);
                // Skip and avoid divide by zero for straight junctions at 180 degrees. Limit to min() of nominal speeds.
                if (cos_theta > -0.95F) {
                    // Compute maximum junction velocity based on maximum acceleration and junction deviation
                    float sin_theta_d2 = sqrtf(0.5F * (1.0F - cos_theta)); // Trig half angle identity. Always positive.
                    vmax_junction = std::min(vmax_junction, sqrtf(acceleration * junction_deviation * sin_theta_d2 / (1.0F - sin_theta_d2)));
                }
            }
        }
    }
    block.max_entry_speed = vmax_junction;

    // Initialize block entry speed. Compute based on deceleration to user-defined minimum_planner_speed.
    float v_allowable = maxAllowableSpeed(-acceleration, minimum_planner_speed, block.millimeters);
    block.entry_speed = std::min(vmax_junction, v_allowable);


    // stick on the end of the block queue
    block_queue.push_back(block);
    return true;
}

// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the
// acceleration within the allotted distance.
float Planner::maxAllowableSpeed(float acceleration, float target_velocity, float distance)
{
    return sqrtf(target_velocity * target_velocity - 2.0F * acceleration * distance);
}

// entry speed is in mm/sec
// this code written by Arthur Wolf
void Planner::calculateTrapezoid(Block &block, float entryspeed, float exitspeed)
{
    float initial_rate = block.nominal_rate * (entryspeed / block.nominal_speed); // steps/sec
    float final_rate = block.nominal_rate * (exitspeed / block.nominal_speed);

    // How many steps ( can be fractions of steps, we need very precise values ) to accelerate and decelerate
    // This is a simplification to get rid of rate_delta and get the steps/s² accel directly from the mm/s² accel
    float acceleration_per_second = (block.acceleration * block.steps_event_count) / block.millimeters;

    float maximum_possible_rate = sqrtf( ( block.steps_event_count * acceleration_per_second ) + ( ( powf(initial_rate, 2) + powf(final_rate, 2) / 2.0F ) ) );

    //puts "maximum_possible_rate: #{block.maximum_possible_rate} steps/sec, #{maximum_possible_rate/STEPS_PER_MM} mm/sec"

    // Now this is the maximum rate we'll achieve this move, either because
    // it's the higher we can achieve, or because it's the higher we are
    // allowed to achieve
    block.maximum_rate = std::min(maximum_possible_rate, block.nominal_rate);

    // Now figure out how long it takes to accelerate in seconds
    float time_to_accelerate = ( block.maximum_rate - initial_rate ) / acceleration_per_second;

    // Now figure out how long it takes to decelerate
    float time_to_decelerate = ( final_rate -  block.maximum_rate ) / -acceleration_per_second;

    // Now we know how long it takes to accelerate and decelerate, but we must
    // also know how long the entire move takes so we can figure out how long
    // is the plateau if there is one
    float plateau_time = 0;

    // Only if there is actually a plateau ( we are limited by nominal_rate )
    if(maximum_possible_rate > block.nominal_rate) {
        // Figure out the acceleration and deceleration distances ( in steps )
        float acceleration_distance = ( ( initial_rate + block.maximum_rate ) / 2.0F ) * time_to_accelerate;
        float deceleration_distance = ( ( block.maximum_rate + final_rate ) / 2.0F ) * time_to_decelerate;

        // Figure out the plateau steps
        float plateau_distance = block.steps_event_count - acceleration_distance - deceleration_distance;

        // Figure out the plateau time in seconds
        plateau_time = plateau_distance / block.maximum_rate;
    }

    // Figure out how long the move takes total ( in seconds )
    float total_move_time = time_to_accelerate + time_to_decelerate + plateau_time;
    //puts "total move time: #{total_move_time}s time to accelerate: #{time_to_accelerate}, time to decelerate: #{time_to_decelerate}"

    // We now have the full timing for acceleration, plateau and deceleration,
    // yay \o/ Now this is very important these are in seconds, and we need to
    // round them into ticks. This means instead of accelerating in 100.23
    // ticks we'll accelerate in 100 ticks. Which means to reach the exact
    // speed we want to reach, we must figure out a new/slightly different
    // acceleration/deceleration to be sure we accelerate and decelerate at
    // the exact rate we want

    // First off round total time, acceleration time and deceleration time in ticks
    uint32_t acceleration_ticks = floorf( time_to_accelerate * STEP_TICKER_FREQUENCY );
    uint32_t deceleration_ticks = floorf( time_to_decelerate * STEP_TICKER_FREQUENCY );
    uint32_t total_move_ticks   = floorf( total_move_time    * STEP_TICKER_FREQUENCY );

    // Now deduce the plateau time for those new values expressed in tick
    //uint32_t plateau_ticks = total_move_ticks - acceleration_ticks - deceleration_ticks;

    // Now we figure out the acceleration value to reach EXACTLY maximum_rate(steps/s) in EXACTLY acceleration_ticks(ticks) amount of time in seconds
    float acceleration_time = acceleration_ticks / STEP_TICKER_FREQUENCY;  // This can be moved into the operation bellow, separated for clarity, note :Â we need to do this instead of using time_to_accelerate(seconds) directly because time_to_accelerate(seconds) and acceleration_ticks(seconds) do not have the same value anymore due to the rounding
    float deceleration_time = deceleration_ticks / STEP_TICKER_FREQUENCY;

    float  acceleration_in_steps = ( block.maximum_rate - initial_rate ) / acceleration_time;
    float deceleration_in_steps = ( block.maximum_rate - final_rate ) / deceleration_time;

    // Note we use this value for acceleration as well as for deceleration, if that doesn't work, we can also as well compute the deceleration value this way :
    // float deceleration(steps/sÂ²) = ( final_rate(steps/s) - maximum_rate(steps/s) ) / acceleration_time(s);
    // and store that in the block and use it for deceleration, which -will- yield better results, but may not be useful. If the moves do not end correctly, try computing this value, adding it to the block, and then using it for deceleration in the step generator

    // Now figure out the two acceleration ramp change events in ticks
    block.accelerate_until = acceleration_ticks;
    block.decelerate_after = total_move_ticks - deceleration_ticks;

    // Now figure out the acceleration PER TICK, this should ideally be held as a float, even a double if possible as it's very critical to the block timing
    // steps/tick^2

    block.acceleration_per_tick =  acceleration_in_steps / STEP_TICKER_FREQUENCY_2;
    block.deceleration_per_tick = deceleration_in_steps / STEP_TICKER_FREQUENCY_2;

    // We now have everything we need for this block to call a Steppermotor->move method !!!!
    // Theorically, if accel is done per tick, the speed curve should be perfect.

    // We need this to call move()
    block.total_move_ticks = total_move_ticks;

    //puts "accelerate_until: #{block.accelerate_until}, decelerate_after: #{block.decelerate_after}, acceleration_per_tick: #{block.acceleration_per_tick}, total_move_ticks: #{block.total_move_ticks}"

    block.initial_rate = initial_rate;
}

