/**
	Most of the Planner is taken direct from Smoothie,with inlky minor changes
	https://github.com/Smoothieware/Smoothieware/blob/edge/src/modules/robot/Planner.cpp

	calculateTrapezoid() was designed by Arthur Wolf for Smoothie
	https://gist.github.com/arthurwolf/ed8cc3bce15ac395d8e7
*/
#include "Kernel.h"
#include "Planner.h"
#include "Dispatcher.h"
#include "Block.h"
#include "GCode.h"
#include "MotionControl.h"
#include "Actuator.h"
#include "Lock.h"

#include <math.h>
#include <algorithm>
#include <array>
#include <tuple>
#include <iostream>
#include <string.h>

Planner::Planner()
{
	reset();
}

void Planner::reset()
{
	memset(previous_unit_vec, 0, sizeof(previous_unit_vec));
}

void Planner::initialize()
{
	// register the gcodes this class handles
	using std::placeholders::_1;

	// G codes

	// M codes
	THEDISPATCHER.addHandler( Dispatcher::MCODE_HANDLER, 204, std::bind( &Planner::handleConfigurations, this, _1) );
	THEDISPATCHER.addHandler( Dispatcher::MCODE_HANDLER, 205, std::bind( &Planner::handleConfigurations, this, _1) );
	THEDISPATCHER.addHandler( Dispatcher::MCODE_HANDLER, 500, std::bind( &Planner::handleSaveConfiguration, this, _1) );
}

// check to see if this axis is the only one moving
// the steps_to_move must have been set in block
bool Planner::isSoloMove(const Block& block, char axis)
{
	uint8_t ai= THEKERNEL.getMotionControl().getAxisActuator(axis);
	for (size_t i = 0; i < block.steps_to_move.size(); ++i) {
		if(block.steps_to_move[i] != 0 && i != ai) {
			return false;
		}
	}
	return true;
}

static uint32_t id = 0;
bool Planner::plan(const float *last_target, const float *target, int n_axis,  Actuator *actuators, float rate_mms)
{
	//printf("last_target: %f,%f,%f target: %f,%f,%f rate: %f\n", last_target[0], last_target[1], last_target[2], target[0], target[1], target[2], rate_mms);

	float deltas[n_axis];
	float distance = 0.0F;
	float sos = 0.0F;
	bool move = false;
	for (int i = 0; i < n_axis; ++i) {
		deltas[i] = target[i] - last_target[i];
		if(deltas[i] == 0) {
			continue;
		}
		move = true;
		if(THEKERNEL.getMotionControl().isPrimaryAxis(i)) {
			sos += powf(deltas[i], 2);
		}
	}
	// nothing moved
	if(!move) return false;

	// set if none of the primary axis is moving
	bool auxilliary_move= false;
	if(sos > 0.0F){
		distance = sqrtf( sos );

	} else {
		// non primary axis move (like extrude)
		// select the biggest one (usually just E)
		auto mi= std::max_element(&deltas[0], &deltas[n_axis], [](float a, float b){ return std::abs(a) < std::abs(b); } );
		distance = *mi;
		auxilliary_move= true;
	}

	// this is a HACK FIXME we need to be able to handle n axis properly
	uint8_t xaxis = THEKERNEL.getMotionControl().getAxisActuator('X');
	uint8_t yaxis = THEKERNEL.getMotionControl().getAxisActuator('Y');
	uint8_t zaxis = THEKERNEL.getMotionControl().getAxisActuator('Z');
	// uint8_t eaxis = THEKERNEL.getMotionControl().getAxisActuator('E');

	// TODO what if we have 4 primary axis? does this become unit_vec[4]?
	float unit_vec[3] {deltas[xaxis] / distance, deltas[yaxis] / distance, deltas[zaxis] / distance};

	// use default acceleration to start with
	float acceleration = default_acceleration;

	// Do not move faster than the configured max speed for any axis
	// downgrade the speed to make it right
	// also check the acceleration against the axis acceleration, and downgrade if needed
	for (int i = 0; i < n_axis; ++i) {
		if(deltas[i] == 0) continue;
		// adjust speed
		float max_speed = actuators[i].getMaxSpeed(); // in mm/sec
		float axis_speed = fabsf((deltas[i]/distance) * rate_mms);
		if (axis_speed > max_speed) {
			rate_mms *= ( max_speed / axis_speed );
		}
		// adjust acceleration
		float ma =  actuators[i].getAcceleration(); // in mm/sec²
		if(ma > 0.0F) {  // if axis does not have acceleration set then it uses the default_acceleration
			float ca = fabsf((deltas[i]/distance) * acceleration);
			if (ca > ma) {
				acceleration *= ( ma / ca );
			}
		}
	}

	// create the new block
	Block block;
	block.id = id++;

	block.acceleration = acceleration; // save acceleration in block

	// set direction and steps to move for each axis
	for (int i = 0; i < n_axis; i++) {
		std::tuple<bool, uint32_t> r = actuators[i].stepsToTarget(target[i]);
		block.direction.push_back(std::get<0>(r));
		block.steps_to_move.push_back(labs(std::get<1>(r)));
	}

	// Max number of steps, for all axis
	auto mi= std::max_element(block.steps_to_move.begin(), block.steps_to_move.end());
	block.steps_event_count = *mi;

	block.millimeters = distance;

	// Calculate speed in mm/sec for each axis. No divide by zero due to previous checks.
	block.nominal_speed = rate_mms;           // (mm/s) Always > 0
	block.nominal_rate = ceilf(block.steps_event_count * rate_mms / distance); // (step/s) Always > 0

	// default junction deviation
	float junction_deviation = this->junction_deviation;

	// if z_junction_deviation >= 0 and this is a z only move, then set the special z_junction_deviation
	// we also set unit_vec to zero if z_junction_deviation == 0 so next move starts at zero to
	if(z_junction_deviation >= 0.0F && isSoloMove(block, 'Z')) {
		junction_deviation= z_junction_deviation;
		if(junction_deviation == 0.0F) {
			memset(unit_vec, 0, sizeof(unit_vec));
		}
	}

	float vmax_junction = minimum_planner_speed; // Set default max junction speed
	if(!auxilliary_move && junction_deviation > 0.0F && (previous_unit_vec[0] != 0.0F || previous_unit_vec[1] != 0.0F || previous_unit_vec[2] != 0.0F)) {
		// if the primary axis do not move then treat it as if it always starts at 0
		// if the previous_unit_vec is all zeroes also start at 0
		// So if this move is a retract, or the previous was a retract then we will start at 0 speed
		// or if z_junction_deviation is 0 or previous move was solo z and ZJD is 0 then we also start at 0

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
		// and this allows one to stop with little to no decleration in many cases. This is particularly bad on leadscrew based systems that will skip steps.

		if (!lookahead_q.empty()) {
			float previous_nominal_speed = lookahead_q.front().nominal_speed;

			if (previous_nominal_speed > 0.0F) {
				// Compute cosine of angle between previous and current path. (previous_unit_vec is negative)
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
	}

	block.max_entry_speed = vmax_junction;

	// Initialize block entry speed. Compute based on deceleration to user-defined minimum_planner_speed.
	float v_allowable = maxAllowableSpeed(-acceleration, minimum_planner_speed, block.millimeters);
	block.entry_speed = std::min(vmax_junction, v_allowable);

	// Initialize planner efficiency flags
	// Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
	// If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
	// the current block and next block junction speeds are guaranteed to always be at their maximum
	// junction speeds in deceleration and acceleration, respectively. This is due to how the current
	// block nominal speed limits both the current and next maximum junction speeds. Hence, in both
	// the reverse and forward planners, the corresponding block junction speed will always be at the
	// the maximum junction speed and may always be ignored for any speed reduction checks.
	block.nominal_length_flag = (block.nominal_speed <= v_allowable);

	// Always calculate trapezoid for new block
	block.recalculate_flag = true;

	// Update previous path unit_vector and nominal speed
	memcpy(previous_unit_vec, unit_vec, sizeof(previous_unit_vec)); // previous_unit_vec[] = unit_vec[]

	// stick on the head/front of the block queue
	lookahead_q.push_front(block);

	// Math-heavy re-computing of the whole queue to take the new
	recalculate();

	// starting at end of the queue move any block that has recalculate_flag set to false into the ready queue
	auto curi = lookahead_q.rbegin();
	while(curi != lookahead_q.rend()) {
		auto nexti = std::next(curi);
		if(nexti == lookahead_q.rend()) break;

		// if both this one and the next one have recalculate flag flase then move this one to ready queue
		// this always leaves the last entry as a non calculate block and preserves the exit speeds
		if(!curi->recalculate_flag && !nexti->recalculate_flag) {
			Lock l(READY_Q_MUTEX); // gets a mutex on this
			l.lock();
			ready_q.push_front(*curi);
			l.unLock();
			lookahead_q.pop_back();
			curi = nexti;
		} else {
			// we stop looking when we hit the last block that has is set
			break;
		}
	}

	return true;
}

// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the
// acceleration within the allotted distance.
float Planner::maxAllowableSpeed(float acceleration, float target_velocity, float distance) const
{
	return sqrtf(target_velocity * target_velocity - 2.0F * acceleration * distance);
}

float Planner::maxExitSpeed(const Block &b) const
{
	// if nominal_length_flag is asserted
	// we are guaranteed to reach nominal speed regardless of entry speed
	// thus, max exit will always be nominal
	if (b.nominal_length_flag) return b.nominal_speed;

	// otherwise, we have to work out max exit speed based on entry and acceleration
	float max = maxAllowableSpeed(-b.acceleration, b.entry_speed, b.millimeters);

	return std::min(max, b.nominal_speed);
}

// Called by Planner::recalculate() when scanning the plan from head to tail.
float Planner::reversePass(Block &b, float exit_speed)
{
	// If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
	// If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
	// check for maximum allowable speed reductions to ensure maximum possible planned speed.
	if (b.entry_speed != b.max_entry_speed) {
		// If nominal length true, max junction speed is guaranteed to be reached. Only compute
		// for max allowable speed if block is decelerating and nominal length is false.
		if ((!b.nominal_length_flag) && (b.max_entry_speed > exit_speed)) {
			float max_entry_speed = maxAllowableSpeed(-b.acceleration, exit_speed, b.millimeters);
			b.entry_speed = std::min(max_entry_speed, b.max_entry_speed);
			return b.entry_speed;

		} else {
			b.entry_speed = b.max_entry_speed;
		}
	}

	return b.entry_speed;
}

// Called by Planner::recalculate() when scanning the plan from tail to head.
// returns maximum exit speed of this block
float Planner::forwardPass(Block &b, float prev_max_exit_speed)
{
	// If the previous block is an acceleration block, but it is not long enough to complete the
	// full speed change within the block, we need to adjust the entry speed accordingly. Entry
	// speeds have already been reset, maximized, and reverse planned by reverse planner.
	// If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.

	// TODO: find out if both of these checks are necessary
	if (prev_max_exit_speed > b.nominal_speed) prev_max_exit_speed = b.nominal_speed;
	if (prev_max_exit_speed > b.max_entry_speed) prev_max_exit_speed = b.max_entry_speed;

	if (prev_max_exit_speed <= b.entry_speed) {
		// accel limited
		b.entry_speed = prev_max_exit_speed;
		// since we're now acceleration or cruise limited
		// we don't need to recalculate our entry speed anymore
		b.recalculate_flag = false;
		//std::cout << "recalculate_flag set to false: " << b.id << "\n";
	}
	// else
	// // decel limited, do nothing

	return maxExitSpeed(b);
}

/*
 * a newly added block is decel limited
 *
 * we find its max entry speed given its exit speed
 *
 * for each block, walking backwards in the queue:
 *
 * if max entry speed == current entry speed
 * then we can set recalculate to false, since clearly adding another block didn't allow us to enter faster
 * and thus we don't need to check entry speed for this block any more
 *
 * once we find an accel limited block, we must find the max exit speed and walk the queue forwards
 *
 * for each block, walking forwards in the queue:
 *
 * given the exit speed of the previous block and our own max entry speed
 * we can tell if we're accel or decel limited (or coasting)
 *
 * if prev_exit > max_entry
 *     then we're still decel limited. update previous trapezoid with our max entry for prev exit
 * if max_entry >= prev_exit
 *     then we're accel limited. set recalculate to false, work out max exit speed
 *
 * finally, work out trapezoid for the final (and newest) block.
 */
void Planner::recalculate()
{
	/*
	 * Step 1:
	 * For each block, given the exit speed and acceleration, find the maximum entry speed
	 */

	float entry_speed = minimum_planner_speed;
	auto curi = lookahead_q.begin();
	auto lasti = std::prev(lookahead_q.end()); // iterator pointing to last entry in queue

	if (lookahead_q.size() > 1) {
		// from head to tail
		while(curi != lasti && curi->recalculate_flag) {
			entry_speed = reversePass(*curi, entry_speed);
			curi = std::next(curi);
		}

		if(curi == lasti) {
			// special case first entry needs to have recalculate_flag set to false
			curi->recalculate_flag = false;
		}

		/*
		 * Step 2:
		 * now current points to either the tail (back) of the queue or first non-recalculate block
		 * and has not had its reverse_pass called
		 * or its calc trap
		 * entry_speed is set to the *exit* speed of current.
		 * each block from current to head has its entry speed set to its max entry speed- limited by decel or nominal_rate
		 */

		float exit_speed = maxExitSpeed(*curi);
		while (curi != lookahead_q.begin()) {
			auto previ = curi;
			curi = std::prev(curi);

			// we pass the exit speed of the previous block
			// so this block can decide if it's accel or decel limited and update its fields as appropriate
			exit_speed = forwardPass(*curi, exit_speed);
			calculateTrapezoid(*previ, previ->entry_speed, curi->entry_speed);
		}
	}

	/*
	 * Step 3:
	 * work out trapezoid for final (and newest) block
	 */
	calculateTrapezoid(*curi, curi->entry_speed, minimum_planner_speed);
}

// this code written by Arthur Wolf based on his acceleration per tick work for Smoothie
void Planner::calculateTrapezoid(Block &block, float entryspeed, float exitspeed)
{
	//std::cout << "calculateTrapezoid for: " << block.id << " entry: " << entryspeed << ", exit: " << exitspeed << "\n";

	float initial_rate = block.nominal_rate * (entryspeed / block.nominal_speed); // steps/sec
	float final_rate = block.nominal_rate * (exitspeed / block.nominal_speed);
	//printf("Initial rate: %f, final_rate: %f\n", initial_rate, final_rate);
	// How many steps ( can be fractions of steps, we need very precise values ) to accelerate and decelerate
	// This is a simplification to get rid of rate_delta and get the steps/s² accel directly from the mm/s² accel
	float acceleration_per_second = (block.acceleration * block.steps_event_count) / block.millimeters;

	float maximum_possible_rate = sqrtf( ( block.steps_event_count * acceleration_per_second ) + ( ( powf(initial_rate, 2) + powf(final_rate, 2) ) / 2.0F ) );

	//printf("id %d: acceleration_per_second: %f, maximum_possible_rate: %f steps/sec, %f mm/sec\n", block.id, acceleration_per_second, maximum_possible_rate, maximum_possible_rate/100);

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

	float  acceleration_in_steps = (acceleration_time > 0.0F ) ? ( block.maximum_rate - initial_rate ) / acceleration_time : 0;
	float deceleration_in_steps =  (deceleration_time > 0.0F ) ? ( block.maximum_rate - final_rate ) / deceleration_time : 0;

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
	block.exit_speed = exitspeed;
}

void Planner::moveAllToReady()
{
	Lock l(READY_Q_MUTEX); // gets a mutex on this
	l.lock();
	while(!lookahead_q.empty()) {
		ready_q.push_front(lookahead_q.back());
		lookahead_q.pop_back();
	}
	l.unLock();
}

void Planner::purge()
{
	Lock l(READY_Q_MUTEX); // gets a mutex on this
	l.lock();
	lookahead_q.clear();
	ready_q.clear();
	l.unLock();
	reset();
}

bool Planner::handleSaveConfiguration(GCode &gc)
{
	THEDISPATCHER.getOS().printf("M204 S%1.4f ", default_acceleration);
	for(auto& a : THEKERNEL.getMotionControl().getActuators()) {
		float acc= a.getAcceleration();
		if(acc > 0.0F) {
			THEDISPATCHER.getOS().printf("%c%1.4f ", a.getAxis(), acc);
		}
	}
	THEDISPATCHER.getOS().printf("\n");

	THEDISPATCHER.getOS().printf("M205 S%1.4f X%1.4f Z%1.4f\n", minimum_planner_speed, junction_deviation, z_junction_deviation);
	return true;
}

bool Planner::handleConfigurations(GCode &gc)
{
	switch(gc.getCode()) {
		case 204: // M204 Snnn - set default acceleration to nnn, Xnnn sets X acceleration, ... , Znnn sets z acceleration etc
			if (gc.hasArg('S')) {
				default_acceleration = gc.getArg('S');
			}
			for(auto& a : THEKERNEL.getMotionControl().getActuators()) {
				char axis= a.getAxis();
				if(gc.hasArg(axis)){
					a.setAcceleration(gc.getArg(axis));
				}
			}

			break;

		case 205:  // M205 Xnnn - set junction deviation, Z - set Z junction deviation, S - Minimum planner speed
			if (gc.hasArg('S')) {
				minimum_planner_speed = gc.getArg('S');
			}
			if (gc.hasArg('X')) {
				junction_deviation = gc.getArg('X');
			}
			if (gc.hasArg('Z')) {
				z_junction_deviation = gc.getArg('Z');
			}
			break;

		default: return false;
	}

	return true;
}


#include "prettyprint.hpp"
void Planner::dump(std::ostream &o) const
{
	for (int i = 0; i < 2; ++i) {
		Queue_t q = i == 0 ? lookahead_q : ready_q;
		o << (i == 0 ? "Look ahead Queue:\n" : "Ready Queue\n");
		for(auto &b : q) {
			o <<
			  "Id: " << b.id                         << ", " <<
			  "accelerate_until: " <<  b.accelerate_until          << ", " <<
			  "decelerate_after: " <<  b.decelerate_after          << ", " <<
			  "acceleration_per_tick: " <<  b.acceleration_per_tick     << ", " <<
			  "deceleration_per_tick: " <<  b.deceleration_per_tick     << ", " <<
			  "total_move_ticks: " <<  b.total_move_ticks          << ", " <<
			  "maximum_rate: " <<  b.maximum_rate              << ", " <<
			  "nominal_rate: " <<  b.nominal_rate              << ", " <<
			  "nominal_speed: " <<  b.nominal_speed             << ", " <<
			  "nominal_length_flag: " <<  b.nominal_length_flag << ", " <<
			  "acceleration: " <<  b.acceleration             << ", " <<
			  "millimeters: " <<  b.millimeters               << ", " <<
			  "steps_event_count: " <<  b.steps_event_count         << ", " <<
			  "initial_rate: " <<  b.initial_rate              << ", " <<
			  "max_entry_speed: " <<  b.max_entry_speed           << ", " <<
			  "entry_speed: " <<  b.entry_speed               << ", " <<
			  "exit_speed: " <<  b.exit_speed                << ", " <<
			  "recalculate_flag: " <<  b.recalculate_flag   << "," <<
			  "direction: " <<  b.direction                 << "," <<
			  "steps_to_move: " << b.steps_to_move << "\n";
		}
	}
}
