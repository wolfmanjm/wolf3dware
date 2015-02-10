#pragma once

#include <bitset>
#include <vector>

#define STEP_TICKER_FREQUENCY 100000.0F
#define STEP_TICKER_FREQUENCY_2 (STEP_TICKER_FREQUENCY*STEP_TICKER_FREQUENCY)

struct Block {
	uint32_t id{0};
	uint32_t accelerate_until{0};
	uint32_t decelerate_after{0};
	float acceleration_per_tick{0};
	float deceleration_per_tick {0};
	uint32_t total_move_ticks{0};
	float maximum_rate {0};
	float nominal_rate{0};
	float nominal_speed{0};
	float acceleration{0};
	float millimeters{0};
	uint32_t steps_event_count{0};
	float initial_rate{0};
	float max_entry_speed{0};
	float entry_speed{0};
	float exit_speed{0};
	std::vector<uint32_t> steps_to_move;
	std::vector<bool> direction;
	struct {
		bool nominal_length_flag:1;
		bool recalculate_flag:1;
		bool ready:1;
	};
};

