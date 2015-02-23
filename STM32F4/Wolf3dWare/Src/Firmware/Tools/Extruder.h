#pragma once

#include <stdint.h>

class GCode;

class Extruder
{
public:
	Extruder(uint8_t i) : pool_index(i), retracted(false), cancel_zlift_restore(false) {};
	~Extruder() {};
	void initialize();

private:
	bool handleFilamentDiameter(GCode&);
	bool handleRetractSettings(GCode&);
	bool handleUnRetractSettings(GCode&);
	bool handleFlowRateSetting(GCode&);
	bool handleRetract(GCode& gc);
	bool handleG0G1(GCode& gc);

	void setScale();
	uint8_t pool_index; // tells us which temoerature cointrol we are associated with

	float filament_diameter{3.0};
	float volumetric_multiplier{1.0F};
	float extruder_multiplier{1.0F};
	float retract_length{1.0F};
	float retract_feedrate{30.0F};
	float retract_zlift_length{0.0F};
	float retract_zlift_feedrate{0.0F};
	float retract_recover_length{0.0F};
	float retract_recover_feedrate{8.0F};

	struct {
		bool retracted:1;
		bool cancel_zlift_restore:1;
	};
};
