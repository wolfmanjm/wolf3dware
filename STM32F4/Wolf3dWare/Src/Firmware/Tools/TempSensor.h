/*
    this file was part of smoothie (http://smoothieware.org/)
    it has been highly modified for this project, and is licensed under the same license as smoothieware
*/


#pragma once

#include <map>
#include <cmath>

class TempSensor
{
public:
	virtual ~TempSensor() {}
	virtual void initialize()= 0;

	// Return temperature in degrees Celsius.
	virtual float getTemperature() { return NAN; }

	using sensor_options_t = std::map<char, float>;
	virtual bool setOptional(const sensor_options_t& options) { return false; }
	virtual bool getOptional(sensor_options_t& options) { return false; }
	virtual void getRaw() {}
};

