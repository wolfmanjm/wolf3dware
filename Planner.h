#pragma once

class GCode;

class Planner
{
public:
	Planner();
	~Planner();
	bool plan(GCode& gc, const float *last_target, const float *target, int n_axis, float rate_mms);

};
