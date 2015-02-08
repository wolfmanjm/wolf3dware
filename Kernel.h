#pragma once

#define THEKERNEL Kernel::getInstance()

class MotionControl;
class Planner;
class GCodeProcessor;

class Kernel
{
public:
	Kernel();
	~Kernel();
    static Kernel &getInstance()
    {
        static Kernel instance;
        return instance;
    }

    MotionControl& getMotionControl() const { return *motion_control; }
    Planner& getPlanner() const { return *planner; }
    GCodeProcessor& getGCodeProcessor() const { return *gcode_processor; }

private:
	MotionControl *motion_control;
	Planner *planner;
	GCodeProcessor *gcode_processor;

};
