#pragma once

#include <functional>


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

    void initialize();

    MotionControl& getMotionControl() const { return *motion_control; }
    Planner& getPlanner() const { return *planner; }
    GCodeProcessor& getGCodeProcessor() const { return *gcode_processor; }

    // General HAL functions go here
    enum HAL_FUNCTION_INDEX
    {
        // EEPROM/FLASH read and write for non volatile storage
        NV_INIT,
        NV_WRITE,
        NV_READ,

        N_HAL_FUNCTIONS
    };
    using HAL_function_t = std::function<size_t(void*,size_t,uint32_t)>;
    void assignHALFunction(HAL_FUNCTION_INDEX i, HAL_function_t fnc) { hal_functions[i] = fnc; }

    size_t nonVolatileWrite(void *buf, size_t len, uint32_t offset) { return hal_functions[NV_WRITE] ? hal_functions[NV_WRITE](buf, len, offset) : 0;  }
    size_t nonVolatileRead(void *buf, size_t len, uint32_t offset) { return hal_functions[NV_READ] ? hal_functions[NV_READ](buf, len, offset) : 0; }

private:
	MotionControl *motion_control;
	Planner *planner;
	GCodeProcessor *gcode_processor;
    HAL_function_t hal_functions[N_HAL_FUNCTIONS];
};
