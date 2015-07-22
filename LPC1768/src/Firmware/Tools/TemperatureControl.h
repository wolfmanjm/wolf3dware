/*
      this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
      smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
      smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
      you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "Lock.h"

#include <stdint.h>
#include <string>
#include <functional>

class TempSensor;
class GCode;

class TemperatureControl {

    public:
        TemperatureControl(const char *, uint8_t, TempSensor&);
        ~TemperatureControl();
        void initialize();
        enum HAL_FUNCTION_INDEX
        {
            SET_PWM
        };
        using HAL_function_t = std::function<void(uint8_t, float)>;
        void assignHALFunction(HAL_FUNCTION_INDEX i, HAL_function_t fnc) { if(i == SET_PWM) setPWM= fnc; }

    private:
        static void temperatureTimerCallback( void const *pTc );
        bool onGcodeReceived(GCode& gc);
        void setDesiredTemperature(float desired_temperature);
        float getTemperature() const { return last_reading; }
        void readTemperatureTick();
        void pidProcess(float);
        void setPIDp(float p) { p_factor = p; }
        void setPIDi(float i) { i_factor = i * PIDdt; }
        void setPIDd(float d) { d_factor = d / PIDdt; }
        HAL_function_t setPWM;

        Lock lock;
        void* read_temperature_timer_handle{0};

        uint8_t pool_index;
        float target_temperature{0};
        float max_temp{280};
        TempSensor& sensor;
        std::string designator;

        // PID runtime
        float i_max{100};
        float max_pwm{100}; // percentage
        float pwm_out{0};
        float last_reading{0};
        float hysteresis{2};
        float i_term{0};
        float last_input{0};

        // PID settings
        float p_factor;
        float i_factor;
        float d_factor;
        float PIDdt;

        struct {
            bool use_bangbang:1;
            bool min_temp_violated:1;
            bool active:1;
            bool readonly:1;
            bool sensor_settings:1;
        };
};
