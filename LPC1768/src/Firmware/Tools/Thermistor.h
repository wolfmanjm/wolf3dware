/*
    this file was part of smoothie (http://smoothieware.org/)
    it has been highly modified for this project, and is licensed under the same license as smoothieware
*/

#pragma once

#include "TempSensor.h"

#include <tuple>
#include <functional>

class Thermistor : public TempSensor
{
    public:
        Thermistor(uint8_t, float);
        ~Thermistor();
        void initialize();

        float getTemperature();
        bool setOptional(const sensor_options_t& options);
        bool getOptional(sensor_options_t& options);
        void getRaw(GCode& gc);
        static std::tuple<float,float,float> calculateSteinhartHartCoefficients(float t1, float r1, float t2, float r2, float t3, float r3);
        enum HAL_FUNCTION_INDEX
        {
            GET_ADC
        };
        using HAL_function_t = std::function<uint16_t(uint8_t)>;
        void assignHALFunction(HAL_FUNCTION_INDEX i, HAL_function_t fnc) { if(i == GET_ADC) getADC= fnc; }

    private:
        int newThermistorReading();
        float adcValueToTemperature(int adc_value);
        void calcJk();
        HAL_function_t getADC;

        uint8_t pool_index;

        // Thermistor computation settings using beta, not used if using SteinhartHart
        float r0;
        float t0;

        // on board resistor settings
        int r1;
        int r2;
        float max_adc;

        union {
            // this saves memory as we only use either beta or SHH
            struct{
                float beta;
                float j;
                float k;
            };
            struct{
                float c1;
                float c2;
                float c3;
            };
        };

        struct {
            bool bad_config:1;
            bool use_steinhart_hart:1;
        };
};

