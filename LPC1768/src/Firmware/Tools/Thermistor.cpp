/*
    this file was part of smoothie (http://smoothieware.org/)
    it has been highly modified for this project, and is licensed under the same license as smoothieware
*/

#include "Thermistor.h"
#include "../Dispatcher.h"
#include "../GCode.h"

// a const list of predefined thermistors
#include "predefined_thermistors.h"

#include <fastmath.h>
#include <algorithm>
#include <string>
#include <deque>
#include <string.h>

Thermistor::Thermistor(uint8_t index)
{
    pool_index= index;
    bad_config = false;
    use_steinhart_hart= false;
    beta= 0.0F; // not used by default
}

Thermistor::~Thermistor()
{
}

void Thermistor::initialize()
{
    // Values are here : http://reprap.org/wiki/Thermistor
    r0   = 100000;
    t0   = 25;
    r1   = 0;
    r2   = 4700;
    beta = 4066;

    bool found= false;

    // load a predefined thermistor name if found
    std::string thermistor = "Semitec";
    if(!thermistor.empty()) {
        for (auto& i : predefined_thermistors) {
            if(thermistor.compare(i.name) == 0) {
                c1 = i.c1;
                c2 = i.c2;
                c3 = i.c3;
                r1 = i.r1;
                r2 = i.r2;
                use_steinhart_hart= true;
                found= true;
                break;
            }
        }

        // fall back to the old beta pre-defined table if not found above
        if(!found) {
            for (auto& i : predefined_thermistors_beta) {
                if(thermistor.compare(i.name) == 0) {
                    beta = i.beta;
                    r0 = i.r0;
                    t0 = i.t0;
                    r1 = i.r1;
                    r2 = i.r2;
                    use_steinhart_hart= false;
                    found= true;
                    break;
                }
            }
        }
    }

    if(!use_steinhart_hart) {
        // if using beta
        calcJk();

    }
}

// calculate the coefficients from the supplied three Temp/Resistance pairs
// copied from https://github.com/MarlinFirmware/Marlin/blob/Development/Marlin/scripts/createTemperatureLookupMarlin.py
std::tuple<float,float,float> Thermistor::calculateSteinhartHartCoefficients(float t1, float r1, float t2, float r2, float t3, float r3)
{
    float l1 = logf(r1);
    float l2 = logf(r2);
    float l3 = logf(r3);

    float y1 = 1.0F / (t1 + 273.15F);
    float y2 = 1.0F / (t2 + 273.15F);
    float y3 = 1.0F / (t3 + 273.15F);
    float x = (y2 - y1) / (l2 - l1);
    float y = (y3 - y1) / (l3 - l1);
    float c = (y - x) / ((l3 - l2) * (l1 + l2 + l3));
    float b = x - c * (powf(l1,2) + powf(l2,2) + l1 * l2);
    float a = y1 - (b + powf(l1,2) * c) * l1;

    if(c < 0) {
        //THEKERNEL.OOBprintf("WARNING: negative coefficient in calculate_steinhart_hart_coefficients. Something may be wrong with the measurements\n");
        c = -c;
    }
    return std::make_tuple(a, b, c);
}

void Thermistor::calcJk()
{
    // Thermistor math
    if(beta > 0.0F) {
        j = (1.0F / beta);
        k = (1.0F / (t0 + 273.15F));
    }else{
        //THEKERNEL.OOBprintf("WARNING: beta cannot be 0\n");
        bad_config= true;
    }
}

float Thermistor::getTemperature()
{
    if(bad_config) return infinityf();
    return adcValueToTemperature(newThermistorReading());
}

void Thermistor::getRaw(GCode& gc)
{
    if(bad_config) {
       gc.getOS().printf("WARNING: The config is bad for this temperature sensor\n");
    }

    int adc_value= newThermistorReading();
    if(adc_value == 0) {
        gc.getOS().printf("not a valid ADC reading\n");
        return;
    }

    // resistance of the thermistor in ohms
    float r = r2 / ((4095.0F / adc_value) - 1.0F);
    if (r1 > 0.0F) r = (r1 * r) / (r1 - r);

    gc.getOS().printf("adc= %d, resistance= %f\n", adc_value, r);

    if(use_steinhart_hart) {
        gc.getOS().printf("S/H c1= %1.18f, c2= %1.18f, c3= %1.18f\n", c1, c2, c3);
        float l = logf(r);
        float t= (1.0F / (c1 + c2 * l + c3 * powf(l,3))) - 273.15F;
        gc.getOS().printf("S/H temp= %f\n", t);
    }else{
        float t= (1.0F / (k + (j * logf(r / r0)))) - 273.15F;
        gc.getOS().printf("beta temp= %f\n", t);
    }
}

float Thermistor::adcValueToTemperature(int adc_value)
{
    if ((adc_value == 4095) || (adc_value == 0))
        return infinityf();

    // resistance of the thermistor in ohms
    float r = r2 / ((4095.0F / adc_value) - 1.0F);
    if (r1 > 0.0F) r = (r1 * r) / (r1 - r);

    float t;
    if(use_steinhart_hart) {
        float l = logf(r);
        t= (1.0F / (c1 + c2 * l + c3 * powf(l,3))) - 273.15F;
    }else{
        // use Beta value
        t= (1.0F / (k + (j * logf(r / r0)))) - 273.15F;
    }

    return t;
}

// reading the DMA filled ADC buffer of 8 readings and taking the 4 middle values as average
int Thermistor::newThermistorReading()
{
    uint16_t *dma_bufb= getADC(pool_index); // get a pointer to the copy of the DMA buffer
    if(dma_bufb == nullptr) {
        // not setup for this channel
        return 0;
    }
    // grab the buffer
    uint16_t b[8];
    memcpy(b, dma_bufb, sizeof(b));
    std::deque<uint16_t> buf(b, b+8);

    // sort
    std::sort (buf.begin(), buf.end());
    // eliminate first and last two
    buf.pop_back(); buf.pop_back();
    buf.pop_front(); buf.pop_front();
    uint16_t sum= std::accumulate(buf.begin(), buf.end(), 0);
    return roundf(sum/4.0F); // return the average
}

bool Thermistor::setOptional(const sensor_options_t& options) {
    bool define_beta= false;
    bool change_beta= false;
    uint8_t define_shh= 0;

    for(auto &i : options) {
        switch(i.first) {
            case 'B': beta= i.second; define_beta= true; break;
            case 'R': r0= i.second; change_beta= true; break;
            case 'X': t0= i.second; change_beta= true; break;
            case 'I': c1= i.second; define_shh++; break;
            case 'J': c2= i.second; define_shh++; break;
            case 'K': c3= i.second; define_shh++; break;
        }
    }

    bool error= false;
    // if in Steinhart-Hart mode make sure B is specified, if in beta mode make sure all C1,C2,C3 are set and no beta settings
    // this is needed if swapping between modes
    if(use_steinhart_hart && define_shh == 0 && !define_beta) error= true; // if switching from SHH to beta need to specify new beta
    if(!use_steinhart_hart && define_shh > 0 && (define_beta || change_beta)) error= true; // if in beta mode and switching to SHH malke sure no beta settings are set
    if(!use_steinhart_hart && !(define_beta || change_beta) && define_shh != 3) error= true; // if in beta mode and switching to SHH must specify all three SHH
    if(use_steinhart_hart && define_shh > 0 && (define_beta || change_beta)) error= true; // if setting SHH anfd already in SHH do not specify any beta values

    if(error) {
        bad_config= true;
        return false;
    }
    if(define_beta || change_beta) {
        calcJk();
        use_steinhart_hart= false;
    }else if(define_shh > 0) {
        use_steinhart_hart= true;
    }else{
        return false;
    }

    if(bad_config) bad_config= false;

    return true;
}

bool Thermistor::getOptional(sensor_options_t& options) {
    if(use_steinhart_hart) {
        options['I']= c1;
        options['J']= c2;
        options['K']= c3;

    }else{
        options['B']= beta;
        options['X']= t0;
        options['R']= r0;
    }

    return true;
};
