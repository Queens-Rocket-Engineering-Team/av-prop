/*
 * Header for TMAG5273A1 Linear 3D Hall Effect Sensor Library
 * 
 * Consult Table 6-2 in datasheet for I2C adresses if NOT using the A1 model. 
       * https://www.ti.com/lit/ds/symlink/tmag5273.pdf
 * 
 * QRET Avionics 2025-2026
*/

#ifndef TMAG5273_H
#define TMAG5273_H

#include <Arduino.h>
#include <Wire.h>

#include "registers.h"

class TMAG5273 {
    public:
        bool init(uint8_t addr = 0x35, TwoWire &port = Wire); 

        // data
        float getFluxX(); 
        float getFluxY(); 
        float getFluxZ(); 
        float getTemp();
        bool getAllFlux(float* axes); 

        // settings
        bool setAveraging(uint8_t mode); 
        
        uint8_t getDeviceStatus(); 

    private: 
        uint8_t _addr; 
        TwoWire* _i2c;
        float _range; 
 

        // temperature constants from datasheets (section 5.6)
        const float TADC_T0 = 17508.0f;
        const float TSENSE_T0 = 25.0f; 
        const float TADC_RES = 58.0f; 

        const float B16_SCALE = 32768.0f; // 32,768 (2^15) is the 16-bit scaling value

        bool readRegister(uint8_t reg, uint8_t* buffer, uint8_t len = 1); 
        bool writeRegister(uint8_t reg, uint8_t data); 
        int16_t combineBytes(uint8_t msb, uint8_t lsb); 
        float rawTomT(int16_t raw, float range = 80.0f); // if range changes, change this to 40.0f
}; 

#endif 