/*
 * TMAG5273 Linear 3D Hall Effect Sensor Library
 * Authors: Jeevan Sanchez
 * Hardware: TI TMAG5273 https://www.ti.com/lit/ds/symlink/tmag5273.pdf
 * Env: Arduino Framework (PlatformIO - STM32 & ESP32)
 * Created: May.4.2026
 * Updated: May.6.2026
 * Purpose: SRAD Library for use across control boards interfacing with TMAG5273 sensor(s)
 * 
 * QRET Avionics 2025-2026
*/

#include "TMAG5273.h"

/// @brief initialize the sensor
/// @param addr I2C address of sensor (0x35 by default)
/// @param port reference to I2C port
/// @return true on successful initialization, false otherwise
bool TMAG5273::init(uint8_t addr, TwoWire &port) {
    _addr = addr; 
    _i2c = &port;  

    // confirm device (0X5449 | "TI" in ASCII)
    uint8_t id_msb =0;
    uint8_t id_lsb = 0;
    if (!readRegister(TMAG5273_REG::MANUFACTURER_ID_MSB, &id_msb)) return false; 
    if (!readRegister(TMAG5273_REG::MANUFACTURER_ID_LSB, &id_lsb)) return false; 

    if (id_msb != 0x54 || id_lsb != 0x49) return false;

    // configuration

    // configure SENSOR_CONFIG_1
    // bits 7-4: MAG_CH_EN = 7h (X, Y, Z enabled)
    // bits 3-0: SLEEP = 0h (1ms)
    // combined: 0111 0000b = 0x70
    if (!writeRegister(TMAG5273_REG::SENSOR_CONFIG_1, 0x70)) return false; 

    // configure T_CONFIG (Temperature)
    // bit 0: T_CH_EN = 1h (enable)
    if (!writeRegister(TMAG5273_REG::T_CONFIG, 0x01)) return false; 

    // set averaging speed (1h = 2x avg)
    if (!setAveraging(0x01)) return false; 

    // configure SENSOR_CONFIG_2
    // bits 3-2: ANGLE_EN = 0h (disabled)
    // bit 1: X_Y_RANGE = 1h (+- 80mT)
    // bit 0: Z_RANGE = 1h (+- 80mT)
    // combined: 0000 0011 = 0x03 
    if (!writeRegister(TMAG5273_REG::SENSOR_CONFIG_2, 0x03)) return false; 
    _range = 80.0f; 

    // activate 
    // bits 4: LP_LN = 0h (low active current mode)
    // bits 1-0: OPERATING_MODE = 2h (continuous measure mode)
    // combined: 0000 0010 = 0x02
    if (!writeRegister(TMAG5273_REG::DEVICE_CONFIG_2, 0x02)) return false;

    uint8_t deviceStatus = getDeviceStatus(); 
    if (deviceStatus != 0) return false; 

    return true;
} 

/// @brief obtain flux values from sensor readings for all axes
/// @param axes pointer to float array to store flux values in mT
/// @return true on successful sensor read, false otherwise
bool TMAG5273::getAllFlux(float* axes) {
    uint8_t buffer[6]; // buffer for X_MSB through Z_LSB

    // obtain data; sensor auto-increments
    if (!readRegister(TMAG5273_REG::X_MSB_RESULT, buffer, 6)) return false; 

    // combine MSB & LSB, convert to mT, and copy to output aray
    axes[0] = rawTomT(combineBytes(buffer[0], buffer[1])); 
    axes[1] = rawTomT(combineBytes(buffer[2], buffer[3])); 
    axes[2] = rawTomT(combineBytes(buffer[4], buffer[5])); 

    return true;
}

// Obtain flux values (mT) for a specific axis (X, Y, Z)

float TMAG5273::getFluxX() {
    float data[3]; 
    if (!getAllFlux(data)) return NAN; // unsuccessful getAllFlux()

    return data[0]; 
}

float TMAG5273::getFluxY() {
    float data[3]; 
    if (!getAllFlux(data)) return NAN; // unsuccessful getAllFlux()

    return data[1]; 
}

float TMAG5273::getFluxZ() {
    float data[3]; 
    if (!getAllFlux(data)) return NAN; // unsuccessful getAllFlux()

    return data[2]; 
}

/// @brief obtain temperature values from sensor
/// @return float temperature value in celsius
float TMAG5273::getTemp() {
    uint8_t buffer[2]; 

    // read MSB and LSB
    if (!readRegister(TMAG5273_REG::T_MSB_RESULT, buffer, 2)) return NAN; // read error

    // combine into unsigned 16-bit
    uint16_t rawTemp = combineBytes(buffer[0], buffer[1]); 

    // convert to celsius using Equation 12 from the datsheet using provided constants 
    // T = TSENSE_T0 + ((rawTemp - TADC_T0) / (TADC_RES))
    float temp = TSENSE_T0 + (((float)rawTemp - TADC_T0) / TADC_RES);
    return temp; 
}

/// @brief set the averaging speed
/// @param mode averaging mode speed; found on datasheet Table 8-3
/// @return true if successful set, false otherwise
bool TMAG5273::setAveraging(uint8_t mode) {
     // guard against mode size 
    if (mode > 0x07) return false;

    uint8_t value = (mode & 0x07) << 2; // (mode & 0x07) ensures we only ever use the bottom 3 bits
    return writeRegister(TMAG5273_REG::DEVICE_CONFIG_1, value); 
}

/*  bit 3: OSC_ER (oscillator error) 
 bit 2: INT_ER (interrupt pin error) 
 bit 1: OTP_CRC_ER (optional CRC error)
 bit 0: VCC_UV_ER (VCC undervoltage error) */

/// @brief check device status for hardware erors
/// @return the raw status byte; 0 means no errors. 
uint8_t TMAG5273::getDeviceStatus() {
    uint8_t status = 0;
    if (!readRegister(TMAG5273_REG::DEVICE_STATUS, &status)) return 0XFF; // read error 

    return (status & 0x0F); // only focus on bits 3-0
}

/// @brief write a single byte of data into a register
/// @param reg reigster address to write to 
/// @param data information to be written
/// @return true if the write succeeds, false otherwise
bool TMAG5273::writeRegister(uint8_t reg, uint8_t data) {
    if (!_i2c) return false; 

    _i2c->beginTransmission(_addr); 
    _i2c->write(reg); 
    _i2c->write(data); 

    if (_i2c->endTransmission() == 0) {
            return true; // success
    } else {
        return false; // endTransmission() = 1 = data too long, 2 = NACK on addr transmit, 3 = NACK on data transmit, 4 = other
    }
}

/// @brief reads one or more bytes from a specific register
/// @param reg starting register address to read from
/// @param buffer pointer to a byte array for stored data
/// @param len number of bytes to read (defaults to 1)
/// @return true if the I2C transaction was successful and requested number of bytes were received, false otherwise
bool TMAG5273::readRegister(uint8_t reg, uint8_t* buffer, uint8_t len) {
    if (!_i2c || len == 0) return false;

    _i2c->beginTransmission(_addr);
    _i2c->write(reg);

    if (_i2c->endTransmission(false) != 0) return false;  // send repeated start to keep bus active

    // request "len" bytes from sensor
    if (_i2c->requestFrom(_addr, len) == len) { 
        for (uint8_t i = 0; i < len; i++) {
            buffer[i] = _i2c->read(); // copy data into buffer
        }
        
        return true;
    }

    return false;
}

/// @brief combine 8-bit data from msb and lsb registers
/// @param msb data from msb register
/// @param lsb data from lsb register
/// @return combined 16-bit data
int16_t TMAG5273::combineBytes(uint8_t msb, uint8_t lsb) {
    // shift, add, cast
    return (int16_t)((msb << 8) | lsb); 
}

 /// @brief convert raw 16-bit data to a milli-Tesla float value
 /// @param raw 16-bit data read from sensor
 /// @param range sensor range configured in init()
 /// @return milli-Tesla float value
 float TMAG5273::rawTomT(int16_t raw, float range) {
    return (float)raw * (range / B16_SCALE);
 }

