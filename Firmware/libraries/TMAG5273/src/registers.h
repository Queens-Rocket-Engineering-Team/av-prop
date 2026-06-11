#ifndef REGISTERS_H
#define REGISTERS_H

/*
 * Register definitions for the TMAG5273
 * Consult the datasheet (Table 8-1) for register contents: https://www.ti.com/lit/ds/symlink/tmag5273.pdf
 *  
 * QRET Avionics 2025-2026
 */

namespace TMAG5273_REG {
    
    // configuration 
    constexpr uint8_t DEVICE_CONFIG_1 = 0X00;
    constexpr uint8_t DEVICE_CONFIG_2 = 0x01; 
    constexpr uint8_t SENSOR_CONFIG_1 = 0X02; 
    constexpr uint8_t SENSOR_CONFIG_2 = 0X03;
    constexpr uint8_t T_CONFIG = 0x07; 
     

    // identity 
    constexpr uint8_t DEVICE_ID = 0X0D; 
    constexpr uint8_t MANUFACTURER_ID_LSB = 0x0E;
    constexpr uint8_t MANUFACTURER_ID_MSB = 0X0F; 

    // data 
    // only defined MSB (auto-increments to LSB)
    const uint8_t T_MSB_RESULT = 0X10; 
    constexpr uint8_t X_MSB_RESULT = 0X12; 
    constexpr uint8_t Y_MSB_RESULT = 0X14; 
    constexpr uint8_t Z_MSB_RESULT = 0X16; 

    // status
    constexpr uint8_t DEVICE_STATUS = 0x1C; 
    
};

 #endif 
