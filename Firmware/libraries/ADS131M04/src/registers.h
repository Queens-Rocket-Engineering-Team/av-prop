#ifndef REGISTERS_H
#define REGISTERS_H

/*
 * Register definitions for the ADS131M04
 * Consult the datasheet (Table 8-12) for register contents: https://www.ti.com/lit/ds/symlink/ads131m04.pdf
 * 
 * Not used in current implementation, will be used if library extends to ADC settings and configuration
 * 
 * QRET Avionics 2025-2026
 */

namespace ADS131M04_REG {

    // device settings and indicators
    constexpr uint8_t ID = 0x00;
    constexpr uint8_t STATUS = 0X01; 

    // global settings
    constexpr uint8_t MODE = 0X02; 
    constexpr uint8_t CLOCK = 0x03; 
    constexpr uint8_t GAIN = 0x04; 
    constexpr uint8_t CFG = 0x06; 
    constexpr uint8_t THRSHLD_MSB = 0X07;
    constexpr uint8_t THRSHLD_LSB = 0X08;

    // channel-specific setting
    constexpr uint8_t CH0_CFG = 0X09;
    constexpr uint8_t CH0_OCAL_MSB = 0X0A;
    constexpr uint8_t CH0_OCAL_LSB = 0X0B;
    constexpr uint8_t CH0_GCAL_MSB = 0x0C;
    constexpr uint8_t CHO0_GCAL_LSB = 0X0D;
    constexpr uint8_t CH1_CFG = 0X0E;
    constexpr uint8_t CH1_OCAL_MSB = 0X0F;
    constexpr uint8_t CH1_OCAL_LSB = 0X10;
    constexpr uint8_t CH1_GCAL_MSB = 0X11;
    constexpr uint8_t CH1_GCAL_LSB = 0X12; 
    constexpr uint8_t CH2_CFG = 0X13;
    constexpr uint8_t CH2_OCAL_MSB = 0X14;
    constexpr uint8_t CH2_OCAL_LSB = 0X15;
    constexpr uint8_t CH2_GCAL_MSB = 0X16;
    constexpr uint8_t CH2_GCAL_LSB = 0X17;
    constexpr uint8_t CH3_CFG = 0X18;
    constexpr uint8_t CH3_OCAL_MSB = 0x19;
    constexpr uint8_t CH3_OCAL_LSB = 0X1A;
    constexpr uint8_t CH3_GCAL_MSB = 0X1B;
    constexpr uint8_t CH3_GCAL_LSB = 0X1C;

    // reg map crc and reserved
    constexpr uint8_t REGMAP_CRC = 0X3E; 
    constexpr uint8_t RESERVED = 0x3F;
};

 #endif 
