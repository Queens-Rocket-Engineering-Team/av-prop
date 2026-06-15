#ifndef PINOUTS_H
#define PINOUTS_H

#include <cstdint>

// pinouts.h - Pegasus Upper Control Module V1.0 2025/2026 (ESP32-S3) pin map.
namespace pins {

// --- CAN ---
constexpr uint8_t kCanTx = 1;
constexpr uint8_t kCanRx = 2;

// --- Solenoids / valve ---
constexpr uint8_t kSol1En = 9;
constexpr uint8_t kSol2En = 10;
constexpr uint8_t kVptEn  = 47;

// --- ADC (ADS131M04, SPI) ---
constexpr uint8_t kAdcClkin = 11;
constexpr uint8_t kAdcMosi  = 12;
constexpr uint8_t kAdcMiso  = 13;
constexpr uint8_t kAdcSclk  = 14;
constexpr uint8_t kAdcDrdy  = 19;

// --- Hall (I2C, TMAG5273) ---
constexpr uint8_t kHallScl     = 17;
constexpr uint8_t kHallSda     = 18;
constexpr uint8_t kHallI2cAddr = 0x35;

// --- Sensing ---
constexpr uint8_t kSense24v = 7;

// --- LEDs ---
constexpr uint8_t kWifiLed  = 35;
constexpr uint8_t kCanLed   = 36;
constexpr uint8_t kDebugLed  = 37;
constexpr uint8_t kRgbData  = 42;

// --- Buzzer ---
constexpr uint8_t kBuzzEn = 41;

}

#endif // PINOUTS_H
