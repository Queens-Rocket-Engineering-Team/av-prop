#ifndef PINOUTS_H
#define PINOUTS_H

#include <Arduino.h> // needed for PB/A pin assignment
#include <cstdint>

// pinouts.h - Hydra Lower Control Module V1.0 2025/2026 (STM32F103CB) pin map.
namespace pins {

// --- Serial (USB-UART Bridge; TX/RX are swapped on the PCB) ---
//  !!! Fix in next board spin !!!
// Use software serial
constexpr uint8_t kSerialTx = PA9;  // USART_TX -> USB_RX
constexpr uint8_t kSerialRx = PA10; // USART_RX <- USB_TX

// --- CAN bus ---
constexpr uint8_t kCanRx = PB8;
constexpr uint8_t kCanTx = PB9;

// --- SPI bus 1 (ADC & flash) ---
constexpr uint8_t kSpiSclk = PA5;
constexpr uint8_t kSpiMiso = PA6;
constexpr uint8_t kSpiMosi = PA7;

// --- Flash ---
constexpr uint8_t kFlashCs    = PB0;
constexpr uint8_t kFlashReset = PB12;

// --- ADC (ADS131M04) ---
constexpr uint8_t kAdcClkin = PA0;   // TIM2 CH1 clock output to ADS131M04
constexpr uint8_t kAdcDrdy  = PA1;
constexpr uint8_t kAdcCs    = PA3;
constexpr uint8_t kCjcSense = PA4;   // cold-junction thermistor, ADC12_IN4

// --- Hall I2C bus 1 ---
constexpr uint8_t kHallScl1 = PB6;
constexpr uint8_t kHallSda1 = PB7;

// --- Hall I2C bus 2 ---
constexpr uint8_t kHallScl2 = PB10;
constexpr uint8_t kHallSda2 = PB11;

// --- Solenoids / valve power ---
constexpr uint8_t kSol1En = PB13;
constexpr uint8_t kSol2En = PB14;
constexpr uint8_t kVptEn  = PA2;
constexpr uint8_t kVsolEn = PA11;

// --- Sensing ---
constexpr uint8_t kVsolSense = PB1;

// --- LEDs ---
constexpr uint8_t kCanLed  = PB4;
constexpr uint8_t kDebugLed = PA8;
constexpr uint8_t kRgbData  = PB5;

}

#endif  // LOWER_CONTROL_PINOUTS_H
