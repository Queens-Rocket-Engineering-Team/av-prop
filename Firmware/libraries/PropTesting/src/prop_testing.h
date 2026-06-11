/*
 * Header for testing utilities.
 *
 * Constants for ADC config.
 * Constants for PT and TC processing.
 * Function prototypes
 * 
 * QRET Avionics 2025-2026
 */

#ifndef PROP_TESTING_H
#define PROP_TESTING_H

#include <Arduino.h>
#include <ADS131M04.h>
#include <SPI.h>
#include <TMAG5273.h>
#include <Wire.h>
#include <SoftwareSerial.h>

// PT constants
const float shuntResistance = 62.0;
const float maxPSI = 100; // needs to be double-checked

// Thermistor (cold junction) constants 
// Ohmite TX Series NTC thermistor on pin 10
// Assumes voltage divider: 3.3V -> R_SERIES -> NTC -> GND
const float THERM_R_SERIES  = 10000.0;  // series resistor (Ohms)
const float THERM_R0        = 10000.0;  // thermistor resistance at 25°C (Ohms)
const float THERM_B         = 3435.0;   // B-value (K) — TX06F103F3435ER / TX08F103F3435ER
const float THERM_T0        = 298.15;   // 25°C in Kelvin
const float STM32_VREF      = 3.3;
const float STM32_ADC_RES   = 4095.0;   // 12-bit

// --- K-type thermocouple constants ---
// CH0 on the ADS131M04 is a K-type thermocouple
// Seebeck coefficient for K-type: ~41.276 µV/°C (linear approx, valid -200 to +1372°C)
const float K_TYPE_SEEBECK_UV_PER_C = 41.276; // µV / °C

// POWER
void enablePower(const int enablePin);
void disablePower(const int enablePin); 

// ADC
void adcSetup(ADS131M04& adc);
bool readAllADC(ADS131M04& adc, int32_t* outputBuffer);

// PT
float processPT(float voltagePT);

// TC
float readColdJunction(const int TEMP_SENSE);
float readDeltaTemp(float voltage);
float processTC(float voltageTC, const int thermPin);

// VALVE CONTROL
void enableValve(const int solENPin);
void disableValve(const int solENPin);

// UTILS
void blinkLed(int ledPin, int delayMs = 800);
void flashLeds(const int ledArray[], int ledCount);

#endif