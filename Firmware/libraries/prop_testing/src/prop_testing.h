#ifndef PROP_TESTING_H
#define PROP_TESTING_H

#include <Arduino.h>

// --- Pressure transducer (4-20 mA) ---
constexpr float kPtShuntOhms = 62.0f;
// TODO: update for PTs in use
constexpr float kPtMaxPsi    = 2000.0f;

// --- Thermistor (cold-junction NTC, Ohmite TX series) ---
constexpr float kThermSeriesOhms = 10000.0f;
constexpr float kThermR0Ohms     = 10000.0f;
constexpr float kThermBeta       = 3435.0f;
constexpr float kThermT0Kelvin   = 298.15f;
constexpr float kAdcVref         = 1.2f;
constexpr float kAdcMaxCount     = 4095.0f;

// --- K-type thermocouple ---
constexpr float kSeebeckUvPerC = 41.276f;

// Pressure transducer
float processPT(float voltagePt);

// Thermocouple
float readColdJunction(int thermPin);
float readDeltaTemp(float voltage);
float processTC(float voltageTc, int thermPin);

// Power control
void enablePower(int enablePin);
void disablePower(int enablePin);

// Valve control
void enableValve(int solEnPin);
void disableValve(int solEnPin);

// LED utilities
void blinkLed(int ledPin, int delayMs = 800);
void flashLeds(const int ledArray[], int ledCount);

#endif // PROP_TESTING_H
