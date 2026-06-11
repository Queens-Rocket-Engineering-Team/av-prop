/*
 * Propulsion Systems Testing Firmware
 * Authors: Jeevan Sanchez, Tristan Alderson
 * Hardware: Lower LC Board (HYDRA), Upper LC Board (PEGASUS)
 * Env: PlatformIO (STM32 and ESP32)
 * Created: ~Feb.12.2026
 * Updated: May.7.2026
 * Purpose: SRAD firmware for peripheral testing of prop. control boards.
 * 
 * QRET Avionics 2025-2026
*/
#include <prop_testing.h>

// POWER ===================
// controlled power delivery
void enablePower(const int enablePin) {
    digitalWrite(enablePin, HIGH);
    delay(100); // inrush delay
}

void disablePower(const int enablePin) {
    digitalWrite(enablePin, LOW); 
}

// ADC ===================
// configure an ADS131M04 ADC with SPI
void adcSetup(ADS131M04& adc) {
    // clock setup is handled in board-specific files BEFORE this function is called 
    
    adc.init();
    delay(100); 
}

// PRESSURE TRANSDUCER ===================
// returns and optionally logs PSI reading from converted ADC reading voltage
// optional DEBUG mode (single ADC read) or writes to a buffer
// uses pressure linear scaling
float processPT(float voltagePT) {
    float current = (voltagePT / shuntResistance) *  1000.0; // current in mA
    float PSI = (current - 4.0) * (maxPSI / (16.0)); // P = (I - I_min) * (P_max / (I_max - I_min)) | for 4-20 mA PT (16 = 20 - 4)
    return PSI;
} 

// THERMOCOUPLE ===================
//  Steinhart-Hart B-parameter equation:
//  1/T = 1/T0 + (1/B) * ln(R / R0)
float readColdJunction(const int TEMP_SENSE) {
    uint32_t raw = analogRead(TEMP_SENSE); // read from CJC Thermistor

    // Convert ADC count to thermistor resistance via voltage divider
    float vOut = (raw / STM32_ADC_RES) * STM32_VREF;           // voltage at pin
    float rTherm = THERM_R_SERIES * vOut / (STM32_VREF - vOut); // NTC resistance (Ω)

    // B-parameter equation -> temperature in Kelvin
    float tKelvin = 1.0 / (1.0 / THERM_T0 + (1.0 / THERM_B) * log(rTherm / THERM_R0));

    return tKelvin - 273.15; // convert to Celsius
}

// --- K-type Thermocouple Temperature ---
// cold junction compensation (CJC):
//   T_hot = T_cold + (V_tc / Seebeck)
// voltage in Volts, returns temperature in Celsius.
float readDeltaTemp(float voltage) {
    float microVoltage = voltage * 1e6; // V -> µV
    return (float)microVoltage / K_TYPE_SEEBECK_UV_PER_C; // µV / (µV/°C) = °C 
    // return deltaTemp
}

// returns and optionally logs temp reading from converted ADC reading voltage
float processTC(float voltageTC, const int thermPin) {
    float deltaTemp = readDeltaTemp(voltageTC); 
    float coldJunctionTemp = readColdJunction(thermPin); 
    float compensatedTemp = coldJunctionTemp + deltaTemp;
    return compensatedTemp; 
}

// VALVE CONTROL ===================
// actuate a solenoid for a custom duration or two seconds (when not specified)
// solID refers to the connector designator (on PCB), defaults to one
void enableValve(const int solENPin) {
    digitalWrite(solENPin, HIGH);
}

void disableValve(const int solENPin) {
    digitalWrite(solENPin, LOW); 
}

// UTILS ===================
// blink a single LED
void blinkLed(int ledPin, int delayMs) {
    digitalWrite(ledPin, HIGH);
    delay(delayMs); 
    digitalWrite(ledPin, LOW);
    delay(delayMs);
}

// blink several LEDs with an array of pins
void flashLeds(const int ledArray[], int ledCount) {
    for (int i = 0; i < ledCount; i++) {
        digitalWrite(ledArray[i], HIGH);
        delay(800);
        digitalWrite(ledArray[i], LOW);
    }
}







