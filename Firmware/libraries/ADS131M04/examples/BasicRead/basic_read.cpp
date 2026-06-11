/*
 * Simple Read Example for ADS131M04
 * 
 * Example developed for ESP32
           * For STM32, declare a SoftwareSerial instance for printing if needed
 * 
 * QRET Avionics 2025-2026
*/

#include <ADS131M04.h>

// UPDATE with board ADC SPI  pins
#define ADC_CS_PIN 0
#define ADC_DRDY_PIN 0

// create ADC object
ADS131M04 adc(ADC_CS_PIN, ADC_DRDY_PIN, &SPI);

void setup() {
    Serial.begin(9600);
    delay(1000); 

    // initialize SPI
    SPI.begin();

    // initialize ADC
    adc.init();

    Serial.println("ADS131M04 initialized");
}

void loop() {

    // reading buffers
    int32_t raw[4];
    float volts[4]; 

    // read all 4 channels 
    if (adc.readChannels(raw)) {
        
        // convert raw readings to voltages 
        adc.computeVoltages(raw, volts); 

        // print channel data
        for (int i = 0; i < 4; i++) {

            Serial.print("CH");
            Serial.print(i);

            Serial.print(": ");

            // raw ADC reading
            Serial.print(raw[i]);
            Serial.print(" raw");

            Serial.print(" | ");

            // voltage value
            Serial.print(volts[i], 6);
            Serial.println(" V");
        }

        Serial.println();
    }

    else {
        Serial.println("ADC read timeout");
    }
    
    delay(250);

    }