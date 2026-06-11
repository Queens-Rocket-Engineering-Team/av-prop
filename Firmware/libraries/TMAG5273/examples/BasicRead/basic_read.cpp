/*
 * Simple Read Example for TMAG5273
 * 
 * Example developed for ESP32
           * For STM32, declare a SoftwareSerial instance for printing if needed
 * 
 * QRET Avionics 2025-2026
*/

#include <TMAG5273.h>

// create sensor object
TMAG5273 sensor; 

// UPDATE with board SCL/SDA pins
const int SCL_PIN = 0; 
const int SDA_PIN = 0; 

void setup() {
    Serial.begin(9600);

    Wire.begin(); // call with SCL/SDA_PINs as arguments if not using default pins 

    // initialize sensor
    if (!sensor.init(0x35, Wire)) {
        Serial.println("TMAG5273 init failed"); 
        while (1) delay(100); 
    }

    Serial.println("TMAG5273 online");
}

void loop() {
    float axes[3]; 
    
    // obtain and log magnetic flux data 
    if (sensor.getAllFlux(axes)) {
        Serial.print("X: "); 
        Serial.print(axes[0], 2); 
        
        Serial.print(" | Y: "); 
        Serial.print(axes[1], 2); 

        Serial.print(" | Z: "); 
        Serial.print(axes[2], 2); 
        
        Serial.println(" mT"); 
    }

    // obtain and log temperature data
    float temp = sensor.getTemp(); 
    if (!isnan(temp)) {
        Serial.print(" | Temp: "); 
        Serial.print(temp, 2); 
        Serial.println(" C");
    }

    delay(250); 
}