/*
 * ADS131M04 ADC Library
 * Authors: Jeevan Sanchez
 * Hardware: TI ADS131M04 https://www.ti.com/lit/ds/symlink/ads131m04.pdf
 * Env: Arduino Framework (PlatformIO - STM32 & ESP32)
 * Created: Apr.23.2026
 * Updated: 
 * Purpose: SRAD Library for use across control boards
 * 
 * Note: Clock must be setup in user code before init() is called
 * 
 * QRET Avionics 2025-2026
*/

#include "ADS131M04.h"

ADS131M04::ADS131M04(int8_t _CS_PIN, int8_t _DRDY_PIN, SPIClass* _SPI) {
    csPin = _CS_PIN;
    drdyPin = _DRDY_PIN;
    spi = _SPI;
    initialized = false;
}

// setup the ADC
void ADS131M04::init() {
    pinMode(csPin, OUTPUT); 
    pinMode(drdyPin, INPUT_PULLUP);
    digitalWrite(csPin, HIGH); 

    spi->begin(); 
    
    initialized = true;
}

bool ADS131M04::readChannels(int32_t *readingsBuffer) {
    if (!initialized) return false; 
    
    uint32_t timeout = micros(); 
    while (digitalRead(drdyPin) == HIGH) { // monitor the DRDY (active-low) pin
        if (micros() - timeout > 2000) return false; // 2ms timeout
    }

    uint32_t frame[6]; 
    
    // pull the raw frame (status + 4 channels + CRC)
    captureFrame(frame, 0x0000); 

    // decode and store
    for (int i = 0; i < 4; i++) {

        // calculate signed value and store in internal cace for getChanelData()
        int32_t val = extendSign(frame[i+1]); 
        channels[i] = val; 

        if (readingsBuffer != nullptr) {
             readingsBuffer[i] = val; 
        }
    }

    return true;
}

// copy readings in voltage to an array
void ADS131M04::computeVoltages(const int32_t *raw, float* volts) {
    for (int i = 0; i < 4; i++) {
        volts[i] = ((float)raw[i] / resFactor) * voltageRef;
    }
}

// fetch data from a given channel
int32_t ADS131M04::getChannelData(uint8_t ch) {
    return channels[ch]; 
}

// send a 16-bit command over SPI while reading a 24-bit response and package it into a 32-bit variable
uint32_t ADS131M04::spiTransfer(uint16_t cmd) {
    uint32_t response = 0; 

    response |= spi->transfer(cmd >> 8); // send command MSB to receive 1 byte from ` 
    response <<= 8; 
    response |= spi->transfer(cmd & 0XFF); // send command LSB
    response <<= 8;
    response |= spi->transfer(0x00); // dummy write to get final byte

    return (response & 0xFFFFFF); // mask to ensure 24 bits
}

// perform a single transaction by reading an SPI frame
/*
* Consists of:
    * 1 Command
    * 4 Channnel data words
    * 1 CRC
*/
void ADS131M04::captureFrame(uint32_t *outputArr, uint16_t command) {
    if (csPin != -1) digitalWrite(csPin, LOW); // enable the ADC (if CS is connected)
    spi->beginTransaction(SPISettings(SPEED_SCLK, MSBFIRST, SPI_MODE1));

    outputArr[0] = spiTransfer(command); // store command response

    for (int i = 1; i < 5; i++) {
        outputArr[i] = spiTransfer(0X0000); // store channel data (CH0-CH3)
    }
 
    outputArr[5] = spiTransfer(0x0000); // store CRC reading

    spi->endTransaction();
    if (csPin != -1) digitalWrite(csPin, HIGH);
}

// decode 2's complement
int32_t ADS131M04::extendSign(uint32_t data) {

    // cast and mask
    int32_t val = (int32_t)(data & 0XFFFFFF); 

    val <<= 8; 
    val >>= 8; 

    return val;
}


