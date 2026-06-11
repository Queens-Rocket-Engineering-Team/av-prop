# ADS131M04 ADC Library
> QRET Avionics 2025-2026

A C++/Arduino library for interfacing with the TI ADS131M04 24-bit precision delta-sigma ADC.

## Features
- **4-Channel Acquisition:** Simultaneous sampling of all four ADC channels.
- **24-Bit Resolution:** Full signed 24-bit conversion support with 2's complement decoding.
- **Voltage Conversion:** Built-in raw-to-voltage conversion utilities.
- **DRDY Synchronization:** Uses the ADC DRDY pin for synchronized sampling.
- **Platform Compatibility:** Designed for **ESP32** and **STM32** architectures via the Arduino `SPIClass` interface.

---

## Installation

#### PlatformIO
Add the following to your `platformio.ini` file:

```ini
lib_deps =
    https://github.com/Queens-Rocket-Engineering-Team/av-prop.git#main
```

#### Manual Installation
1. Download this repository
2. Place the folder into your project's `lib/` directory.

---

## Wiring Reference

| ADS131M04 Pin | Function                | Connection |
|----------------|-------------------------|------------|
| AVDD / DVDD    | Power Supply            | 3.3V       |
| DGND / AGND    | Ground                  | GND        |
| SCLK           | SPI Clock               | MCU SPI SCK pin |
| DIN            | SPI MOSI                | MCU SPI MOSI pin |
| DOUT           | SPI MISO                | MCU SPI MISO pin |
| CS             | Chip Select             | MCU GPIO output |
| DRDY           | Data Ready (active-low) | MCU GPIO input |

**Note:** Ensure the ADC clock source is configured before calling `init()`.

---

## Quick Start

First, import the required libraries and create an instance of the `ADS131M04` class.

```cpp
#include <ADS131M04.h>
#include <SPI.h>

// UPDATE with board ADC SPI pins
#define ADC_CS_PIN      0
#define ADC_DRDY_PIN    0

ADS131M04 adc(ADC_CS_PIN, ADC_DRDY_PIN, &SPI);
```

In the implementation's setup section, initialize the SPI bus, ADC, and serial port.

```cpp
SPI.begin();

Serial.begin(115200);
delay(1000);

adc.init();

Serial.println("ADS131M04 initialized");
```

In the implementation's main loop, read the ADC channels and convert the readings into voltages.

```cpp
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
```

Check out the [BasicRead](/Firmware/libraries/ADS131M04/examples/BasicRead/basic_read.cpp) example to run this.

---

## API Reference

`ADS131M04(int8_t csPin, int8_t drdyPin, SPIClass* spi)`
Constructs an ADS131M04 object.
- `csPin`: SPI chip-select pin.
- `drdyPin`: Data-ready interrupt pin.
- `spi`: Pointer to the SPI interface.

---

`void init()`
Initializes GPIO and SPI communication for the ADC.
- Configures:
  - CS as output
  - DRDY as input pull-up
  - SPI peripheral
- Must be called before any read operation.

---

`bool readChannels(int32_t *readingsBuffer)`
Reads all four ADC channels from a single SPI frame transaction.
- Waits for DRDY to assert low before sampling.
- Stores signed 24-bit ADC readings into the provided buffer.
- **Returns:** `true` on successful read, `false` on timeout or initialization failure.

---

`void computeVoltages(const int32_t *raw, float* volts)`
Converts raw ADC readings into voltage values.
- `raw`: Pointer to signed ADC readings.
- `volts`: Output array for converted voltage values.

---

`int32_t getChannelData(uint8_t ch)`
Returns the most recent cached reading for a given ADC channel.
- `ch`: Channel index (`0-3`)
- **Returns:** Signed 24-bit ADC reading.

---

QRET Avionics 25/26