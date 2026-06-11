# TMAG5273 Low-Power Linear 3D Hall-Effect Sensor Library
> QRET Avionics 2025-2026 

A C++/Arduino library for interfacing with the TI TMAG5273A1.

## Features 
- **3-Axis Sensing:** Full magnetic flux density retrieval (X, Y, Z) in mT. 
- **Internal Temperature Compensation:** Integrated temperature sensing for environmental control and reading offsets. 
- **Platform Compatibility:** Designed for **ESP32** and **STM32** architectures via the `TwoWire` interface.
  
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

| TMAG5273 Pin | Function              |  Connection | 
|--------------|-----------------------|----------------|
| VCC          | Power (2.3V - 3.6V)   | 3V3            | 
| SDA          | I2C Data              | MCU I2C SDA pin| 
| SCL          | I2C Clock             | MCU I2C SCL pin| 
| GND          | Ground                | GND            | 

**Note:** Ensure 2.2kΩ - 10kΩ pull-up resistors are present on the SDA and SCL lines. 

---

## Quick Start 
First, import libraries and create an instance of the `TMAG5273` class to the implementation. 
```cpp
#include <TMAG5273.h>
#include <Wire.h>

TMAG5273 sensor;
```

In the implementation's setup section, initialize the I2C bus and the sensor. 
Also configure the serial port for logging.
```cpp
Wire.begin(); // call with specific SCL/SDA pins if not using your MCU's defaults.
Serial.begin(9600); // adjust baud rate as needed. 

if (!sensor.init(0x35, Wire)) {
    Serial.println("TMAG5273 not detected"); 
    while (1); // halt the program to terminate running without a valid connection. 
};
```

Here, the `setAveraging(uint8_t mode)` method can be used to configure the averaging speed. 
By default, it is set to 2x average. 

In the implementation's main loop, retreive and log data from the sensor with the `getAllFlux(float* axes)` and `getTemp()` methods.  
```cpp 
float axes[3]; // store the flux readings for each axis 

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

```
Check out the [BasicRead](/Firmware/libraries/TMAG5273/examples/BasicRead/basic_read.cpp) example to run this. 

---

## API Reference
`bool init(uint8_t addr, TwoWire &port)`
Initializes the sensor, verifies the Manufacturer ID (TI), and configures the device for continuous 3-axis conversion at $\pm$ 80 mT. 
- **Default Address**: `0x35` (A1 model). 
- **Returns:** `true` if initialization and self-check (Device Status) succeed. 

---

`bool getAllFlux(float* axes)`
Performs a burst read of the X, Y, and Z magnetic registers and writes result to a float array of size 3.
- **Returns:** `true` if the I2C transaction succeeds. 
- Results are in milli-Tesla (mT).

---

`float getFluxX()`, `float getFluxY()`, `float getFluxZ()`
Returns the magnetic flux density for a specific axis in milli-Tesla (mT). 
- **Returns:** The flux value as a `float`. Returns `NAN` if the sensor read fails. 
- **Note:** These methods call `getAllFlux()` internally. 

---

`float getTemp()`
Reads the internal temperature sensor. 
- **Returns:** Die temperature in Celsius as a `float`. Returns `NAN` on communication failure. 

---

`uint8_t getDeviceStatus()`
Reads the `DEVICE_STATUS` register (bits 3:0) to scan for hardware-level errors. 
- **Returns:**
  - `0X00`: No errors. 
  - `bit 3`: Oscillator error.
  - `bit 2`: Interrupt error. 
  - `bit 1`: OTP CRC error. 
  - `bit 0`: VCC undervoltage. 
  - `0xFF` I2C Read Failure.

---

`bool setAveraging(uint8_t mode)`
Configures the internal digital averaging filter. 
- **Modes:**
  - `0x00` No averaging (1x)
  - `0x01`: 2x averaging
  - `0x02`: 4x averaging
  - Refer to Table 8-3 in [Datasheet](https://www.ti.com/lit/ds/symlink/tmag5273.pdf) for higher modes. 
- **Returns:** `true` if the configuration write succeeds. 

---

QRET Avionics 25/26

