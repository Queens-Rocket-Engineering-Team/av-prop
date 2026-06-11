# Propulsion Systems Testing Firmware Library  
> QRET Avionics 2025–2026  

A lightweight utility library used for **propulsion control board testing** on HYDRA (STM32) and PEGASUS (ESP32) platforms.

---

## Features

### Power Control
- `enablePower(pin)`
- `disablePower(pin)`

### Valve / Actuation
- `enableValve(pin)`
- `disableValve(pin)`

### ADC Support
- `adcSetup(adc)`
- `readAllADC(adc, buffer)`

### Sensor Conversions
- `processPT(voltage)` → 4–20mA pressure to PSI  
- `readColdJunction(pin)` → thermistor temperature (°C)  
- `readDeltaTemp(voltage)` → thermocouple delta temperature  
- `processTC(voltage, pin)` → compensated thermocouple temperature  

### Debug Utilities
- `blinkLed(pin, delayMs)`
- `flashLeds(pinArray, count)`

---

## Hardware

- ADS131M04 external ADC  
- K-type thermocouples with cold junction compensation  
- 4–20mA pressure transducers  
- STM32 / ESP32 control boards  
- Solenoid valves and test actuators  

---

## Notes

- ADC scaling and VREF assumptions are platform-dependent (STM32 vs ESP32).
- All sensor conversion functions assume externally defined calibration constants.
- This library does not implement control logic—only hardware interaction and conversion utilities.

---

## QRET Avionics 2025–2026