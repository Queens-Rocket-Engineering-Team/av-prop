# Propulsion Integration Firmware

Firmware for the Avionics Propulsion Integration system. Written in C/C++, built with PlatformIO.

Contents:
- **Libraries** — firmware modules shared across programs.
- **Board-Specific Testing Code** — per-board test programs.

## PlatformIO (VSCode)

1. Install the [PlatformIO IDE](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide) extension.
2. Open the `Firmware` folder — PlatformIO auto-detects each project's `platformio.ini`.

Build, flash, and monitor from the CLI (or the matching checkmark/arrow/plug toolbar icons):

```bash
pio run            # compile
pio run -t upload  # flash (USB for ESP, ST-Link for STM)
pio device monitor # serial monitor
```

---

QRET Avionics 25/26
