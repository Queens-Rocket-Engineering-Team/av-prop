# Propulsion Integration Firmware
This directory contains all firmware involved in this iteration of the Avionics Propulsion Integration system.
- All firmware is written in C/C++. 
- PlatformIO is primarily used for flashing code.
- Support for the Arduino IDE is documented below.  
  

Including: 

### Libraries
- Firmware modules for use across programs. 

### Board-Specific Testing Code
- Testing code for different propulsion boards.

## Usage:

> ### To Setup PlatformIO (VSCode)
1. Install the [**PlatformIO IDE**](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide) extension from the VSCode marketplace. 
2. Ensure you are in the `Firmware` folder -- PlatformIO will automatically detect the `platform.ini` config file for setup. 

> ### To Flash with PlatformIO
1. Plug in your board (USB for ESP, ST-Link for STM). 
2. Build & Upload
   - Click the **checkmark** to **compile** code.
   - Click the **arrow** to **flash** the firmware to the board.
     - In the **CLI** run `pio run -t upload`
3. Click the **plug** icon to open the **Serial Monitor**.
     - In the **CLI** run `pio device monitor` 

> ### To Configure Arduino IDE Environment for ESP32-S3-based Boards
  1. Ensure your [Arduino IDE](https://support.arduino.cc/hc/en-us/articles/360019833020-Download-and-install-Arduino-IDE) is up-to-date.   
  2.  Open the *Preferences* menu (Ctrl+Comma), and add the link below into the *Additional Boards Manager* field. 
https://espressif.github.io/arduino-esp32/package_esp32_index.json 

  1. Navigate to the *Boards Manager* (Ctrl+Shift+B), and download the package **esp32** by **Espressif Systems**. 
  2. Once installed, open the *Tools* menu, and select the *Board* field, under the library you just installed (esp32), select the board: **ESP32S3 Dev Module**. 
  3. After selecting your board, you will see more fields under the *Tools* menu, find the field **USB CDC On Boot**, and **ENABLE** it. 
  4. Make sure you've selected the corrected port and upload!


--- 

QRET Avionics 25/26
