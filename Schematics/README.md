# Propulsion Integration Schematics
This directory contains all schematics involved in this iteration of the Avionics Propulsion Integration system.
- All schematic and board files are created with **Kicad V8**.

Including: 

### Lower Control Board (Hydra)
- Launch control board for the **lower** valve bay. 
- Features valve control, analog sensing, and communication via CAN. 
- Interfaces with an STM32F103x8 microcontroller, controls two solenoids, reads from one Thermocouple, two Pressure Transducers, and two Hall-Effect sensors.

### Upper Control Board (Pegasus)
- Launch control board for the **upper** valve bay. 
- Features valve control, analog sensing, and communication via CAN **and** WIFI. 
- Interfaces with an ESP32-S3 microcontroller, controls two solenoids, reads two Pressure Transducers, and one Hall-Effect sensor.

### Hall Effect Breakout Board 
- Breakout board for a TMAG5273 Hall-Effect Sensor.
- Supports communication via I2C. 

### Shared Circuit Modules
- Circuitry used across both control boards.
- Solenoid control, pressure transducer and thermocouple entry circuitry. 


## Usage:
> ### To create a new board
1. Launch KiCad v8. 
2. Navigate to the **File** menu and select **New Project...** or use the command `Ctrl+N`. 
3. This will open the file explorer. Ensure you are in the `/Schematics` directory. 
4. Name the new board descriptively -- this is also the name of the KiCad project.
    - A **descriptive** name clearly states its function or category. 
    - A name that follows the theme of the rocket should be kept on the board or in schematic files -- not as the name of the KiCad Project. (i.e. "Hydra" for the lower launch control board is a theme-following name)
    - Descriptive shorthands like `lc` (Launch Control) can be helpful in naming boards/projects.
5. Before creating the project, ensure the *Create a new folder for project* box is selected in the bottom-right section of the file explorer. 
6. Press "Save" to create a new project. 
7. In the folder created by KiCad, it wil be helpful to create folders for component symbols, footprints, and models. 
   - You're all done -- it's also always smart to look at existing projects for additional guidance.
  
> ### To add symbols, footprints, and models
1. Download and unzip resources from a provider like [SnapMagic](https://www.snapeda.com/home/) , [UltraLibrarian](https://www.ultralibrarian.com/), etc.
2. Move your downloaded resources.
    - For **symbols** move the raw `kicad_sym` file into your `symbols` folder.
    - For **footprints** move the `kicad_mod` file that is **in** an appropiately named folder into your `footprints` folder.
    - For **models** move the raw `STEP` file into your `models` folder.
3. To import resources navigate to the **Preferences** menu and select **Manage Symbol Libraries** -- note that this wil be **Manage Footprint Libraries** if you are in the PCB editor.
4. Switch the tab to **Project-Specific Libraries** and from there select the insert & folder icon to add your resource via the file explorer. 
5. After adding your resource select save and you're good to select it in the editor. 
   
6. For importing **3D-models**, navigate to the associated footprint in the PCB editor. 
7. Select it and press `E` to open the properties menu. 
8. Navigate to the **3D Models** tab to add your model through the file explorer. 
9. Press save. `Ctrl+S`

--- 

QRET Avionics 25/26
