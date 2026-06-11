# Upper Control Board (PEGASUS) Documentation

<!--- removed left align div - weird github web issue --->
<details>
<summary><strong>Table of Contents</strong></summary>

- [Upper Control Board (PEGASUS) Documentation](#upper-control-board-pegasus-documentation)
  - [Overview](#overview)
    - [Key Features](#key-features)
  - [Circuit Design](#circuit-design)
    - [Power Management](#power-management)
      - [Power Entry](#power-entry)
      - [Power MUX](#power-mux)
      - [Power Switching](#power-switching)
    - [MCU Breakout](#mcu-breakout)
      - [Mode Selection](#mode-selection)
      - [CANBUS Interface](#canbus-interface)
      - [USB Interface](#usb-interface)
      - [System Feedback](#system-feedback)
    - [Valve Control](#valve-control)
    - [Sensing](#sensing)
      - [Data Acquisition](#data-acquisition)
      - [Pressure Transducers](#pressure-transducers)
  - [Board Design](#board-design)
    - [Physical Design Notes](#physical-design-notes)
    - [Electrical Characteristics](#electrical-characteristics)
    - [System Partitioning](#system-partitioning)
  - [Revision History](#revision-history)
  - [Contributors](#contributors)

</details>

---

## Overview

<p align="center">
  <img src="../../Datasheets/images/PEGASUS_Board_Front.png" width="38%">
  <img src="../../Datasheets/images/upper_board_3D.png" width="58%">
</p>

<em>Fig. 1: Pegasus</em>

The **Upper LC/Pegasus** board is a control board designed for integration with the rocket's propulsion system.
It operates in the **upper valve bay**, and drives a solenoid to control the rocket's vent valve.
This board supports analog pressure sensing, a digital hall-effect sensor for solenoid-status reading, and communication with the rest of the stack through CANbus. It is also **WiFi-capable**.

Interfaces with:
- ESP32-S3 Module
- Two 24V 205292 Solenoids
- Two 24V 4-20mA Pressure Transducers (PT)
- One TMAG273 digital hall-effect sensor (via breakout board)

### Key Features

- ESP32-S3 microcontroller with integrated WiFi
- I2C and SPI interfaces for sensor integration
- Controlled power delivery for sensors
- ORing circuits for power management
- Auditory and visual indication for status and debugging
- 4-layer PCB with dedicated ground planes
- Protection and filtering components across board

---

## Circuit Design

---

### Power Management

This board receives power from the Hybrid Power Module through the Backplane.
For more information on power delivery, check out [this repository](https://github.com/Queens-Rocket-Engineering-Team/av-power).

The supply delivers **three** levels of voltage to the board:

| Voltage (V) | Purpose |
|-------------|---------|
| 3.3 | Logic supply for microcontroller and other peripheral devices; also serves as a reference for signal conditioning and protection circuits |
| 5 | Intermediate supply for specific peripheral devices |
| 24 | High-voltage supply for driving external devices such as solenoids and sensors |

> **Note:** 3.3V is referred to as **3V3** throughout this document.

#### Power Entry

Power rails enter through a PCIe connector (Stack Edge Connector) and are conditioned by bulk filtering and input protection circuitry. These circuits apply to the 5V and 24V rails, each decoupled with 100 µF electrolytic capacitors to handle voltage transients and provide local energy storage. The 24V rail includes a 2A fuse for overcurrent protection.

Other power sources come from communication devices such as the USB connection, creating a voltage rail of 5V, which is then stepped down to 3V3 with a **Low-Dropout Regulator (LDO)**.

All rails feature test points and LEDs for diagnosis and debugging.

Additionally, the 24V rail uses a voltage divider for sensing. Two resistors (100 kΩ and 10 kΩ) scale the 24V input down to approx. 2.18V for measurement by an ESP32 ADC input:

$$V = 24 \times \frac{10k}{(100k + 10k)} \approx 2.18\ \text{V}$$

#### Power MUX

Power Multiplexers (MUX) are circuits that select between multiple power inputs to drive a single load. This is needed to avoid back-feeding between power sources, which could lead to unintended powering of interfaces and potential damage.

This implementation uses a **Dual OR-ing** circuit to select between two 3.3V inputs: Backplane (V_BP) and USB (V_USB).

- The circuit involves two Ideal Diode ICs, which use a MOSFET and controller to mimic the behaviour of an **Ideal Diode** (zero voltage drop when forward-biased, zero current when reverse-biased).
- The logic follows:
  - **ON condition**: A chip turns on if its V<sub>in</sub> is **higher** than the common output rail V<sub>out</sub>.
  - Each chip uses its enable *CE* pin to monitor the output rail -- if it sees a higher voltage present, it remains closed to prevent back-feeding.

In summary, a source will only drive the load if its voltage is higher than the current voltage on the output rail, allowing only **one** source to drive the output rail at a time.

- Capacitors are placed at the input pins to filter noise and prevent voltage spikes when a source is suddenly connected or disconnected.
- On the output rail, capacitors act as reservoirs to maintain steady voltage while also filtering noise generated by the loads connected to the 3V3 rail.

#### Power Switching

The delivery of 24V to the pressure transducers is controlled through a switching circuit.
- This was added to reduce power consumption by enabling sensor power only during active operation.
- The controlled output rail is identified as `VPT`.

The circuit uses a **CMOS** (dual-MOSFET) setup: one N-channel at the input to drive a P-channel for the output.

**MOSFET Basics**

| Type | ON Condition |
|------|-------------|
| N-Channel | Gate voltage higher than source -- V<sub>GS</sub> > V<sub>th</sub> |
| P-Channel | Gate voltage lower than source -- V<sub>GS</sub> < V<sub>th</sub> |

The **ON** state connects the **Drain** (output) pin to the **Source** pin.

**Logic**

- The ESP32 outputs a digital signal `VPT-SW` to drive the gate of the N-channel MOSFET. The source of the MOSFET is set to GND (0V).
- The gate is now at a higher voltage than the source, setting the drain to GND.
- The drain connects to a resistor divider with 24V to drive the gate of the P-channel MOSFET. The source of the MOSFET is set to 24V.
  - The resistor divider limits the gate voltage to approximately 14.81 V, lower than the source, setting the drain to 24V and powering the `VPT` rail.

$$V_g = 24 \times \frac{10k}{(6.2k + 10k)} \approx 14.81\ \text{V}$$

$$V_{gs} = 14.81 - 24 \approx -9.19\ \text{V}$$

> From the [DMP4065 Datasheet](${KIPRJMOD}/../../Datasheets/pMOSFET_DMP4065S-7_DS.pdf), V<sub>GS(max)</sub> = ±20 V -- within safe operating range.

- VPT is filtered with a Pi filter, featuring a ferrite bead between a 10 µF bulk capacitor and a 0.1 µF high-frequency decoupling capacitor to suppress both low-frequency ripple and high-frequency noise.

The circuit also includes two resistors at the MOSFET gate:
- **Series (Gate) Resistor**: Limits inrush current into the gate while controlling switching speed and protecting the MCU pin.
- **Pull-Down Resistor**: Connects signal to GND to define a default logic state when nothing is actively driving the gate.

---

### MCU Breakout

The [ESP32-S3](https://documentation.espressif.com/esp32-s3_datasheet_en.pdf) module was selected for this board's MCU.

Key features:
- Integrated 2.4 GHz WiFi
- Native USB Serial/JTAG controller
- Internal CAN controller (TWAI)
- Flexible GPIO matrix

#### Mode Selection

The S3 can be put into **BOOT** mode and **RESET** with onboard buttons.

**BOOT** mode determines whether the chip runs existing code from flash memory or waits to download new firmware.
- The ESP32 reads the state of the BOOT pin (IO0) during reset to select the appropriate boot mode.
- It is **active-low**: when pulled to GND, it will wait for new code over serial/USB.
- The circuit consists of a momentary push-button and a 0.1 µF capacitor to filter out button noise (debouncing).

**RESET** controls the Enable (EN) pin which is the ON/OFF (reset) switch for the ESP32.
- It is **active-low**: when pulled to GND it turns off the chip, and when released, it restarts.
- The circuit consists of a momentary push-button, a pull-up resistor to ensure the chip is enabled by default, and a 0.1 µF capacitor to filter out button noise (debouncing).

| BOOT0 Level | Boot Mode |
|-------------|-----------|
| HIGH | Programming Mode |
| LOW | Memory Mode |

#### CANBUS Interface

The S3 includes an internal CAN controller but requires a CAN transceiver to interface with the stack's CANBUS.
- Differential pairs from the backplane edge connector are routed to the CAN transceiver, which converts them into TX and RX signals for the S3.
- The CAN transceiver selected is a high-speed [TCAN1051](https://www.ti.com/lit/ds/symlink/tcan1051h.pdf).

#### USB Interface

The S3 includes an internal USB controller, eliminating any intermediate circuitry between the USB connector and S3.
- Differential pairs from the USB connector are routed directly to the S3.
- The S3 has an internal pull-up resistor on the positive USB data line.

#### System Feedback

This board incorporates visual and auditory feedback through LEDs and a buzzer.

- Visual indicators include WiFi status, CAN status, and a general DEBUG indicator.
  - These consist of standard current-driven LEDs and an ARGB LED.
- The buzzer provides general-purpose feedback when visual indication isn't available (in the stack).

---

### Valve Control

Valves are actuated via solenoids using MOSFET-based switching circuits driven by the MCU.
- The system uses a Festo 205292 solenoid to control the rocket vent valve.
  - Operates at 24V and is active-low.
  - Controlled via low-side switching (active when pulled to ground).
  - Interfaces through a MOLEX Nano-Fit connector.

The circuits consist of an N-channel MOSFET for low-side switching, flyback (Schottky) diodes for protection, and hardware-driven LEDs for visual indication.

> See the **MOSFET Basics** section under [Power Management](#power-management) for background on MOSFET operation.

**Logic**

- The solenoid positive terminal is connected to 24V.
- The STM32 outputs a digital signal that serves as the circuit's control input.
- This signal feeds an N-channel MOSFET with its source connected to GND (0V) and its drain connected to the negative terminal of the solenoid input (SOL_RET).
- The signal will turn the MOSFET `ON` since the gate voltage (3.3V) will be higher than the source voltage (0V).
- This connects the MOSFET drain to source, pulling the solenoid negative terminal to ground and actuating the valve.

**Protection & Filtering**

- A 40V, 3A SS34 Schottky diode is placed across the solenoid to provide a path for inductive flyback (voltage spike) current during turn-off.
  - This is common practice to protect against voltage spikes generated by inductive loads during switching.
- A local 0.1 µF capacitor is placed near the connector for high-frequency decoupling.

**Visual Indication**

- For indication and debugging a purely-hardware driven LED circuit was leveraged.
  - An LED with a series current-limiting resistor is placed across the solenoid, with the anode on the positive terminal and the cathode on the negative terminal.
  - By default, the anode is connected to 24V and the cathode is floating.
  - When the solenoid is driven and the negative terminal is pulled low, the LED is connected on both sides and emits.

---

### Sensing

This board interfaces with several external sensors, including analog pressure transducers and a digital hall-effect sensor.
- Pressure transducers are used for analog telemetry measurements.
- Hall-effect sensors are used for solenoid state detection and feedback.

#### Data Acquisition

Analog sensor data is acquired via an external **ADC** (Analog-to-Digital Converter) through the SPI protocol. Digital signals from hall-effect sensors are acquired through the I2C protocol.

The ADC used is the [ADS131M04](https://www.ti.com/lit/ds/symlink/ads131m04.pdf) -- 8 input channels, 24-bit resolution, 32 kSPS programmable sampling rate.
- Each input channel receives a voltage or differential voltage from a sensor (V<sub>ref</sub> = 1.2 V).
- A [delta-sigma](https://e2e.ti.com/blogs_/archives/b/precisionhub/posts/delta-sigma-adc-basics-understanding-the-delta-sigma-modulator) modulator converts the voltage into a high-frequency bitstream representing the input analog signal. This modulator is driven by an external crystal oscillator (8.319 MHz).
- The bitstream is then filtered to produce a clean, high-resolution digital value.
- The ADC sends a 24-bit number representing the measured voltage to the MCU via SPI.

An **RC filter** is used at the ADC inputs as a low-pass filter to smooth out high-frequency noise. The capacitor charges and discharges through the resistor, smoothing fast transients so the ADC sees a stable voltage.

> **Brief Overview on Analog Signals:** Analog signals vary continuously, taking any value within a given range, unlike digital signals which have discrete levels. Changes in a sensor's physical quantity are represented proportionally as a voltage or current signal.

#### Pressure Transducers

The PT circuits are built around signal conditioning and protection before the ADC.
- The PT used is a 4-20 mA [Setra 209H](https://www.setra.com/hubfs/Setra_Product_Data_Sheets/Model_209H_Spec_Sheet_2016_PRELIMINARY_WATERMARK.pdf) connected via Molex Nano-Fit.
- The signal line is initially referenced to GND through a 62 Ω shunt resistor to produce a voltage drop proportionate to the signal (current):

$$R_{shunt} = \frac{V_{ref}}{I_{max}} = \frac{1.2\ \text{V}}{0.02\ \text{A}} = 60\ \Omega$$

> A 62 Ω resistor was chosen for availability.

- The signal is then clamped to 3.3V and GND using low-leakage diodes.
- A 1 kΩ resistor is placed in series between the signal and diode to limit current into the diodes during overvoltage conditions, protecting the input from excessive current.
- The clamped signal then passes through a 200 Ω resistor connected to a 10 nF capacitor, forming the RC low-pass filter before being sampled by the ADC.

---

## Board Design

### Physical Design Notes

- The board measures 69.5 mm × 60 mm.
- Connectors are placed at the board edge for accessibility.

### Electrical Characteristics

The board is a 4-layer PCB with dedicated ground planes for noise reduction and signal integrity.

| Layer | Net | Purpose |
|-------|-----|---------|
| 1 | Signals + Power | Top-level routing |
| 2 | GND | Inner ground plane |
| 3 | GND | Inner ground plane |
| 4 | Signals + Power | Bottom-level routing |

- Inner ground planes minimize impedance in return paths and reduce EMI.
- Decoupling capacitors are placed close to IC power pins to reduce loop inductance.
- Valve control circuits use local ground pours to support high-current return paths (MOSFET source).
- The ESP32-S3 module requires a large local ground pour for stable grounding and RF performance.

### System Partitioning

- The board is partitioned into distinct analog, digital, and power domains to reduce noise coupling and improve signal integrity.
- Analog sensing circuitry is physically isolated from high-current valve control circuits.
- Power distribution is routed from the top of the board and propagates downward through regulated domains.
- High-current switching paths are separated from sensitive signal routing to minimize interference.

---

## Revision History

| Revision | Date | Description |
|----------|------|-------------|
| 0.0.1 | 2025-12-15 | Initial prototype with full peripheral integration |
| 0.0.2 | 2025-12-30 | Optimized routing and component placement |
| 1.1 | 2026-01-16 | Onboard trace cuts and adjustments to prevent short due to reversed diode in thermocouple circuitry |
| 2.0 | 2026-03-05 | Started prototype for second revision of board for manufacturing. Including fixed TC circuit, buzzer, and power control for PT supply |
| 2.1 | 2026-03-27 | Review sprint: route cleanup and silkscreen addition |

---

## Contributors

Jeevan Sanchez, Tristan Alderson

---

*QRET Avionics 25/26*
