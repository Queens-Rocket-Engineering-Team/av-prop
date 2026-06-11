#ifndef BOARD_H
#define BOARD_H

#include <Arduino.h>
#include <cstddef>
#include <cstdint>

#include <aim_can_driver.h>
#include <aim_network.h>
#include <aim_safety.h>
#include <AimNodeConfig.h>

// Pin Definitions
constexpr uint8_t kCanTxPin = 1;
constexpr uint8_t kCanRxPin = 2;
constexpr uint8_t kSense24vPin = 7;
constexpr uint8_t kSol1EnPin = 9;
constexpr uint8_t kSol2EnPin = 10;
constexpr uint8_t kVptEnPin = 47;

// ADC SPI
constexpr uint8_t kAdcClkinPin = 11;
constexpr uint8_t kAdcMosiPin = 12;
constexpr uint8_t kAdcMisoPin = 13;
constexpr uint8_t kAdcSclkPin = 14;
constexpr uint8_t kAdcDrdyPin = 19;

// HALL I2C
constexpr uint8_t kHallSclPin = 17;
constexpr uint8_t kHallSdaPin = 18;
constexpr uint8_t kHallI2cAddr = 0x35U;

// LEDs
constexpr uint8_t kWifiLedPin = 35;
constexpr uint8_t kCanLedPin = 36;
constexpr uint8_t kDebugLedPin = 37;

constexpr uint8_t kBuzzEnPin = 41;
constexpr uint8_t kRgbDataPin = 42;

// board Identity & Configuration
#define BOARD_ORIGIN aim::Node::UProp
#define BOARD_NAME "PEGASUS"

enum BoardTelemetryCol {
  BOARD_LOG_TIME_MS,
  BOARD_LOG_PT1_PSI,
  BOARD_LOG_PT2_PSI,
  BOARD_LOG_24V_SENSE,
  BOARD_LOG_HALL,
  BOARD_LOG_RSSI,
  BOARD_LOG_VPT_FET,
  BOARD_LOG_V1_FET,
  BOARD_LOG_V2_FET,
  BOARD_LOG_COL_COUNT
};

static constexpr const char* kBoardTelemetryHeaders[BOARD_LOG_COL_COUNT] = {
  "Time_ms",
  "PT1_PSI",
  "PT2_PSI",
  "24V_SEN",
  "HALL",
  "RSSI",
  "VPT_FET",
  "V1_FET",
  "V2_FET",
};

using BoardConfig = AimNodeCfg;

extern BoardConfig g_boardConfig;

// Live state owned by board.cpp.
// Valves/PTs: [0..1] local (upper), [2..3] mirrored from Lower Control over CAN.
// Relays: [0] UPPER_V_PT, [1] LOWER_V_PT, [2] LOWER_V_SOL.
// Hall: [0] UPPER_HALL1, [1] LOWER_HALL1, [2] LOWER_HALL2.
// VoltageSense: [0] UPPER_24V_SENSE, [1] LOWER_VSOL_SENSE.
extern bool  g_valveStates[4];
extern bool  g_24VoltageFet[3];
extern float g_ptValues[4];
extern float g_thermocouple;
extern float g_hallEffect[3];
extern float g_24VoltageSense[2];

#define BOARD_CAN_BAUD 500000U

#define BOARD_SERIAL_BAUD 115200U

#define WIFI_SSID "TELUS1917"
#define WIFI_PASS "s24ec9424u"

static constexpr uint8_t BOARD_ENDPOINT_LOWER_BASE = 8U;

/**
 * Hydra (Lower) -> Upper Wire Contract
 * -----------------------------------
 * All packets from Hydra use endpoint IDs >= BOARD_ENDPOINT_LOWER_BASE (8).
 * IDs 0-3 in the lower space map to wire IDs 8-11.
 * 
 * | Wire EP | PacketType | Payload          | Sink                 |
 * |---------|------------|------------------|----------------------|
 * | 9       | Sensor     | float °C (TC)    | g_thermocouple       |
 * | 10/11   | Sensor     | float PSI        | g_ptValues[2/3]      |
 * | 10/11   | Valve      | 0/1 echo         | g_valveStates[2/3]   |
 * | 12      | Sensor     | float V (VSOL)   | g_24VoltageSense[1]  |
 * | 12      | Valve      | 0/1 VSOL FET     | g_24VoltageFet[2]    |
 * | 13      | Valve      | 0/1 VPT FET      | g_24VoltageFet[1]    |
 * | 16/17   | Sensor     | float (Hall)     | g_hallEffect[1/2]    |
 * 
 * Sensor endpoints sent every 100 ms. 
 * Valve state on change + echo on command receipt.
 */
enum BoardEndpointId : uint8_t {
  BOARD_ENDPOINT_SYSTEM = 0U,
  BOARD_ENDPOINT_TC     = 1U,
  BOARD_ENDPOINT_PT1    = 2U,
  BOARD_ENDPOINT_PT2    = 3U,
  BOARD_ENDPOINT_V_SOL  = 4U, 
  BOARD_ENDPOINT_V_PT   = 5U, 
  BOARD_ENDPOINT_VALVE1 = 6U,
  BOARD_ENDPOINT_VALVE2 = 7U,
  BOARD_ENDPOINT_HALL1  = 8U,  // wire endpoint 16 (LOWER_BASE + 8)
  BOARD_ENDPOINT_HALL2  = 9U   // wire endpoint 17
};

enum BoardState : uint8_t {
  INIT = 0U,
  OPERATIONAL = 1U,
  DEBUG_CONSOLE = 2U,
  LOW_POWER = 3U,
  FAULT = 4U
};

enum BoardActuatorCommand : uint32_t {
  BOARD_ACTUATOR_CLOSED = 0U,
  BOARD_ACTUATOR_OPEN = 1U
};

bool boardInitHardware(void);
void boardServiceTx(uint32_t schedulerNowMs, uint32_t networkNowMs, AimNetwork& aim, uint32_t boardState);
bool boardHandleCanPacket(const aim::Pkt& pkt, uint32_t networkNowMs, AimNetwork& aim);

// Add board-specific periodic behavior in boardUpdate().
void boardUpdate(uint32_t schedulerNowMs);

// Kicks off the WiFi/QLCP connection state machine; the link is serviced
// non-blockingly from boardUpdate() every loop tick.
void boardStartNetwork(void);

#ifndef FLIGHT_BUILD
void boardPrintNetworkStatus(Print& out);
bool boardSetValveStateDirect(uint8_t index, bool open);
#endif

#endif  // BOARD_H
