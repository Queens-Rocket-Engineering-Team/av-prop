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
constexpr uint8_t CAN_TX_PIN = 1;
constexpr uint8_t CAN_RX_PIN = 2;
constexpr uint8_t SENSE_24V_PIN = 7;
constexpr uint8_t SOL1_EN_PIN = 9;
constexpr uint8_t SOL2_EN_PIN = 10;
constexpr uint8_t VPT_EN_PIN = 47;

// ADC SPI
constexpr uint8_t ADC_CLKIN_PIN = 11;
constexpr uint8_t ADC_MOSI_PIN = 12;
constexpr uint8_t ADC_MISO_PIN = 13;
constexpr uint8_t ADC_SCLK_PIN = 14;
constexpr uint8_t ADC_DRDY_PIN = 19;

// HALL I2C
constexpr uint8_t HALL_SCL_PIN = 17;
constexpr uint8_t HALL_SDA_PIN = 18;

// LEDs
constexpr uint8_t WIFI_LED_PIN = 35;
constexpr uint8_t CAN_LED_PIN = 36;
constexpr uint8_t DEBUG_LED_PIN = 37;

constexpr uint8_t BUZZ_EN_PIN = 41;
constexpr uint8_t RGB_DATA_PIN = 42;

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

enum BoardEndpointId : uint8_t {
  BOARD_ENDPOINT_SYSTEM = 0U,
  BOARD_ENDPOINT_TC = 1U,
  BOARD_ENDPOINT_PT1 = 2U,
  BOARD_ENDPOINT_PT2 = 3U,
  BOARD_ENDPOINT_V_SOL = 4U, 
  BOARD_ENDPOINT_V_PT = 5U, 
  BOARD_ENDPOINT_VALVE1 = 6U,
  BOARD_ENDPOINT_VALVE2 = 7U
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
