#ifndef NODE_H
#define NODE_H

#include <Arduino.h>
#include <cstddef>
#include <cstdint>

#include <aim_can_driver.h>
#include <aim_network.h>
#include <aim_safety.h>

// ── Pin Definitions ──────────────────────────────────────────────────
constexpr uint8_t CAN_TX_PIN = 1;
constexpr uint8_t CAN_RX_PIN = 2;
constexpr uint8_t SENSE_24V_PIN = 7;
constexpr uint8_t SOL1_EN_PIN = 9;
constexpr uint8_t SOL2_EN_PIN = 10;

// ADC SPI
constexpr uint8_t ADC_CLKIN_PIN = 11;
constexpr uint8_t ADC_MOSI_PIN = 12;
constexpr uint8_t ADC_MISO_PIN = 13;
constexpr uint8_t ADC_SCLK_PIN = 14;

// HALL I2C
constexpr uint8_t HALL_SCL_PIN = 17;
constexpr uint8_t HALL_SDA_PIN = 18;

constexpr uint8_t ADC_DRDY_PIN = 19;

// LEDs
constexpr uint8_t WIFI_LED_PIN = 35;
constexpr uint8_t CAN_LED_PIN = 36;
constexpr uint8_t DEBUG_LED_PIN = 37;

constexpr uint8_t BUZZ_EN_PIN = 41;
constexpr uint8_t RGB_DATA_PIN = 42;
constexpr uint8_t VPT_EN_PIN = 47;

// ── Node Identity & Configuration ────────────────────────────────────
#define NODE_ORIGIN AIM_ORG_UPROP
#define NODE_NAME "UPPER_CONTROL"

#define NODE_CAN_BAUD 500000U

#define NODE_SERIAL_BAUD 115200U

#define WIFI_SSID "TELUS1917"
#define WIFI_PASS "s24ec9424u"

static constexpr uint8_t NODE_ENDPOINT_LOWER_BASE = 16U;

enum NodeEndpointId : uint8_t {
  NODE_ENDPOINT_SYSTEM = 0U,
  NODE_ENDPOINT_PT1 = 1U,
  NODE_ENDPOINT_PT2 = 2U,
  NODE_ENDPOINT_VALVE1 = 4U,
  NODE_ENDPOINT_VALVE2 = 5U
};

enum NodeState : uint8_t {
  INIT = 0U,
  OPERATIONAL = 1U,
  DEBUG_CONSOLE = 2U,
  FLASH_DUMP = 3U,
  FLASH_ERASE = 4U,
  SAFE_MODE = 5U,
  LOW_POWER = 6U,
  FAULT = 7U
};

enum NodeActuatorCommand : uint32_t {
  NODE_ACTUATOR_CLOSED = 0U,
  NODE_ACTUATOR_OPEN = 1U
};

bool nodeInitHardware(void);
void nodeServiceLocalTelemetry(uint32_t schedulerNowMs, uint32_t networkNowMs, AimNetwork& aim);
bool nodeHandleCanPacket(const aimPkt& pkt, uint32_t networkNowMs, AimNetwork& aim);

// Add node-specific periodic behavior in nodeUpdate().
void nodeUpdate(uint32_t schedulerNowMs);

void nodeStartTasks(AimNetwork& aim);

#ifndef FLIGHT_BUILD
void nodePrintNetworkStatus(Print& out);
void nodePrintCanStatus(Print& out);
void nodePrintSensorStatus(Print& out);
bool nodeSetValveStateDirect(uint8_t index, bool open);
bool nodeGetValveState(uint8_t index);
#endif

#endif  // NODE_H
