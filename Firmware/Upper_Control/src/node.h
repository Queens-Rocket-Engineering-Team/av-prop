#ifndef NODE_H
#define NODE_H

#include <Arduino.h>
#include <cstdint>

#include <aim_can_driver.h>
#include <aim_network.h>
#include <aim_safety.h>

#include "pinouts.h"

namespace node {
constexpr char     kName[]      = "PEGASUS";
constexpr uint32_t kCanBaud     = 500000U;
constexpr uint32_t kSerialBaud   = 115200U;
}  // namespace node

// Standard telemetry columns for flight recording
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

// Standard node interface
void nodeInit(uint32_t nowMs);
void nodeUpdate(uint32_t schedulerNowMs);
void nodeServiceCanTx(uint32_t schedulerNowMs, AimNetwork& aim);
void nodeOnRx(const aim::Msg& m, uint32_t nowMs);

aim::NodeState nodeCurrentState();
uint16_t nodeErrorBits();

// Read-only accessors for logging, console, and QLCP
bool nodeGetValveState(uint8_t index);
bool nodeGet24vFetState(uint8_t index);
float nodeGetPtValue(uint8_t index);
float nodeGetHallEffect(uint8_t index);
float nodeGet24vSense(uint8_t index);
float nodeGetThermocouple();

#ifndef FLIGHT_BUILD
#include <aim_console.h>
const AimConsoleHook* nodeConsoleHooks(uint8_t& count);
#endif

#endif  // NODE_H
