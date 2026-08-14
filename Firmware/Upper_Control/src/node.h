#ifndef NODE_H
#define NODE_H

#include <Arduino.h>
#include <cstdint>

#include <aim_network.h>
class AimFlightRecorder;

#include "pinouts.h"

namespace node {
constexpr char kName[] = "PEGASUS";
constexpr aim::Source kSource = aim::Source::Ucm;
constexpr uint32_t kCanBaud = 1000000U;
constexpr uint32_t kSerialBaud = 115200U;
}  // namespace node

static constexpr uint8_t  kLogCols           = 3U;
static constexpr uint16_t kLogOriginRefresh  = 100U;
static constexpr uint32_t kLogMaxSize        = 0;
static const char* const  kLogHeaders[kLogCols] = {"time", "solenoid0", "solenoid1"};

// Standard node interface
extern AimNetwork g_aim;
void nodeInit();
void nodeUpdate(uint32_t nowMs);
void nodeServiceLog(uint32_t nowMs, AimFlightRecorder& recorder);
void nodeServiceCanTx(uint32_t nowMs, AimNetwork& aim);
void nodeOnRx(const aim::Msg& m, uint32_t nowMs);
aim::NodeState nodeCurrentState();
uint16_t nodeErrorBits();

#ifndef FLIGHT_BUILD
#include <aim_console.h>
const AimConsoleHook* nodeConsoleHooks(uint8_t& count);
#endif

#endif  // NODE_H
