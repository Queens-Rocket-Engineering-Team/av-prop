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

static constexpr uint8_t  kLogCols           = 12U;
static constexpr uint16_t kLogOriginRefresh  = 100U;
static constexpr uint32_t kLogMaxSize        = 0;
// Sensors are catalog-scaled ints (PSI x100, mV); controls are 0/1 in UcmControl enum order.
static const char* const  kLogHeaders[kLogCols] = {
    "time", "pt202", "pt102", "volt24", "vsol",
    "av204", "avSpare", "av203", "av205", "pwrPtUcm", "pwrSolLcm", "pwrPtLcm"};

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
