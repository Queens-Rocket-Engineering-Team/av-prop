#include "node.h"

#include <ADS131M04.h>
#include <SPI.h>
#include <logger.h>

#include <cstring>

namespace {

constexpr uint8_t kAdcClockChannel = 0U;
constexpr uint32_t kAdcClockHz = 8192000U;
constexpr uint8_t kAdcClockDuty = 1U;
constexpr uint32_t kTelemetryPeriodMs = 100U;
constexpr size_t kAdcChannelCount = 4U;

constexpr float kPtShuntResistanceOhms = 62.0f;
constexpr float kPtMaxPsi = 100.0f;

struct NodeSensorMapping {
  uint8_t endpointId;
  uint8_t adcChannel;
};

struct NodeControlMapping {
  uint8_t endpointId;
  uint8_t pin;
  bool defaultOpen;
};

constexpr NodeSensorMapping kLocalPtSensors[] = {
    {NODE_ENDPOINT_PT1, 0U},
    {NODE_ENDPOINT_PT2, 1U},
};

constexpr NodeControlMapping kLocalValveControls[] = {
    {NODE_ENDPOINT_VALVE1, SOL1_EN_PIN, false},
    {NODE_ENDPOINT_VALVE2, SOL2_EN_PIN, false},
};

constexpr char kNodeConfigJson[] = R"json({
  "device_name": "PEGASUS-UPPER",
  "device_type": "Gateway",
  "sensor_info": {
    "pressure_transducer": {
      "PT1": {
        "sensor_index": "UPPER_PT1",
        "unit": "PSI",
        "endpoint_id": 1
      },
      "PT2": {
        "sensor_index": "UPPER_PT2",
        "unit": "PSI",
        "endpoint_id": 2
      }
    }
  },
  "controls": {
    "Valve1": {
      "control_index": "UPPER_SOL1",
      "default_state": "CLOSED",
      "endpoint_id": 4
    },
    "Valve2": {
      "control_index": "UPPER_SOL2",
      "default_state": "CLOSED",
      "endpoint_id": 5
    }
  },
  "routing": {
    "local_endpoint_max": 15,
    "lower_endpoint_base": 16,
    "lower_destination": "AIM_DEST_LPROP"
  }
})json";

ADS131M04 g_adc(-1, ADC_DRDY_PIN, &SPI);

bool g_nodeHardwareReady = false;
uint32_t g_lastTelemetryMs = 0U;
bool g_valveStates[2] = {false, false};

bool s_isLocalEndpoint(uint8_t endpointId) {
  return endpointId <= NODE_ENDPOINT_LOCAL_MAX;
}

bool s_isLowerEndpoint(uint8_t endpointId) {
  return endpointId >= NODE_ENDPOINT_LOWER_BASE && endpointId <= AIM_PKT_TIMED_ENDPOINT_MAX;
}

int s_localValveIndexFromEndpoint(uint8_t endpointId) {
  for (size_t i = 0; i < (sizeof(kLocalValveControls) / sizeof(kLocalValveControls[0])); i++) {
    if (kLocalValveControls[i].endpointId == endpointId) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

float s_processPressurePsi(float voltagePt) {
  const float currentMa = (voltagePt / kPtShuntResistanceOhms) * 1000.0f;
  return (currentMa - 4.0f) * (kPtMaxPsi / 16.0f);
}

bool s_setLocalValveState(uint8_t endpointId, bool open) {
  const int valveIndex = s_localValveIndexFromEndpoint(endpointId);
  if (valveIndex < 0) {
    return false;
  }
  digitalWrite(static_cast<int>(kLocalValveControls[valveIndex].pin), open ? HIGH : LOW);
  g_valveStates[valveIndex] = open;
  return true;
}

bool s_sendFloatTelemetry(AimNetwork& aim, uint8_t endpointId, float value, uint32_t networkNowMs) {
  uint32_t payload = 0U;
  static_assert(sizeof(payload) == sizeof(value), "float payload packing assumes 32-bit float");
  std::memcpy(&payload, &value, sizeof(payload));
  return aim.sendTimedPktEx(endpointId, networkNowMs, payload, NODE_PRIMARY_DEST, AIM_TYP_SENSOR);
}

}  // namespace

const char* nodeConfigJson(void) {
  return kNodeConfigJson;
}

size_t nodeConfigJsonLen(void) {
  return sizeof(kNodeConfigJson) - 1U;
}

bool nodeInitHardware(void) {
  const int indicatorLeds[] = {WIFI_LED_PIN, CAN_LED_PIN, DEBUG_LED_PIN};
  for (size_t i = 0; i < (sizeof(indicatorLeds) / sizeof(indicatorLeds[0])); i++) {
    pinMode(indicatorLeds[i], OUTPUT);
    digitalWrite(indicatorLeds[i], LOW);
  }

  pinMode(VPT_EN_PIN, OUTPUT);
  digitalWrite(VPT_EN_PIN, LOW);

  for (size_t i = 0; i < (sizeof(kLocalValveControls) / sizeof(kLocalValveControls[0])); i++) {
    pinMode(static_cast<int>(kLocalValveControls[i].pin), OUTPUT);
    const bool defaultOpen = kLocalValveControls[i].defaultOpen;
    digitalWrite(static_cast<int>(kLocalValveControls[i].pin), defaultOpen ? HIGH : LOW);
    g_valveStates[i] = defaultOpen;
  }

  ledcSetup(kAdcClockChannel, kAdcClockHz, kAdcClockDuty);
  ledcAttachPin(ADC_CLKIN_PIN, kAdcClockChannel);
  ledcWrite(kAdcClockChannel, 1U);

  SPI.begin(ADC_SCLK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN, -1);
  g_adc.init();

  digitalWrite(VPT_EN_PIN, HIGH);
  g_nodeHardwareReady = true;
  g_lastTelemetryMs = millis();
  LOG_INFO("Node hardware initialized");
  return true;
}

void nodeServiceLocalTelemetry(uint32_t schedulerNowMs, uint32_t networkNowMs, AimNetwork& aim) {
  if (!g_nodeHardwareReady) {
    return;
  }
  if ((schedulerNowMs - g_lastTelemetryMs) < kTelemetryPeriodMs) {
    return;
  }
  g_lastTelemetryMs = schedulerNowMs;

  int32_t rawData[kAdcChannelCount] = {0};
  if (!g_adc.readChannels(rawData)) {
    LOG_WARN("ADC sample timeout");
    return;
  }

  float volts[kAdcChannelCount] = {0.0f};
  g_adc.computeVoltages(rawData, volts);

  for (size_t i = 0; i < (sizeof(kLocalPtSensors) / sizeof(kLocalPtSensors[0])); i++) {
    const uint8_t channel = kLocalPtSensors[i].adcChannel;
    const float psi = s_processPressurePsi(volts[channel]);
    if (!s_sendFloatTelemetry(aim, kLocalPtSensors[i].endpointId, psi, networkNowMs)) {
      LOG_ERROR("PT telemetry send failed for endpoint=%u", static_cast<unsigned int>(kLocalPtSensors[i].endpointId));
    }
  }
}

bool nodeHandleCanPacket(const aimPkt& pkt, uint32_t networkNowMs, AimNetwork& aim) {
  if (pkt.dest != NODE_ORIGIN && pkt.dest != AIM_DEST_BROADCAST) {
    return false;
  }

  if (pkt.type != AIM_TYP_VALVE) {
    return false;
  }

  const uint8_t endpointId = pkt.getEndpointId();

  const bool openCommand = (pkt.getPayload() != NODE_ACTUATOR_CLOSED);

  if (s_isLocalEndpoint(endpointId)) {
    if (!s_setLocalValveState(endpointId, openCommand)) {
      LOG_WARN("Valve command for unknown local endpoint=%u", static_cast<unsigned int>(endpointId));
      return true;
    }

    const uint32_t statePayload = openCommand ? NODE_ACTUATOR_OPEN : NODE_ACTUATOR_CLOSED;
    if (!aim.sendTimedPktEx(endpointId, networkNowMs, statePayload, pkt.origin, AIM_TYP_VALVE)) {
      LOG_ERROR("Valve state echo failed for endpoint=%u", static_cast<unsigned int>(endpointId));
    }
    return true;
  }

  if (s_isLowerEndpoint(endpointId)) {
    if (!aim.sendTimedPktEx(endpointId, networkNowMs, pkt.getPayload(), AIM_DEST_LPROP, AIM_TYP_VALVE)) {
      LOG_ERROR("Forward to Lower failed for endpoint=%u", static_cast<unsigned int>(endpointId));
    }
    return true;
  }

  return true;
}

void nodeUpdate(uint32_t schedulerNowMs) {
  // TODO: Upper control is not finished yet; add recurring node logic here.
  (void)schedulerNowMs;
}
