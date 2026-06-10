#include "board.h"

#include <ADS131M04.h>
#include <SPI.h>
#include <logger.h>

#include <cstring>
#include <WiFi.h>

extern "C" {
#include "wifi_tools.h"
#include <qlcp_lib.h>
}

bool g_boardHardwareReady = false;
uint32_t g_lastTelemetryMs = 0U;
bool g_valveStates[4] = {false, false, false, false};
float g_ptValues[4] = {0.0f, 0.0f, 0.0f, 0.0f};

ADS131M04 g_adc(-1, ADC_DRDY_PIN, &SPI);
static AimNetwork* s_aimPtr = nullptr;

constexpr uint8_t kAdcClockChannel = 0U;
constexpr uint32_t kAdcClockHz = 8192000U;
constexpr uint8_t kAdcClockDuty = 1U;
constexpr uint32_t kTelemetryPeriodMs = 100U;
constexpr size_t kAdcChannelCount = 4U;

constexpr float kPtShuntResistanceOhms = 62.0f;
constexpr float kPtMaxPsi = 100.0f;

struct BoardSensorMapping {
  uint8_t endpointId;
  uint8_t adcChannel;
};

struct BoardControlMapping {
  uint8_t endpointId;
  uint8_t pin;
  bool defaultOpen;
};

constexpr BoardSensorMapping kLocalPtSensors[] = {
    {BOARD_ENDPOINT_PT1, 0U},
    {BOARD_ENDPOINT_PT2, 1U},
};

constexpr BoardControlMapping kLocalValveControls[] = {
    {BOARD_ENDPOINT_VALVE1, SOL1_EN_PIN, false},
    {BOARD_ENDPOINT_VALVE2, SOL2_EN_PIN, false},
};

constexpr char kBoardQlcpConfigJson[] = R"json({
  "device_name": "PEGASUS-UPPER",
  "device_type": "Sensor Monitor",
  "sensor_info": {
    "voltage_sense": {
      "LOWER_VSOL_SENSE": {
        "sensor_index": "LOWER_SOL_VOLTAGE_SENSE",
        "unit": "V"
      },
      "UPPER_24V_SENSE": {
        "sensor_index": "UPPER_24_VOLTAGE_SENSE",
        "unit": "V"
      }
    },
    "hall_effect": {
      "UPPER_HALL1": {
        "sensor_index": "HALL1",
        "unit": "A"       
      },
      "LOWER_HALL1": {
        "sensor_index": "HALL2",
        "unit": "A"
      },
      "LOWER_HALL2": {
        "sensor_index": "HALL3",
        "unit": "A"
      }
    },
    "thermocouple": {
      "LOWER_TC1": {
        "sensor_index": "TC1",
        "type": "K",
        "unit": "C"
      }
    },
    "pressure_transducer": {
      "UPPER_PT1": {
        "sensor_index": "PT1",
        "resistor_ohms": 62,
        "max_pressure_PSI": 100,
        "unit": "PSI"
      },
      "UPPER_PT2": {
        "sensor_index": "PT2",
        "resistor_ohms": 62,
        "max_pressure_PSI": 100,
        "unit": "PSI"
      },
      "LOWER_PT1": {
        "sensor_index": "PT3",
        "resistor_ohms": 62,
        "max_pressure_PSI": 100,
        "unit": "PSI"
      },
      "LOWER_PT2": {
        "sensor_index": "PT4",
        "resistor_ohms": 62,
        "max_pressure_PSI": 100,
        "unit": "PSI"
      }
    }
  },
  "controls": {
    "UPPER_V_PT": {
      "control_index": "UPPER_V_PT_CTL",
      "default_state": "OPEN",
      "type": "relay"
    },
    "LOWER_V_PT": {
      "control_index": "LOWER_V_PT_CTL",
      "default_state": "OPEN",
      "type": "relay"
    },
    "LOWER_V_SOL": {
      "control_index": "LOWER_V_SOL_CTL",
      "default_state": "OPEN",
      "type": "relay"
    },
    "UPPER_VALVE1": {
      "control_index": "SOL1",
      "default_state": "CLOSED",
      "type": "solenoid"
    },
    "UPPER_VALVE2": {
      "control_index": "SOL2",
      "default_state": "CLOSED",
      "type": "solenoid"
    },
    "LOWER_VALVE1": {
      "control_index": "SOL3",
      "default_state": "CLOSED",
      "type": "solenoid"
    },
    "LOWER_VALVE2": {
      "control_index": "SOL4",
      "default_state": "CLOSED",
      "type": "solenoid"
    }
  }
})json";

// ── QLCP network link (single-threaded: all state below is owned by the
// main loop — no atomics, queues, or event groups required) ────────────────

enum QlcpNetState : uint8_t {
  QLCP_NET_IDLE = 0U,
  QLCP_NET_WIFI_START,
  QLCP_NET_WIFI_WAIT,
  QLCP_NET_DISCOVER,
  QLCP_NET_TCP_CONNECT,
  QLCP_NET_CONNECTED,
  QLCP_NET_BACKOFF
};

constexpr uint32_t kNetWifiWaitTimeoutMs  = 15000U;
constexpr uint32_t kNetTcpConnectTimeoutMs = 5000U;
// Inherited from the old SO_RCVTIMEO: the server must heartbeat faster than
// this or the link is torn down and rediscovered.
constexpr uint32_t kNetRxIdleTimeoutMs    = 15000U;
constexpr uint32_t kNetBackoffMinMs       = 1000U;
constexpr uint32_t kNetBackoffMaxMs       = 8000U;

static net_link_t   s_netLink = {};
static QlcpNetState s_netState = QLCP_NET_IDLE;
static uint32_t     s_stateEnteredMs = 0U;
static uint32_t     s_lastRxMs = 0U;
static uint32_t     s_backoffMs = kNetBackoffMinMs;
static bool         s_configSent = false;
static uint16_t     s_sequence = 0U;
static uint32_t     s_tsOffset = 0U;
static uint16_t     s_streamFrequencyHz = 0U;  // 0 = stream off
static uint32_t     s_lastStreamTxMs = 0U;

// Counts reported to the QLCP server: PT1-4 sensors, SOL1-4 controls.
constexpr uint8_t kQlcpControlCount = 4U;
constexpr uint8_t kQlcpSensorCount  = 4U;

static void s_fillHeader(qlcp_header& header) {
  header.sequence = static_cast<uint8_t>(s_sequence++);
  header.timestamp = s_tsOffset + millis();
}

static const char* s_netStateName(QlcpNetState st) {
  switch (st) {
    case QLCP_NET_IDLE:        return "IDLE";
    case QLCP_NET_WIFI_START:  return "WIFI_START";
    case QLCP_NET_WIFI_WAIT:   return "WIFI_WAIT";
    case QLCP_NET_DISCOVER:    return "DISCOVER";
    case QLCP_NET_TCP_CONNECT: return "TCP_CONNECT";
    case QLCP_NET_CONNECTED:   return "CONNECTED";
    case QLCP_NET_BACKOFF:     return "BACKOFF";
    default:                   return "?";
  }
}

static void s_netTransition(QlcpNetState next, uint32_t nowMs) {
  LOG_INFO("QLCP net: %s -> %s", s_netStateName(s_netState), s_netStateName(next));
  s_netState = next;
  s_stateEnteredMs = nowMs;
}

// Any link fault: close everything and retry after the (doubling) backoff.
static void s_netFail(uint32_t nowMs) {
  net_link_close_all(&s_netLink);
  s_streamFrequencyHz = 0U;
  s_configSent = false;
  s_netTransition(QLCP_NET_BACKOFF, nowMs);
}

static void qlcpNetService(uint32_t nowMs);
static void qlcpTelemetryService(uint32_t nowMs);

int s_localValveIndexFromEndpoint(uint8_t endpointId) {
  if (endpointId == BOARD_ENDPOINT_VALVE1) return 0;
  if (endpointId == BOARD_ENDPOINT_VALVE2) return 1;
  return -1;
}

#ifndef MOCK_HARDWARE
float s_processPressurePsi(float voltagePt) {
  AIM_ASSERT(voltagePt >= 0.0f && voltagePt <= 5.0f);

  const float currentMa = (voltagePt / kPtShuntResistanceOhms) * 1000.0f;
  const float psi = (currentMa - 4.0f) * (kPtMaxPsi / 16.0f);

  AIM_ASSERT(psi >= -50.0f && psi <= 150.0f);
  return psi;
}
#endif

bool s_setLocalValveState(uint8_t endpointId, bool open) {
  const int valveIndex = s_localValveIndexFromEndpoint(endpointId);
  if (valveIndex < 0) {
    return false;
  }
  digitalWrite(static_cast<int>(kLocalValveControls[valveIndex].pin), open ? HIGH : LOW);
  g_valveStates[valveIndex] = open;

  AIM_ASSERT(g_valveStates[valveIndex] == open);
  return true;
}

bool s_sendFloatTelemetry(AimNetwork& aim, uint8_t endpointId, float value, uint32_t networkNowMs) {
  AIM_ASSERT(endpointId > 0 && endpointId <= AIM_PKT_TIMED_ENDPOINT_MAX);

  aimPkt pkt = {};
  pkt.dest = AIM_DEST_COMMS;
  pkt.type = AIM_TYPE_SENSOR;
  uint32_t payload = 0U;
  static_assert(sizeof(payload) == sizeof(value), "float payload packing assumes 32-bit float");
  std::memcpy(&payload, &value, sizeof(payload));
  
  const bool ok = pkt.packData(endpointId, networkNowMs, payload) && aim.sendPkt(pkt);
  return ok;
}

static void s_sendAck(uint8_t ackType, uint16_t ackSeq) {
  qlcp_server_payload out = {};
  out.packet_type = QLCP_PT_ACK;
  s_fillHeader(out.payload_data.ack.header);
  out.payload_data.ack.ack_packet_type = ackType;
  out.payload_data.ack.ack_sequence = ackSeq;
  if (tcp_tx_payload(&s_netLink, &out) != 0) {
    LOG_DEBUG("ACK dropped — TX busy");  // server retries on its own cadence
  }
}

static void s_sendNack(uint8_t nackType, uint16_t nackSeq, uint8_t errCode) {
  qlcp_server_payload out = {};
  out.packet_type = QLCP_PT_NACK;
  s_fillHeader(out.payload_data.nack.header);
  out.payload_data.nack.nack_packet_type = nackType;
  out.payload_data.nack.nack_sequence = nackSeq;
  out.payload_data.nack.nack_error_code = errCode;
  if (tcp_tx_payload(&s_netLink, &out) != 0) {
    LOG_DEBUG("NACK dropped — TX busy");
  }
}

bool boardInitHardware(void) {
  AIM_ASSERT(!g_boardHardwareReady);

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

#ifndef MOCK_HARDWARE
  SPI.begin(ADC_SCLK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN, -1);
  g_adc.init();
#endif

  digitalWrite(VPT_EN_PIN, HIGH);
  g_boardHardwareReady = true;
  g_lastTelemetryMs = millis();
  LOG_INFO("Board hardware initialized");

  AIM_ASSERT(g_boardHardwareReady);
  return true;
}

void boardServiceLocalTelemetry(uint32_t schedulerNowMs, uint32_t networkNowMs, AimNetwork& aim) {
  AIM_ASSERT(g_boardHardwareReady);

  if ((schedulerNowMs - g_lastTelemetryMs) < kTelemetryPeriodMs) {
    return;
  }
  g_lastTelemetryMs = schedulerNowMs;

  for (size_t i = 0; i < (sizeof(kLocalPtSensors) / sizeof(kLocalPtSensors[0])); i++) {
    const float psi = g_ptValues[i];
    if (!s_sendFloatTelemetry(aim, kLocalPtSensors[i].endpointId, psi, networkNowMs)) {
      LOG_ERROR("PT telemetry send failed for endpoint=%u", static_cast<unsigned int>(kLocalPtSensors[i].endpointId));
    }
  }

  AIM_ASSERT(g_lastTelemetryMs == schedulerNowMs);
}

bool boardHandleCanPacket(const aimPkt& pkt, uint32_t networkNowMs, AimNetwork& aim) {
  if (pkt.dest != BOARD_ORIGIN && pkt.dest != AIM_DEST_BROADCAST) {
    return false;
  }

  if (pkt.type == AIM_TYPE_VALVE) {
    const uint8_t endpointId = pkt.getEndpointId();
    if (endpointId >= BOARD_ENDPOINT_LOWER_BASE) {
      const bool openCommand = (pkt.getPayload() != BOARD_ACTUATOR_CLOSED);
      // Lower valve state echo received back from Lower Control!
      int valveIdx = endpointId - (BOARD_ENDPOINT_LOWER_BASE + 2U);
      if (valveIdx >= 0 && valveIdx < 2) {
        g_valveStates[2 + valveIdx] = openCommand;
        LOG_INFO("Lower valve %d state echo: %d", valveIdx, openCommand);
      }
      return true;
    }
  }

  if (pkt.type == AIM_TYPE_SENSOR) {
    const uint8_t endpointId = pkt.getEndpointId();
    if (endpointId >= BOARD_ENDPOINT_LOWER_BASE) {
      float val = 0.0f;
      const uint32_t payload = pkt.getPayload();
      std::memcpy(&val, &payload, sizeof(val));

      int ptIdx = endpointId - BOARD_ENDPOINT_LOWER_BASE;
      if (ptIdx >= 0 && ptIdx < 2) {
        g_ptValues[2 + ptIdx] = val;
        LOG_DEBUG("Lower PT %d telemetry received: %.2f PSI", ptIdx, val);
      }
    }
    return true;
  }

  return false;
}

void boardUpdate(uint32_t schedulerNowMs) {
  AIM_ASSERT(g_boardHardwareReady);

  static uint32_t lastAdcReadMs = 0U;
  if ((schedulerNowMs - lastAdcReadMs) >= kTelemetryPeriodMs) {
    lastAdcReadMs = schedulerNowMs;

#ifdef MOCK_HARDWARE
    g_ptValues[0] = 50.0f + 10.0f * sin(schedulerNowMs / 1000.0f);
    g_ptValues[1] = 50.0f + 10.0f * cos(schedulerNowMs / 1000.0f);
    g_ptValues[2] = 40.0f + 5.0f * sin(schedulerNowMs / 2000.0f);
    g_ptValues[3] = 40.0f + 5.0f * cos(schedulerNowMs / 2000.0f);
#else
    int32_t rawData[kAdcChannelCount] = {0};
    if (g_adc.readChannels(rawData)) {
      float volts[kAdcChannelCount] = {0.0f};
      g_adc.computeVoltages(rawData, volts);

      for (size_t i = 0; i < (sizeof(kLocalPtSensors) / sizeof(kLocalPtSensors[0])); i++) {
        const uint8_t channel = kLocalPtSensors[i].adcChannel;
        g_ptValues[i] = s_processPressurePsi(volts[channel]);
      }
    } else {
      LOG_WARN("ADC sample timeout");
    }
#endif
  }

  // Network/QLCP services run every tick — including in DEBUG_CONSOLE — so
  // server heartbeat ACKs continue while an operator is in the console.
  qlcpNetService(schedulerNowMs);
  qlcpTelemetryService(schedulerNowMs);
}

static void qlcpHandlePacket(const qlcp_client_payload& in) {
  switch (in.packet_type) {
    case QLCP_PT_TIMESYNC: {
      const uint32_t serverTime = in.payload_data.header_only.timestamp;
      s_tsOffset = serverTime - millis();
      LOG_INFO("QLCP timesync completed. Offset: %u ms", s_tsOffset);
      s_sendAck(QLCP_PT_TIMESYNC, in.payload_data.header_only.sequence);
      break;
    }

    case QLCP_PT_HEARTBEAT: {
      s_sendAck(QLCP_PT_HEARTBEAT, in.payload_data.header_only.sequence);
      break;
    }

    case QLCP_PT_STREAM_START: {
      const uint16_t freq = in.payload_data.stream_start.stream_frequency;
      if (freq > 0U) {
        s_streamFrequencyHz = freq;
        s_lastStreamTxMs = millis();
        LOG_INFO("QLCP Stream Start at %u Hz", freq);
      }
      s_sendAck(QLCP_PT_STREAM_START, in.payload_data.header_only.sequence);
      break;
    }

    case QLCP_PT_STREAM_STOP: {
      s_streamFrequencyHz = 0U;
      LOG_INFO("QLCP Stream Stop");
      s_sendAck(QLCP_PT_STREAM_STOP, in.payload_data.header_only.sequence);
      break;
    }

    case QLCP_PT_CONTROL: {
      const uint8_t cmdId = in.payload_data.control.command_id;
      const uint8_t cmdState = in.payload_data.control.command_state;
      const bool open = (cmdState == QLCP_CS_OPEN);
      bool ok = false;

      LOG_INFO("Received control cmdId=%u state=%u", cmdId, cmdState);

      if (cmdId < 2U) {
        const uint8_t endpoint = (cmdId == 0U) ? BOARD_ENDPOINT_VALVE1 : BOARD_ENDPOINT_VALVE2;
        ok = s_setLocalValveState(endpoint, open);
      } else if (cmdId < 4U) {
#ifdef MOCK_HARDWARE
        g_valveStates[cmdId] = open;
        ok = true;
#else
        const uint8_t endpoint = (cmdId == 2U) ? BOARD_ENDPOINT_LOWER_BASE : (BOARD_ENDPOINT_LOWER_BASE + 1U);
        if (s_aimPtr != nullptr) {
          const uint32_t payload = open ? BOARD_ACTUATOR_OPEN : BOARD_ACTUATOR_CLOSED;
          ok = s_aimPtr->sendTimedPktEx(endpoint, s_aimPtr->syncedMillis(), payload, AIM_DEST_LPROP, AIM_TYPE_VALVE);
        }
#endif
      }

      if (ok) {
        s_sendAck(QLCP_PT_CONTROL, in.payload_data.header_only.sequence);
      } else {
        s_sendNack(QLCP_PT_CONTROL, in.payload_data.header_only.sequence, QLCP_ERR_HARDWARE_FAULT);
      }
      break;
    }

    case QLCP_PT_STATUS_REQUEST: {
      // Encoded immediately by tcp_tx_payload, so the stack array is safe.
      qlcp_control_data controlData[kQlcpControlCount] = {};
      for (uint8_t i = 0U; i < kQlcpControlCount; i++) {
        controlData[i].control_id = i;
        controlData[i].control_state = g_valveStates[i] ? QLCP_CS_OPEN : QLCP_CS_CLOSED;
      }

      qlcp_server_payload out = {};
      out.packet_type = QLCP_PT_STATUS;
      s_fillHeader(out.payload_data.status.header);
      out.payload_data.status.control_data = controlData;
      out.payload_data.status.control_count = kQlcpControlCount;
      out.payload_data.status.device_status = QLCP_DS_ACTIVE;

      if (tcp_tx_payload(&s_netLink, &out) != 0) {
        LOG_DEBUG("STATUS dropped — TX busy");
      }
      break;
    }

    default:
      break;
  }
}

// Worst-case tick: one zero-timeout select/getsockopt, <=512 B RX copy plus
// one decode, <=1 KB TX send copy — single-digit ms against the 2 s task WDT.
static void qlcpNetService(uint32_t nowMs) {
  switch (s_netState) {
    case QLCP_NET_IDLE:
      break;

    case QLCP_NET_WIFI_START:
      LOG_INFO("WiFi Connecting to SSID: %s", WIFI_SSID);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      s_netTransition(QLCP_NET_WIFI_WAIT, nowMs);
      break;

    case QLCP_NET_WIFI_WAIT:
      if (WiFi.status() == WL_CONNECTED) {
        LOG_INFO("WiFi Connected! IP: %s", WiFi.localIP().toString().c_str());
        s_netLink.netif_handle = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (ssdp_listen_begin(&s_netLink) == ESP_OK) {
          s_netTransition(QLCP_NET_DISCOVER, nowMs);
        } else {
          s_netFail(nowMs);
        }
      } else if ((nowMs - s_stateEnteredMs) >= kNetWifiWaitTimeoutMs) {
        LOG_WARN("WiFi connect timeout — retrying");
        WiFi.disconnect();
        s_netTransition(QLCP_NET_WIFI_START, nowMs);
      }
      break;

    case QLCP_NET_DISCOVER: {
      if (WiFi.status() != WL_CONNECTED) {
        s_netFail(nowMs);
        break;
      }
      const int found = ssdp_listen_service(&s_netLink);
      if (found == 1) {
        ssdp_listen_end(&s_netLink);
        if (tcp_connect_begin(&s_netLink) == ESP_OK) {
          s_netTransition(QLCP_NET_TCP_CONNECT, nowMs);
        } else {
          s_netFail(nowMs);
        }
      } else if (found < 0) {
        s_netFail(nowMs);
      }
      break;
    }

    case QLCP_NET_TCP_CONNECT: {
      const int conn = tcp_connect_service(&s_netLink);
      if (conn == 1) {
        if (udp_create_socket(&s_netLink) != ESP_OK) {
          s_netFail(nowMs);
          break;
        }
        s_configSent = false;
        s_lastRxMs = nowMs;
        s_backoffMs = kNetBackoffMinMs;
        s_netTransition(QLCP_NET_CONNECTED, nowMs);
      } else if ((conn < 0) || ((nowMs - s_stateEnteredMs) >= kNetTcpConnectTimeoutMs)) {
        s_netFail(nowMs);
      }
      break;
    }

    case QLCP_NET_CONNECTED: {
      if (WiFi.status() != WL_CONNECTED) {
        s_netFail(nowMs);
        break;
      }

      if (!s_configSent) {
        qlcp_server_payload out = {};
        out.packet_type = QLCP_PT_CONFIG;
        s_fillHeader(out.payload_data.config.header);
        out.payload_data.config.config_data = kBoardQlcpConfigJson;
        out.payload_data.config.config_data_len = sizeof(kBoardQlcpConfigJson) - 1U;
        if (tcp_tx_payload(&s_netLink, &out) == 0) {
          s_configSent = true;
          LOG_INFO("Sent CONFIG packet to server");
        }
      }

      if (tcp_tx_service(&s_netLink) < 0) {
        s_netFail(nowMs);
        break;
      }

      qlcp_client_payload in = {};
      const int rx = tcp_rx_service(&s_netLink, &in);
      if (rx < 0) {
        s_netFail(nowMs);
        break;
      }
      if (rx == 1) {
        s_lastRxMs = nowMs;
        qlcpHandlePacket(in);
      }

      if ((nowMs - s_lastRxMs) >= kNetRxIdleTimeoutMs) {
        LOG_WARN("QLCP server silent — reconnecting");
        s_netFail(nowMs);
      }
      break;
    }

    case QLCP_NET_BACKOFF:
      if ((nowMs - s_stateEnteredMs) >= s_backoffMs) {
        s_backoffMs = (s_backoffMs >= (kNetBackoffMaxMs / 2U)) ? kNetBackoffMaxMs : (s_backoffMs * 2U);
        if (WiFi.status() != WL_CONNECTED) {
          s_netTransition(QLCP_NET_WIFI_START, nowMs);
        } else if (ssdp_listen_begin(&s_netLink) == ESP_OK) {
          s_netTransition(QLCP_NET_DISCOVER, nowMs);
        } else {
          s_netFail(nowMs);  // back to BACKOFF with a longer delay
        }
      }
      break;

    default:
      AIM_ASSERT(false);  // unreachable — all states handled above
      break;
  }
}

static void qlcpTelemetryService(uint32_t nowMs) {
  if ((s_netState != QLCP_NET_CONNECTED) || (s_streamFrequencyHz == 0U)) {
    return;
  }
  const uint32_t periodMs = 1000U / s_streamFrequencyHz;
  if ((nowMs - s_lastStreamTxMs) < periodMs) {
    return;
  }
  s_lastStreamTxMs = nowMs;

  qlcp_sensor_data readings[kQlcpSensorCount] = {};
  for (uint8_t i = 0U; i < kQlcpSensorCount; i++) {
    readings[i].sensor_id = i;
    readings[i].unit = QLCP_UNIT_PSI;
    readings[i].value = g_ptValues[i];
  }

  qlcp_data_packet pkt = {};
  s_fillHeader(pkt.header);
  pkt.sensor_data = readings;
  pkt.sensor_count = kQlcpSensorCount;

  (void)udp_send_data(&s_netLink, &pkt);  // lossy by design
}

void boardStartNetwork(AimNetwork& aim) {
  s_aimPtr = &aim;
  net_link_init(&s_netLink);
  s_netTransition(QLCP_NET_WIFI_START, millis());
}

#ifndef FLIGHT_BUILD
void boardPrintNetworkStatus(Print& out) {
  out.printf("net=%s ip=%s rssi=%d qlcp=%s svr=%s:%u/%u stream=%uHz\n",
    WIFI_SSID, WiFi.localIP().toString().c_str(), WiFi.RSSI(),
    s_netStateName(s_netState),
    s_netLink.server_ip, s_netLink.server_tcp_port, s_netLink.server_udp_port,
    static_cast<unsigned>(s_streamFrequencyHz));
}

void boardPrintSensorStatus(Print& out) {
  out.printf("pt1=%.1f pt2=%.1f pt3=%.1f pt4=%.1f\n", 
    g_ptValues[0], g_ptValues[1], g_ptValues[2], g_ptValues[3]);
}

bool boardSetValveStateDirect(uint8_t index, bool open) {
  if (index >= 4) {
    return false;
  }

  LOG_DEBUG("Direct console actuator override index=%u open=%d", index, open);

  if (index < 2) {
    uint8_t endpoint = (index == 0) ? BOARD_ENDPOINT_VALVE1 : BOARD_ENDPOINT_VALVE2;
    return s_setLocalValveState(endpoint, open);
  } else {
#ifdef MOCK_HARDWARE
    g_valveStates[index] = open;
    return true;
#else
    uint8_t endpoint = (index == 2) ? BOARD_ENDPOINT_LOWER_BASE : (BOARD_ENDPOINT_LOWER_BASE + 1);
    if (s_aimPtr != nullptr) {
      const uint32_t payload = open ? BOARD_ACTUATOR_OPEN : BOARD_ACTUATOR_CLOSED;
      const bool ok = s_aimPtr->sendTimedPktEx(endpoint, s_aimPtr->syncedMillis(), payload, AIM_DEST_LPROP, AIM_TYPE_VALVE);
      return ok;
    }
    return false;
#endif
  }
}

bool boardGetValveState(uint8_t index) {
  if (index < 4) {
    return g_valveStates[index];
  }
  return false;
}
#endif
