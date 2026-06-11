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

static bool s_boardHardwareReady = false;
bool g_valveStates[4]     = {false, false, false, false};
bool g_24VoltageFet[3] = {false, false, false};
float g_ptValues[4]       = {0.0f, 0.0f, 0.0f, 0.0f};
float g_hallEffect[3]     = {0.0f, 0.0f, 0.0f};
float g_24VoltageSense[2] = {0.0f, 0.0f};
float g_thermocouple      = 0.0f;

static ADS131M04 s_adc(-1, ADC_DRDY_PIN, &SPI);

constexpr uint8_t kAdcClockChannel = 0U;
constexpr uint32_t kAdcClockHz = 8192000U;
constexpr uint8_t kAdcClockDuty = 1U;
constexpr uint32_t kTelemetryPeriodMs = 100U;
constexpr size_t kAdcChannelCount = 4U;
constexpr uint8_t kValveIndexBase  = 2U;
constexpr uint8_t kValveLatchCount = 2U;

// Endpoint IDs sent with remote valve commands (packed into CAN timed data).
constexpr uint8_t kValveLatchEndpoints[kValveLatchCount] = {
    BOARD_ENDPOINT_LOWER_BASE + BOARD_ENDPOINT_PT1,
    BOARD_ENDPOINT_LOWER_BASE + BOARD_ENDPOINT_PT2,
};

struct ValveLatch {
    uint32_t payload;
    bool     dirty;
};
static ValveLatch s_valveLatch[kValveLatchCount] = {};

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

static void s_qlcpNetService(uint32_t nowMs);
static void s_qlcpTelemetryService(uint32_t nowMs);

static bool s_setValveByIndex(uint8_t index, bool open) {
  if (index < 2U) {
    digitalWrite(static_cast<int>(kLocalValveControls[index].pin), open ? HIGH : LOW);
    g_valveStates[index] = open;
    return true;
  }
  if ((index >= kValveIndexBase) && (index < (kValveIndexBase + kValveLatchCount))) {
#ifdef MOCK_HARDWARE
    g_valveStates[index] = open;
    return true;
#else
    const uint32_t payload = open ? BOARD_ACTUATOR_OPEN : BOARD_ACTUATOR_CLOSED;
    const uint8_t latchIdx = index - kValveIndexBase;
    if (s_valveLatch[latchIdx].payload != payload) {
      s_valveLatch[latchIdx].payload = payload;
      s_valveLatch[latchIdx].dirty   = true;
    }
    return true;
#endif
  }
  return false;
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
  AIM_ASSERT(!s_boardHardwareReady);

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
  s_adc.init();
#endif

  digitalWrite(VPT_EN_PIN, HIGH);
  s_boardHardwareReady = true;
  LOG_INFO("Board hardware initialized");

  AIM_ASSERT(s_boardHardwareReady);
  return true;
}

void boardServiceTx(uint32_t schedulerNowMs, uint32_t networkNowMs, AimNetwork& aim, uint32_t boardState) {
  AIM_ASSERT(s_boardHardwareReady);

  // PT1 and PT2: sample ADC + send at 100 ms; lastSentMs updated unconditionally
  // so a failed send drops that sample and the next fires a full period later.
  static uint32_t s_lastPtMs = 0U;
  if ((schedulerNowMs - s_lastPtMs) >= kTelemetryPeriodMs) {
    s_lastPtMs = schedulerNowMs;

#ifdef MOCK_HARDWARE
    g_ptValues[0] = 50.0f + 10.0f * sin(schedulerNowMs / 1000.0f);
    g_ptValues[1] = 50.0f + 10.0f * cos(schedulerNowMs / 1000.0f);
    g_ptValues[2] = 40.0f + 5.0f * sin(schedulerNowMs / 2000.0f);
    g_ptValues[3] = 40.0f + 5.0f * cos(schedulerNowMs / 2000.0f);
#else
    int32_t rawData[kAdcChannelCount] = {0};
    if (s_adc.readChannels(rawData)) {
      float volts[kAdcChannelCount] = {0.0f};
      s_adc.computeVoltages(rawData, volts);
      for (size_t i = 0; i < (sizeof(kLocalPtSensors) / sizeof(kLocalPtSensors[0])); i++) {
        const uint8_t channel = kLocalPtSensors[i].adcChannel;
        g_ptValues[i] = s_processPressurePsi(volts[channel]);
      }
    } else {
      LOG_WARN("ADC sample timeout");
    }
#endif

    for (size_t i = 0U; i < (sizeof(kLocalPtSensors) / sizeof(kLocalPtSensors[0])); i++) {
      uint32_t ptPayload = 0U;
      static_assert(sizeof(ptPayload) == sizeof(g_ptValues[0]), "float payload packing assumes 32-bit float");
      std::memcpy(&ptPayload, &g_ptValues[i], sizeof(ptPayload));
      aim::Pkt pkt = {};
      pkt.dest = aim::Node::Comms;
      pkt.type = aim::PacketType::Sensor;
      if (pkt.packData(0U, networkNowMs, ptPayload)) {
        (void)aim.sendPkt(pkt);
      }
    }
  }

  // Heartbeat at 5 s; lastSentMs updated unconditionally (drop on fail).
  static uint32_t s_lastHbMs = 0U;
  if ((schedulerNowMs - s_lastHbMs) >= aim::kHeartbeatTxIntervalDefaultMs) {
    s_lastHbMs = schedulerNowMs;
    aim::Pkt pkt = {};
    pkt.dest = aim::Node::Comms;
    pkt.type = aim::PacketType::Heartbeat;
    if (pkt.packData(0U, networkNowMs, boardState)) {
      (void)aim.sendPkt(pkt);
    }
  }

  // Remote valve latches: dirty-driven, retried every tick until sendPkt succeeds.
  for (uint8_t i = 0U; i < kValveLatchCount; i++) {
    if (s_valveLatch[i].dirty) {
      aim::Pkt pkt = {};
      pkt.dest = aim::Node::LProp;
      pkt.type = aim::PacketType::Valve;
      if (pkt.packData(kValveLatchEndpoints[i], networkNowMs, s_valveLatch[i].payload)) {
        if (aim.sendPkt(pkt)) {
          s_valveLatch[i].dirty = false;
        }
        // sendPkt failure: dirty stays set, retried next tick
      }
    }
  }
}

bool boardHandleCanPacket(const aim::Pkt& pkt, uint32_t networkNowMs, AimNetwork& aim) {
  if (pkt.dest != BOARD_ORIGIN && pkt.dest != aim::Node::Broadcast) {
    return false;
  }

  if (pkt.type == aim::PacketType::Valve) {
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

  if (pkt.type == aim::PacketType::Sensor) {
    const uint8_t endpointId = pkt.getEndpointId();
    if (endpointId >= BOARD_ENDPOINT_LOWER_BASE) {
      float val = 0.0f;
      const uint32_t payload = pkt.getPayload();
      std::memcpy(&val, &payload, sizeof(val));
      // TODO: add non PT support
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
  AIM_ASSERT(s_boardHardwareReady);
  // Network/QLCP services run every tick — including in DEBUG_CONSOLE — so
  // server heartbeat ACKs continue while an operator is in the console.
  s_qlcpNetService(schedulerNowMs);
  s_qlcpTelemetryService(schedulerNowMs);
}

static void s_qlcpHandlePacket(const qlcp_client_payload& in) {
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
      const bool open = (in.payload_data.control.command_state == QLCP_CS_OPEN);
      LOG_INFO("Received control cmdId=%u state=%u", cmdId, in.payload_data.control.command_state);
      if (s_setValveByIndex(cmdId, open)) {
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
static void s_qlcpNetService(uint32_t nowMs) {
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
        s_qlcpHandlePacket(in);
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

static void s_qlcpTelemetryService(uint32_t nowMs) {
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

void boardStartNetwork(void) {
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

bool boardSetValveStateDirect(uint8_t index, bool open) {
  if (index >= 4U) { return false; }
  LOG_DEBUG("Direct console actuator override index=%u open=%d", index, open);
  return s_setValveByIndex(index, open);
}

#endif
