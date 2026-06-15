#include "node.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <ADS131M04.h>
#include <TMAG5273.h>
#include <SPI.h>
#include <Wire.h>
#include <logger.h>
#include <cstring>
#include <WiFi.h>

extern "C" {
#include "wifi_tools.h"
#include <qlcp_lib.h>
}

#define WIFI_SSID "TELUS1917"
#define WIFI_PASS "s24ec9424u"

static SemaphoreHandle_t s_nodeMutex = nullptr;

struct NodeLock {
  NodeLock() {
    if (s_nodeMutex != nullptr) {
      (void)xSemaphoreTake(s_nodeMutex, portMAX_DELAY);
    }
  }
  ~NodeLock() {
    if (s_nodeMutex != nullptr) {
      (void)xSemaphoreGive(s_nodeMutex);
    }
  }
};

static bool s_boardHardwareReady = false;

// Static module-scope state variables replacing the old globals
static bool  s_valveStates[4]     = {false, false, false, false}; // 0: Vent, 1: N2 Supply, 2: Fill Dump (remote), 3: Main (remote)
static bool  s_24VoltageFet[3]    = {false, false, false};        // 0: local VPT, 1: remote VPT, 2: remote VSOL
static float s_ptValues[4]        = {0.0f, 0.0f, 0.0f, 0.0f};     // 0-1: local PT1-2, 2-3: remote PT3-4
static float s_hallEffect[3]      = {0.0f, 0.0f, 0.0f};           // 0: local Hall 1, 1-2: remote Hall 2-3
static float s_24VoltageSense[2]  = {0.0f, 0.0f};                 // 0: local 24V sense, 1: remote VSOL sense
static float s_thermocouple       = 0.0f;                         // remote TC

// Remote valve command retry queues
struct ValveCommandState {
  uint8_t seq = 0U;
  uint8_t desiredState = 0U;
  uint32_t lastSentMs = 0U;
  bool dirty = false;
  bool awaitingAck = false;
};
static ValveCommandState s_valveCmds[4] = {}; // 0: ValveFillDump, 1: ValveMain, 2: PwrSolLcm, 3: PwrPtLcm

// Remote-command slots, indexed 0..3, mapped to their CAN subject.
constexpr uint8_t kRemoteValveSubjects[4] = {
    aim::subject::Av203,      // 0: Fill/Dump
    aim::subject::Av205,      // 1: Main
    aim::subject::PwrSolLcm,  // 2: LCM solenoid power
    aim::subject::PwrPtLcm,   // 3: LCM PT power
};

static uint8_t remoteValveIndex(uint8_t subject) {
  for (uint8_t i = 0U; i < 4U; i++) {
    if (kRemoteValveSubjects[i] == subject) return i;
  }
  return 0xFF;
}

static uint32_t s_lowerLastRxMs = 0U;
static bool     s_lowerLinkUp   = false;

static ADS131M04 s_adc(-1, pins::kAdcDrdy, &SPI);
static TMAG5273 s_hall(pins::kHallI2cAddr, &Wire);

constexpr uint8_t kAdcClockChannel = 0U;
constexpr uint32_t kAdcClockHz = 8192000U;
constexpr uint8_t kAdcClockDuty = 1U;
constexpr uint32_t kTelemetryPeriodMs = 100U;
constexpr uint32_t kLowerStaleTimeoutMs = 1000U;
constexpr uint32_t kValveEchoTimeoutMs = 500U;

constexpr size_t kAdcChannelCount = 4U;
constexpr uint8_t kValveIndexBase  = 2U;
constexpr uint8_t kValveLatchCount = 2U;

constexpr float kPtShuntResistanceOhms = 62.0f;
constexpr float kPtMaxPsi = 100.0f;

struct BoardSensorMapping {
  uint8_t adcChannel;
};

struct BoardControlMapping {
  uint8_t pin;
  bool defaultOpen;
};

constexpr BoardSensorMapping kLocalPtSensors[] = {
    {0U},
    {1U},
};

constexpr BoardControlMapping kLocalValveControls[] = {
    {pins::kSol1En, true},   // Vent: Normally Open (defaultOpen = true)
    {pins::kSol2En, false},  // N2: Normally Closed (defaultOpen = false)
};

constexpr char kBoardQlcpConfigJson[] = R"json({
  "device_name": "PEGASUS-UPPER",
  "device_type": "Sensor Monitor",
  "sensor_info": {
    "pressure_transducer": {
      "Pt202": {
        "sensor_index": "Pt202",
        "resistor_ohms": 62,
        "max_pressure_PSI": 100,
        "unit": "PSI"
      },
      "PtSpare1": {
        "sensor_index": "PtSpare1",
        "resistor_ohms": 62,
        "max_pressure_PSI": 100,
        "unit": "PSI"
      },
      "Pt204": {
        "sensor_index": "Pt204",
        "resistor_ohms": 62,
        "max_pressure_PSI": 100,
        "unit": "PSI"
      },
      "PtSpare2": {
        "sensor_index": "PtSpare2",
        "resistor_ohms": 62,
        "max_pressure_PSI": 100,
        "unit": "PSI"
      }
    },
    "hall_effect": {
      "Av204_Hall": {
        "sensor_index": "Av204_Hall",
        "unit": "V"
      },
      "Av203_Hall": {
        "sensor_index": "Av203_Hall",
        "unit": "V"
      },
      "Av205_Hall": {
        "sensor_index": "Av205_Hall",
        "unit": "V"
      }
    },
    "thermocouple": {
      "TcLowerValve": {
        "sensor_index": "TcLowerValve",
        "type": "K",
        "unit": "C"
      }
    },
    "voltage_sense": {
      "Volt24Ucm": {
        "sensor_index": "Volt24Ucm",
        "unit": "V"
      },
      "VoltSolLcm": {
        "sensor_index": "VoltSolLcm",
        "unit": "V"
      }
    }
  },
  "controls": {
    "Av204_Vent": {
      "control_index": "Av204",
      "default_state": "OPEN",
      "type": "solenoid"
    },
    "AvSpare": {
      "control_index": "AvSpare",
      "default_state": "CLOSED",
      "type": "solenoid"
    },
    "Av203_Fill": {
      "control_index": "Av203",
      "default_state": "OPEN",
      "type": "solenoid"
    },
    "Av205_Coax": {
      "control_index": "Av205",
      "default_state": "CLOSED",
      "type": "solenoid"
    },
    "PwrPtUcm": {
      "control_index": "PwrPtUcm",
      "default_state": "OPEN",
      "type": "relay"
    },
    "PwrSolLcm": {
      "control_index": "PwrSolLcm",
      "default_state": "CLOSED",
      "type": "relay"
    },
    "PwrPtLcm": {
      "control_index": "PwrPtLcm",
      "default_state": "OPEN",
      "type": "relay"
    }
  }
})json";

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
static uint16_t     s_streamFrequencyHz = 0U;
static uint32_t     s_lastStreamTxMs = 0U;

constexpr uint8_t kQlcpControlCount = 7U;
constexpr uint8_t kQlcpSensorCount  = 10U;

static void fillHeader(qlcp_header& header) {
  header.sequence = static_cast<uint8_t>(s_sequence++);
  header.timestamp = s_tsOffset + millis();
}

static void netTransition(QlcpNetState next, uint32_t nowMs) {
  LOG_INFO("QLCP net: %u -> %u", s_netState, next);
  s_netState = next;
  s_stateEnteredMs = nowMs;
}

static void netFail(uint32_t nowMs) {
  net_link_close_all(&s_netLink);
  s_streamFrequencyHz = 0U;
  s_configSent = false;
  netTransition(QLCP_NET_BACKOFF, nowMs);
}

static void qlcpNetService(uint32_t nowMs);
static void qlcpTelemetryService(uint32_t nowMs);

// Schedules a command to be sent to remote valves
static void nodeCommandRemoteValve(uint8_t subject, bool open) {
  const uint8_t idx = remoteValveIndex(subject);
  if (idx == 0xFF) return;

  uint8_t nextState = open ? 1U : 0U;
  if (s_valveCmds[idx].dirty || s_valveCmds[idx].desiredState != nextState || !s_valveCmds[idx].awaitingAck) {
    s_valveCmds[idx].desiredState = nextState;
    s_valveCmds[idx].seq++;
    s_valveCmds[idx].dirty = true;
    s_valveCmds[idx].awaitingAck = true;
  }
}

static bool setValveByIndex(uint8_t index, bool open) {
  NodeLock lock;
  if (index < 2U) {
    const bool defaultOpen = kLocalValveControls[index].defaultOpen;
    const bool pinState = open != defaultOpen; // Energize if we want physical state opposite to defaultOpen
    digitalWrite(static_cast<int>(kLocalValveControls[index].pin), pinState ? HIGH : LOW);
    s_valveStates[index] = open;
    return true;
  }
  if ((index >= kValveIndexBase) && (index < (kValveIndexBase + kValveLatchCount))) {
    nodeCommandRemoteValve((index == 2U) ? aim::subject::Av203 : aim::subject::Av205, open);
    return true;
  }
  if (index == 4U) {
    digitalWrite(pins::kVptEn, open ? HIGH : LOW);
    s_24VoltageFet[0] = open;
    return true;
  }
  if (index == 5U) {
    nodeCommandRemoteValve(aim::subject::PwrSolLcm, open);
    return true;
  }
  if (index == 6U) {
    nodeCommandRemoteValve(aim::subject::PwrPtLcm, open);
    return true;
  }
  return false;
}

#ifndef MOCK_HARDWARE
static float processPressurePsi(float voltagePt) {
  AIM_ASSERT(voltagePt >= 0.0f && voltagePt <= 5.0f);
  const float currentMa = (voltagePt / kPtShuntResistanceOhms) * 1000.0f;
  const float psi = (currentMa - 4.0f) * (kPtMaxPsi / 16.0f);
  AIM_ASSERT(psi >= -50.0f && psi <= 150.0f);
  return psi;
}
#endif

static void sendAck(uint8_t ackType, uint16_t ackSeq) {
  qlcp_server_payload out = {};
  out.packet_type = QLCP_PT_ACK;
  fillHeader(out.payload_data.ack.header);
  out.payload_data.ack.ack_packet_type = ackType;
  out.payload_data.ack.ack_sequence = ackSeq;
  if (tcp_tx_payload(&s_netLink, &out) != 0) {
    LOG_DEBUG("ACK dropped — TX busy");
  }
}

static void sendNack(uint8_t nackType, uint16_t nackSeq, uint8_t errCode) {
  qlcp_server_payload out = {};
  out.packet_type = QLCP_PT_NACK;
  fillHeader(out.payload_data.nack.header);
  out.payload_data.nack.nack_packet_type = nackType;
  out.payload_data.nack.nack_sequence = nackSeq;
  out.payload_data.nack.nack_error_code = errCode;
  if (tcp_tx_payload(&s_netLink, &out) != 0) {
    LOG_DEBUG("NACK dropped — TX busy");
  }
}

static void qlcpHandlePacket(const qlcp_client_payload& in) {
  switch (in.packet_type) {
    case QLCP_PT_TIMESYNC: {
      const uint32_t serverTime = in.payload_data.header_only.timestamp;
      s_tsOffset = serverTime - millis();
      LOG_INFO("QLCP timesync completed. Offset: %u ms", s_tsOffset);
      sendAck(QLCP_PT_TIMESYNC, in.payload_data.header_only.sequence);
      break;
    }
    case QLCP_PT_HEARTBEAT: {
      sendAck(QLCP_PT_HEARTBEAT, in.payload_data.header_only.sequence);
      break;
    }
    case QLCP_PT_STREAM_START: {
      const uint16_t freq = in.payload_data.stream_start.stream_frequency;
      if (freq > 0U) {
        s_streamFrequencyHz = freq;
        s_lastStreamTxMs = millis();
        LOG_INFO("QLCP Stream Start at %u Hz", freq);
      }
      sendAck(QLCP_PT_STREAM_START, in.payload_data.header_only.sequence);
      break;
    }
    case QLCP_PT_STREAM_STOP: {
      s_streamFrequencyHz = 0U;
      LOG_INFO("QLCP Stream Stop");
      sendAck(QLCP_PT_STREAM_STOP, in.payload_data.header_only.sequence);
      break;
    }
    case QLCP_PT_CONTROL: {
      const uint8_t cmdId = in.payload_data.control.command_id;
      const bool open = (in.payload_data.control.command_state == QLCP_CS_OPEN);
      LOG_INFO("Received control cmdId=%u state=%u", cmdId, in.payload_data.control.command_state);
      if (setValveByIndex(cmdId, open)) {
        sendAck(QLCP_PT_CONTROL, in.payload_data.header_only.sequence);
      } else {
        sendNack(QLCP_PT_CONTROL, in.payload_data.header_only.sequence, QLCP_ERR_HARDWARE_FAULT);
      }
      break;
    }
    case QLCP_PT_STATUS_REQUEST: {
      qlcp_control_data controlData[kQlcpControlCount] = {};
      {
        NodeLock lock;
        for (uint8_t i = 0U; i < kQlcpControlCount; i++) {
          controlData[i].control_id = i;
          if (i < 4U) {
            controlData[i].control_state = s_valveStates[i] ? QLCP_CS_OPEN : QLCP_CS_CLOSED;
          } else if (i == 4U) {
            controlData[i].control_state = s_24VoltageFet[0] ? QLCP_CS_OPEN : QLCP_CS_CLOSED;
          } else if (i == 5U) {
            controlData[i].control_state = s_24VoltageFet[2] ? QLCP_CS_OPEN : QLCP_CS_CLOSED;
          } else if (i == 6U) {
            controlData[i].control_state = s_24VoltageFet[1] ? QLCP_CS_OPEN : QLCP_CS_CLOSED;
          }
        }
      }
      qlcp_server_payload out = {};
      out.packet_type = QLCP_PT_STATUS;
      fillHeader(out.payload_data.status.header);
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

static void qlcpNetService(uint32_t nowMs) {
  switch (s_netState) {
    case QLCP_NET_IDLE:
      break;
    case QLCP_NET_WIFI_START:
      LOG_INFO("WiFi Connecting to SSID: %s", WIFI_SSID);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      netTransition(QLCP_NET_WIFI_WAIT, nowMs);
      break;
    case QLCP_NET_WIFI_WAIT:
      if (WiFi.status() == WL_CONNECTED) {
        LOG_INFO("WiFi Connected! IP: %s", WiFi.localIP().toString().c_str());
        s_netLink.netif_handle = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (ssdp_listen_begin(&s_netLink) == ESP_OK) {
          netTransition(QLCP_NET_DISCOVER, nowMs);
        } else {
          netFail(nowMs);
        }
      } else if ((nowMs - s_stateEnteredMs) >= kNetWifiWaitTimeoutMs) {
        LOG_WARN("WiFi connect timeout — retrying");
        WiFi.disconnect();
        netTransition(QLCP_NET_WIFI_START, nowMs);
      }
      break;
    case QLCP_NET_DISCOVER: {
      if (WiFi.status() != WL_CONNECTED) {
        netFail(nowMs);
        break;
      }
      const int found = ssdp_listen_service(&s_netLink);
      if (found == 1) {
        ssdp_listen_end(&s_netLink);
        if (tcp_connect_begin(&s_netLink) == ESP_OK) {
          netTransition(QLCP_NET_TCP_CONNECT, nowMs);
        } else {
          netFail(nowMs);
        }
      } else if (found < 0) {
        netFail(nowMs);
      }
      break;
    }
    case QLCP_NET_TCP_CONNECT: {
      const int conn = tcp_connect_service(&s_netLink);
      if (conn == 1) {
        if (udp_create_socket(&s_netLink) != ESP_OK) {
          netFail(nowMs);
          break;
        }
        s_configSent = false;
        s_lastRxMs = nowMs;
        s_backoffMs = kNetBackoffMinMs;
        netTransition(QLCP_NET_CONNECTED, nowMs);
      } else if ((conn < 0) || ((nowMs - s_stateEnteredMs) >= kNetTcpConnectTimeoutMs)) {
        netFail(nowMs);
      }
      break;
    }
    case QLCP_NET_CONNECTED: {
      if (WiFi.status() != WL_CONNECTED) {
        netFail(nowMs);
        break;
      }
      if (!s_configSent) {
        qlcp_server_payload out = {};
        out.packet_type = QLCP_PT_CONFIG;
        fillHeader(out.payload_data.config.header);
        out.payload_data.config.config_data = kBoardQlcpConfigJson;
        out.payload_data.config.config_data_len = sizeof(kBoardQlcpConfigJson) - 1U;
        if (tcp_tx_payload(&s_netLink, &out) == 0) {
          s_configSent = true;
          LOG_INFO("Sent CONFIG packet to server");
        }
      }
      if (tcp_tx_service(&s_netLink) < 0) {
        netFail(nowMs);
        break;
      }
      qlcp_client_payload in = {};
      const int rx = tcp_rx_service(&s_netLink, &in);
      if (rx < 0) {
        netFail(nowMs);
        break;
      }
      if (rx == 1) {
        s_lastRxMs = nowMs;
        qlcpHandlePacket(in);
      }
      if ((nowMs - s_lastRxMs) >= kNetRxIdleTimeoutMs) {
        LOG_WARN("QLCP server silent — reconnecting");
        netFail(nowMs);
      }
      break;
    }
    case QLCP_NET_BACKOFF:
      if ((nowMs - s_stateEnteredMs) >= s_backoffMs) {
        s_backoffMs = (s_backoffMs >= (kNetBackoffMaxMs / 2U)) ? kNetBackoffMaxMs : (s_backoffMs * 2U);
        if (WiFi.status() != WL_CONNECTED) {
          netTransition(QLCP_NET_WIFI_START, nowMs);
        } else if (ssdp_listen_begin(&s_netLink) == ESP_OK) {
          netTransition(QLCP_NET_DISCOVER, nowMs);
        } else {
          netFail(nowMs);
        }
      }
      break;
    default:
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
  {
    NodeLock lock;
    for (uint8_t i = 0U; i < 4; i++) {
      readings[i].sensor_id = i;
      readings[i].unit = QLCP_UNIT_PSI;
      readings[i].value = s_ptValues[i];
    }
    for (uint8_t i = 0U; i < 3; i++) {
      readings[4+i].sensor_id = 4+i;
      readings[4+i].unit = QLCP_UNIT_VOLTS;
      readings[4+i].value = s_hallEffect[i];
    }
    readings[7].sensor_id = 7;
    readings[7].unit = QLCP_UNIT_CELSIUS;
    readings[7].value = s_thermocouple;

    readings[8].sensor_id = 8;
    readings[8].unit = QLCP_UNIT_VOLTS;
    readings[8].value = s_24VoltageSense[0];

    readings[9].sensor_id = 9;
    readings[9].unit = QLCP_UNIT_VOLTS;
    readings[9].value = s_24VoltageSense[1];
  }

  qlcp_data_packet pkt = {};
  fillHeader(pkt.header);
  pkt.sensor_data = readings;
  pkt.sensor_count = kQlcpSensorCount;

  (void)udp_send_data(&s_netLink, &pkt);
}

static void qlcpTask(void* pvParameters) {
  (void)pvParameters;
  for (;;) {
    uint32_t nowMs = millis();
    qlcpNetService(nowMs);
    qlcpTelemetryService(nowMs);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// --- AIM Node Interface Implementation ---

void nodeInit(uint32_t nowMs) {
  (void)nowMs;

  s_nodeMutex = xSemaphoreCreateMutex();

  const int indicatorLeds[] = {pins::kWifiLed, pins::kCanLed, pins::kDebugLed};
  for (size_t i = 0; i < (sizeof(indicatorLeds) / sizeof(indicatorLeds[0])); i++) {
    pinMode(indicatorLeds[i], OUTPUT);
    digitalWrite(indicatorLeds[i], LOW);
  }

  pinMode(pins::kVptEn, OUTPUT);
  digitalWrite(pins::kVptEn, LOW);

  for (size_t i = 0; i < (sizeof(kLocalValveControls) / sizeof(kLocalValveControls[0])); i++) {
    pinMode(static_cast<int>(kLocalValveControls[i].pin), OUTPUT);
    // Safe/Default boot state: De-energized (LOW)
    digitalWrite(static_cast<int>(kLocalValveControls[i].pin), LOW);
    s_valveStates[i] = kLocalValveControls[i].defaultOpen;
  }

  ledcSetup(kAdcClockChannel, kAdcClockHz, kAdcClockDuty);
  ledcAttachPin(pins::kAdcClkin, kAdcClockChannel);
  ledcWrite(kAdcClockChannel, 1U);

  SPI.begin(pins::kAdcSclk, pins::kAdcMiso, pins::kAdcMosi, -1);
  s_adc.init();
  Wire.begin(pins::kHallSda, pins::kHallScl);
  s_hall.init();

  digitalWrite(pins::kVptEn, HIGH);
  s_24VoltageFet[0] = true;
  s_24VoltageFet[1] = true; // remote LCM VPT defaults ON
  s_valveCmds[3].desiredState = 1U; // remote LCM VPT command defaults ON
  s_boardHardwareReady = true;

  // Start WiFi/QLCP state machine
  net_link_init(&s_netLink);
  netTransition(QLCP_NET_WIFI_START, millis());

  // Spawn QLCP background task on Core 0 to isolate from main control loop (on Core 1)
  (void)xTaskCreatePinnedToCore(
      qlcpTask,
      "qlcp_task",
      8192U, // Stack size in bytes
      nullptr,
      1U,    // Priority (standard background task)
      nullptr,
      0      // Core 0 (WiFi/LwIP core)
  );

  LOG_INFO("Pegasus UCM hardware initialized");
}

void nodeUpdate(uint32_t schedulerNowMs) {
  AIM_ASSERT(s_boardHardwareReady);
  (void)schedulerNowMs;
}

void nodeServiceCanTx(uint32_t schedulerNowMs, AimNetwork& aim) {
  AIM_ASSERT(s_boardHardwareReady);

  // Sync RGB led status based on node current state (and debug console status)
  static aim::NodeState s_lastState = static_cast<aim::NodeState>(0xFF);
  aim::NodeState curState = nodeCurrentState();
  
#ifndef FLIGHT_BUILD
  // If console is active, show Amber (Red+Green)
  bool consoleActive = aimConsoleIsActive();
#else
  bool consoleActive = false;
#endif

  if (consoleActive) {
    neopixelWrite(pins::kRgbData, 255, 191, 0); // Amber
  } else if (curState != s_lastState) {
    s_lastState = curState;
    uint8_t r = 0, g = 0, b = 0;
    switch (curState) {
      case aim::NodeState::Nominal:   g = 255; break;
      case aim::NodeState::Fault:     r = 255; break;
      default:                        b = 255; break;
    }
    neopixelWrite(pins::kRgbData, r, g, b);
  }

  // Sample PTs + publish Sensor messages at 10 Hz
  static uint32_t s_lastPtMs = 0U;
  if ((schedulerNowMs - s_lastPtMs) >= kTelemetryPeriodMs) {
    s_lastPtMs = schedulerNowMs;

#ifdef MOCK_HARDWARE
    {
      NodeLock lock;
      s_ptValues[0] = 50.0f + 10.0f * sin(schedulerNowMs / 1000.0f);
      s_ptValues[1] = 50.0f + 10.0f * cos(schedulerNowMs / 1000.0f);
      s_ptValues[2] = 40.0f + 5.0f * sin(schedulerNowMs / 2000.0f);
      s_ptValues[3] = 40.0f + 5.0f * cos(schedulerNowMs / 2000.0f);
      s_hallEffect[0] = 10.0f * sinf(schedulerNowMs / 500.0f);
    }
#else
    float newPtValues[2] = {0.0f, 0.0f};
    float newHallValue = 0.0f;
    int32_t rawData[kAdcChannelCount] = {0};
    if (s_adc.readChannels(rawData)) {
      float volts[kAdcChannelCount] = {0.0f};
      s_adc.computeVoltages(rawData, volts);
      for (size_t i = 0; i < (sizeof(kLocalPtSensors) / sizeof(kLocalPtSensors[0])); i++) {
        const uint8_t channel = kLocalPtSensors[i].adcChannel;
        newPtValues[i] = processPressurePsi(volts[channel]);
      }
    } else {
      LOG_WARN("ADC sample timeout");
    }
    float flux[3] = {0.0f};
    (void)s_hall.getAllFlux(flux);
    newHallValue = flux[0];

    {
      NodeLock lock;
      s_ptValues[0] = newPtValues[0];
      s_ptValues[1] = newPtValues[1];
      s_hallEffect[0] = newHallValue;
    }
#endif

    float local24v = (static_cast<float>(analogRead(pins::kSense24v)) / 4095.0f) * 3.3f * 11.0f;
    {
      NodeLock lock;
      s_24VoltageSense[0] = local24v;
    }

    aim::Msg pt1Msg = {};
    pt1Msg.cls = aim::Class::Sensor;
    pt1Msg.subject = aim::subject::Pt202;
    {
      NodeLock lock;
      pt1Msg.setSensorValue(static_cast<int32_t>(s_ptValues[0] * 100.0f));
    }
    (void)aim.send(pt1Msg);

    aim::Msg pt2Msg = {};
    pt2Msg.cls = aim::Class::Sensor;
    pt2Msg.subject = aim::subject::PtSpare1;
    {
      NodeLock lock;
      pt2Msg.setSensorValue(static_cast<int32_t>(s_ptValues[1] * 100.0f));
    }
    (void)aim.send(pt2Msg);

    aim::Msg solMsg = {};
    solMsg.cls = aim::Class::Sensor;
    solMsg.subject = aim::subject::Volt24Ucm;
    {
      NodeLock lock;
      solMsg.setSensorValue(static_cast<int32_t>(s_24VoltageSense[0] * 1000.0f));
    }
    (void)aim.send(solMsg);

    // Publish local valve state packets (ValveVent & ValveN2Supply)
    aim::Msg ventMsg = {};
    ventMsg.cls = aim::Class::State;
    ventMsg.subject = aim::subject::Av204;
    {
      NodeLock lock;
      ventMsg.b[0] = s_valveStates[0] ? 1U : 0U; // commanded
      ventMsg.b[1] = s_valveStates[0] ? 1U : 0U; // energized
    }
    ventMsg.b[2] = static_cast<uint8_t>(aim::ValveState::Unknown); // hall
    (void)aim.send(ventMsg);

    aim::Msg n2Msg = {};
    n2Msg.cls = aim::Class::State;
    n2Msg.subject = aim::subject::AvSpare;
    {
      NodeLock lock;
      n2Msg.b[0] = s_valveStates[1] ? 1U : 0U;
      n2Msg.b[1] = s_valveStates[1] ? 1U : 0U;
    }
    n2Msg.b[2] = static_cast<uint8_t>(aim::ValveState::Unknown);
    (void)aim.send(n2Msg);
  }

  // Periodic staleness check for LCM link
  {
    NodeLock lock;
    const bool currentlyUp = (schedulerNowMs - s_lowerLastRxMs) < kLowerStaleTimeoutMs;
    if (currentlyUp != s_lowerLinkUp) {
      s_lowerLinkUp = currentlyUp;
      if (s_lowerLinkUp) {
        LOG_INFO("Lower Control link UP");
      } else {
        LOG_WARN("Lower Control link STALE");
      }
    }
  }

  // Remote valve command retry queue
  {
    NodeLock lock;
    for (uint8_t i = 0U; i < 4U; i++) {
      const bool resend = s_valveCmds[i].awaitingAck &&
                          (schedulerNowMs - s_valveCmds[i].lastSentMs >= kValveEchoTimeoutMs);
      if (!s_valveCmds[i].dirty && !resend) {
        continue;
      }

      aim::Msg m = {};
      m.cls = aim::Class::Cmd;
      m.subject = kRemoteValveSubjects[i];
      m.b[0] = s_valveCmds[i].seq;
      m.b[1] = s_valveCmds[i].desiredState;
      if (aim.send(m)) {
        s_valveCmds[i].dirty = false;
        s_valveCmds[i].lastSentMs = schedulerNowMs;
      }
    }
  }
}

void nodeOnRx(const aim::Msg& m, uint32_t nowMs) {
  if (m.source != aim::Source::Lcm) {
    return;
  }

  NodeLock lock;
  s_lowerLastRxMs = nowMs;

  if (m.cls == aim::Class::Ack) {
    const uint8_t idx = remoteValveIndex(m.subject);
    if (idx != 0xFF && m.b[0] == s_valveCmds[idx].seq) {
      s_valveCmds[idx].awaitingAck = false;
      LOG_INFO("ACK received from LCM: subject=%u result=%u", m.subject, m.b[1]);
    }
  } else if (m.cls == aim::Class::State) {
    bool state = (m.b[0] == 1);
    if (m.subject == aim::subject::Av203) {
      s_valveStates[2] = state;
      s_24VoltageFet[2] = (m.b[1] == 1); // energized
    } else if (m.subject == aim::subject::Av205) {
      s_valveStates[3] = state;
    } else if (m.subject == aim::subject::PwrPtLcm) {
      s_24VoltageFet[1] = state;
    } else if (m.subject == aim::subject::PwrSolLcm) {
      s_24VoltageFet[2] = state;
    }
  } else if (m.cls == aim::Class::Sensor) {
    float val = static_cast<float>(m.sensorValue());

    switch (m.subject) {
      case aim::subject::Pt204:
        s_ptValues[2] = val / 100.0f; // scaled PSI
        break;
      case aim::subject::PtSpare2:
        s_ptValues[3] = val / 100.0f; // scaled PSI
        break;
      case aim::subject::TcLowerValve:
        s_thermocouple = val / 100.0f; // scaled degC
        break;
      case aim::subject::VoltSolLcm:
        s_24VoltageSense[1] = val / 1000.0f; // scaled V
        break;
      default:
        break;
    }
  }
}

aim::NodeState nodeCurrentState() {
  return aim::NodeState::Nominal;
}

uint16_t nodeErrorBits() {
  return 0U;
}

// Accessors implementation
bool nodeGetValveState(uint8_t index) {
  NodeLock lock;
  if (index >= 4U) return false;
  return s_valveStates[index];
}

bool nodeGet24vFetState(uint8_t index) {
  NodeLock lock;
  if (index >= 3U) return false;
  return s_24VoltageFet[index];
}

float nodeGetPtValue(uint8_t index) {
  NodeLock lock;
  if (index >= 4U) return 0.0f;
  return s_ptValues[index];
}

float nodeGetHallEffect(uint8_t index) {
  NodeLock lock;
  if (index >= 3U) return 0.0f;
  return s_hallEffect[index];
}

float nodeGet24vSense(uint8_t index) {
  NodeLock lock;
  if (index >= 2U) return 0.0f;
  return s_24VoltageSense[index];
}

float nodeGetThermocouple() {
  NodeLock lock;
  return s_thermocouple;
}

#ifndef FLIGHT_BUILD
static void hookStatusSnapshot(Stream& out) {
  NodeLock lock;
  out.printf("V_VENT=%s\n", s_valveStates[0] ? "OPEN" : "CLOSED");
  out.printf("V_N2=%s\n", s_valveStates[1] ? "OPEN" : "CLOSED");
  out.printf("V_FILL_DUMP=%s\n", s_valveStates[2] ? "OPEN" : "CLOSED");
  out.printf("V_MAIN=%s\n", s_valveStates[3] ? "OPEN" : "CLOSED");
  out.printf("PT1 (Run Tank)=%.2f PSI\n", s_ptValues[0]);
  out.printf("PT2 (Pre Injector)=%.2f PSI\n", s_ptValues[1]);
  out.printf("PT3 (Chamber)=%.2f PSI\n", s_ptValues[2]);
  out.printf("PT4=%.2f PSI\n", s_ptValues[3]);
  out.printf("TC=%.2f C\n", s_thermocouple);
  out.printf("24V (Local)=%.2f V\n", s_24VoltageSense[0]);
  out.printf("24V (Remote)=%.2f V\n", s_24VoltageSense[1]);
  out.printf("VPT FET=%s\n", s_24VoltageFet[0] ? "ON" : "OFF");
  out.printf("Remote VPT FET=%s\n", s_24VoltageFet[1] ? "ON" : "OFF");
  out.printf("Remote VSOL FET=%s\n", s_24VoltageFet[2] ? "ON" : "OFF");
}

static void hookNetworkStatus(Stream& out) {
  NodeLock lock;
  out.printf("net=%s ip=%s rssi=%d qlcp=%u svr=%s:%u/%u stream=%uHz lower=%s\n",
    WIFI_SSID, WiFi.localIP().toString().c_str(), WiFi.RSSI(),
    s_netState,
    s_netLink.server_ip, s_netLink.server_tcp_port, s_netLink.server_udp_port,
    static_cast<unsigned>(s_streamFrequencyHz),
    s_lowerLinkUp ? "OK" : "STALE");
}

static void hookSetValve(Stream& out) {
  int index = -1;
  int state = -1;
  if (out.read() == ' ' && (index = out.read()) >= '0' && index <= '4') {
    if (out.read() == ' ' && (state = out.read()) >= '0' && state <= '1') {
      uint8_t valveIdx = index - '0';
      bool open = (state == '1');
      if (setValveByIndex(valveIdx, open)) {
        out.printf("Set valve %d to %d\n", valveIdx, open);
      } else {
        out.println("Set valve failed");
      }
      return;
    }
  }
  out.println("Invalid command format. Use: v <index 0-4> <state 0-1>");
}

static const AimConsoleHook s_consoleHooks[] = {
  {'p', "status snapshot", hookStatusSnapshot},
  {'n', "network status", hookNetworkStatus},
  {'v', "set valve (v <idx> <0|1>)", hookSetValve},
};

const AimConsoleHook* nodeConsoleHooks(uint8_t& count) {
  count = sizeof(s_consoleHooks) / sizeof(s_consoleHooks[0]);
  return s_consoleHooks;
}
#endif
