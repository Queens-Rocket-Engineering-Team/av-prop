#include "node.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <ADS131M04.h>
#include <TMAG5273.h>
#include <SPI.h>
#include <Wire.h>
#include <logger.h>
#include <aim_control.h>
#include <cstring>
#include <WiFi.h>

extern "C" {
#include "wifi_tools.h"
#include <qlcp_lib.h>
}

#define WIFI_SSID ""
#define WIFI_PASS ""

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

// Seven controls, indexed to match the QLCP control_id / cmdId contract.
// Two local valves + one local power FET that this board owns, plus four LCM
// controls driven remotely over CAN.
enum UcmControl : uint8_t {
  kCtrlAv204,     // 0: local vent
  kCtrlAvSpare,   // 1: local, No N2 valve (do not energize)
  kCtrlAv203,     // 2: remote (LCM)
  kCtrlAv205,     // 3: remote (LCM)
  kCtrlPwrPtUcm,  // 4: PwrPtUcm  local  PT power FET
  kCtrlPwrSolLcm, // 5: PwrSolLcm remote (LCM)
  kCtrlPwrPtLcm,  // 6: PwrPtLcm  remote (LCM)
  kCtrlCount
};
static aim::Control s_controls[kCtrlCount];

// Four PTs by catalog subject: two sampled locally off the UCM ADC, two received
// from the LCM over CAN. Order matches the QLCP sensor config (Pt202..PtSpare2).
enum UcmPt : uint8_t {
  kPtPt202,     // 0: local  — UCM ADC
  kPtPtSpare1,  // 1: local  — UCM ADC
  kPtPt204,     // 2: remote — LCM
  kPtPtSpare2,  // 3: remote — LCM
  kPtCount
};
static float s_ptValues[kPtCount] = {0.0f, 0.0f, 0.0f, 0.0f};    // PSI
static float s_hallEffect[3]      = {0.0f, 0.0f, 0.0f};           // 0: local Hall 1, 1-2: remote Hall 2-3
static float s_24VoltageSense[2]  = {0.0f, 0.0f};                 // 0: local 24V sense, 1: remote VSOL sense
static float s_thermocouple       = 0.0f;                         // remote TC

static uint32_t s_lowerLastRxMs = 0U;
static bool     s_lowerLinkUp   = false;

static ADS131M04 s_adc(-1, pins::kAdcDrdy, &SPI);
static TMAG5273 s_hall(pins::kHallI2cAddr, &Wire);

constexpr uint8_t kAdcClockChannel = 0U;
constexpr uint32_t kAdcClockHz = 8192000U;
constexpr uint8_t kAdcClockDuty = 1U;
constexpr uint32_t kTelemetryPeriodMs = 100U;
constexpr uint32_t kLowerStaleTimeoutMs = 1000U;

constexpr size_t kAdcChannelCount = 4U;

constexpr float kPtShuntResistanceOhms = 62.0f;
constexpr float kPtMaxPsi = 100.0f;

// ADC channel for each UCM-local PT.
constexpr uint8_t kAdcChPt202    = 0U;
constexpr uint8_t kAdcChPtSpare1 = 1U;

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

// Command a control by its QLCP/cmdId index. Local controls actuate immediately;
// remote controls queue a Cmd that nodeServiceCanTx sends to the LCM.
static bool setControlByIndex(uint8_t index, bool open) {
  if (index >= kCtrlCount) {
    return false;
  }
  NodeLock lock;
  controlSet(s_controls[index], open);
  return true;
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
      if (setControlByIndex(cmdId, open)) {
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
          controlData[i].control_state = controlGet(s_controls[i]) ? QLCP_CS_OPEN : QLCP_CS_CLOSED;
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
    for (uint8_t i = 0U; i < kPtCount; i++) {
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

  // Controls. openLevel = the GPIO level that means logical-open:
  // All local controls boot de-energized (safe); remote controls assume the LCM's
  // own boot defaults. The local PT rail is switched on once hardware is ready.
  controlInitLocal(s_controls[kCtrlAv204], aim::subject::Av204, pins::kSol1En, LOW);
  controlInitLocal(s_controls[kCtrlPwrPtUcm], aim::subject::PwrPtUcm, pins::kVptEn,  HIGH);
  controlInitRemote(s_controls[kCtrlAv203], aim::subject::Av203, true);
  controlInitRemote(s_controls[kCtrlAv205], aim::subject::Av205, false);
  controlInitRemote(s_controls[kCtrlPwrSolLcm], aim::subject::PwrSolLcm, false);
  controlInitRemote(s_controls[kCtrlPwrPtLcm], aim::subject::PwrPtLcm, true);

  ledcSetup(kAdcClockChannel, kAdcClockHz, kAdcClockDuty);
  ledcAttachPin(pins::kAdcClkin, kAdcClockChannel);
  ledcWrite(kAdcClockChannel, 1U);

  SPI.begin(pins::kAdcSclk, pins::kAdcMiso, pins::kAdcMosi, -1);
  s_adc.init();
  // NOTE: add back after fixing hall library
//  Wire.begin(pins::kHallSda, pins::kHallScl);
//  s_hall.init();

  controlSet(s_controls[kCtrlPwrPtUcm], true); // local PT power on once hardware is up
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
      s_ptValues[kPtPt202]    = 50.0f + 10.0f * sin(schedulerNowMs / 1000.0f);
      s_ptValues[kPtPtSpare1] = 50.0f + 10.0f * cos(schedulerNowMs / 1000.0f);
      s_ptValues[kPtPt204]    = 40.0f + 5.0f * sin(schedulerNowMs / 2000.0f);
      s_ptValues[kPtPtSpare2] = 40.0f + 5.0f * cos(schedulerNowMs / 2000.0f);
      s_hallEffect[0] = 10.0f * sinf(schedulerNowMs / 500.0f);
    }
#else
    float ptPt202 = 0.0f;
    float ptPtSpare1 = 0.0f;
    float newHallValue = 0.0f;
    int32_t rawData[kAdcChannelCount] = {0};
    if (s_adc.readChannels(rawData)) {
      float volts[kAdcChannelCount] = {0.0f};
      s_adc.computeVoltages(rawData, volts);
      ptPt202    = processPressurePsi(volts[kAdcChPt202]);
      ptPtSpare1 = processPressurePsi(volts[kAdcChPtSpare1]);
    } else {
      LOG_WARN("ADC sample timeout");
    }

  // NOTE: add back after fixing hall library
//    newHallValue = s_hall.getFluxMagnitude();

    {
      NodeLock lock;
      s_ptValues[kPtPt202]    = ptPt202;
      s_ptValues[kPtPtSpare1] = ptPtSpare1;
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
      pt1Msg.setSensorValue(static_cast<int32_t>(s_ptValues[kPtPt202] * 100.0f));
    }
    (void)aim.send(pt1Msg);

    aim::Msg pt2Msg = {};
    pt2Msg.cls = aim::Class::Sensor;
    pt2Msg.subject = aim::subject::PtSpare1;
    {
      NodeLock lock;
      pt2Msg.setSensorValue(static_cast<int32_t>(s_ptValues[kPtPtSpare1] * 100.0f));
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

    // Publish local valve STATE (Av204 Vent, AvSpare N2).
    for (uint8_t i = kCtrlAv204; i <= kCtrlAvSpare; i++) {
      aim::Msg sm = {};
      {
        NodeLock lock;
        controlBuildState(s_controls[i], sm);
      }
      sm.b[2] = static_cast<uint8_t>(aim::ValveState::Unknown); // hall (not sensed)
      (void)aim.send(sm);
    }
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

  // Service control CAN traffic: remote Cmd (re)sends. Local controls own no
  // commanded subject here, so their service is a no-op.
  {
    NodeLock lock;
    for (uint8_t i = 0U; i < kCtrlCount; i++) {
      controlServiceTx(s_controls[i], schedulerNowMs, aim);
    }
  }
}

void nodeOnRx(const aim::Msg& m, uint32_t nowMs) {
  if (m.source != aim::Source::Lcm) {
    return;
  }

  NodeLock lock;
  s_lowerLastRxMs = nowMs;

  if (m.cls == aim::Class::Ack || m.cls == aim::Class::State) {
    // Route the LCM's Ack/State to whichever remote control owns the subject.
    for (uint8_t i = 0U; i < kCtrlCount; i++) {
      (void)controlOnRx(s_controls[i], m);
    }
  } else if (m.cls == aim::Class::Sensor) {
    float val = static_cast<float>(m.sensorValue());

    switch (m.subject) {
      case aim::subject::Pt204:
        s_ptValues[kPtPt204] = val / 100.0f; // scaled PSI
        break;
      case aim::subject::PtSpare2:
        s_ptValues[kPtPtSpare2] = val / 100.0f; // scaled PSI
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

#ifndef FLIGHT_BUILD
// "UNKNOWN" until the LCM's first State frame confirms a remote control.
static const char* controlStr(const aim::Control& c, const char* hi, const char* lo) {
  if (!controlConfirmed(c)) return "UNKNOWN";
  return controlGet(c) ? hi : lo;
}

static void hookStatusSnapshot(Stream& out) {
  NodeLock lock;
  out.printf("AV204_VENT=%s\n", controlStr(s_controls[kCtrlAv204], "OPEN", "CLOSED"));
  out.printf("AVSpare=%s\n", controlStr(s_controls[kCtrlAvSpare], "OPEN", "CLOSED"));
  out.printf("AV203_FILL_DUMP=%s\n", controlStr(s_controls[kCtrlAv203], "OPEN", "CLOSED"));
  out.printf("AV205_MAIN=%s\n", controlStr(s_controls[kCtrlAv205], "OPEN", "CLOSED"));
  out.printf("Pt202 (Run Tank, local)=%.2f PSI\n", s_ptValues[kPtPt202]);
  out.printf("PtSpare1 (local)=%.2f PSI\n", s_ptValues[kPtPtSpare1]);
  out.printf("Pt204 (Chamber, remote)=%.2f PSI\n", s_ptValues[kPtPt204]);
  out.printf("PtSpare2 (remote)=%.2f PSI\n", s_ptValues[kPtPtSpare2]);
  out.printf("TC (remote)=%.2f C\n", s_thermocouple);
  out.printf("24V (Local)=%.2f V\n", s_24VoltageSense[0]);
  out.printf("24V (Remote)=%.2f V\n", s_24VoltageSense[1]);
  out.printf("VPT FET=%s\n", controlStr(s_controls[kCtrlPwrPtUcm], "ON", "OFF"));
  out.printf("Remote VPT FET=%s\n", controlStr(s_controls[kCtrlPwrPtLcm], "ON", "OFF"));
  out.printf("Remote VSOL FET=%s\n", controlStr(s_controls[kCtrlPwrSolLcm], "ON", "OFF"));
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
  if (out.read() == ' ' && (index = out.read()) >= '0' && index <= '6') {
    if (out.read() == ' ' && (state = out.read()) >= '0' && state <= '1') {
      uint8_t ctrlIdx = index - '0';
      bool open = (state == '1');
      if (setControlByIndex(ctrlIdx, open)) {
        out.printf("Set control %d to %d\n", ctrlIdx, open);
      } else {
        out.println("Set control failed");
      }
      return;
    }
  }
  out.println("Invalid command format. Use: v <index 0-6> <state 0-1>");
}

static const AimConsoleHook s_consoleHooks[] = {
  {'p', "status snapshot", hookStatusSnapshot},
  {'n', "network status", hookNetworkStatus},
  {'v', "set control (v <idx 0-6> <0|1>)", hookSetValve},
};

const AimConsoleHook* nodeConsoleHooks(uint8_t& count) {
  count = sizeof(s_consoleHooks) / sizeof(s_consoleHooks[0]);
  return s_consoleHooks;
}
#endif
