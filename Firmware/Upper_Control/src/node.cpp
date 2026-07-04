#include "node.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <ADS131M04.h>
#include <SPI.h>
#include <logger.h>
#include <aim_control.h>
#include <aim_sensor.h>
#include <aim_job.h>
#include <prop_testing.h>
#include <cstring>
#include <WiFi.h>

extern "C" {
#include "wifi_tools.h"
#include <qlcp_lib.h>
}

#define WIFI_SSID "propnet"
#define WIFI_PASS "propteambestteam"

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

// Seven scalar sensors by catalog subject — four PTs, a thermocouple, two voltage
// senses — sampled locally off the UCM ADC or received from the LCM over CAN.
// Indices 0..3 are the PTs, matching the QLCP sensor_id contract (Pt202..PtSpare2).
enum UcmSensor : uint8_t {
  kSenPt202,     // 0: local  — UCM ADC
  kSenPtSpare1,  // 1: local  — UCM ADC
  kSenPt204,     // 2: remote — LCM
  kSenPtSpare2,  // 3: remote — LCM
  kSenTc,        // 4: remote — LCM thermocouple
  kSenVolt24,    // 5: local  — UCM 24V sense
  kSenVsol,      // 6: remote — LCM VSOL sense
  kSenCount
};
static aim::Sensor s_sensors[kSenCount];
static uint32_t s_lowerLastRxMs = 0U;
static bool     s_lowerLinkUp   = false;

static ADS131M04 s_adc(-1, pins::kAdcDrdy, &SPI);
constexpr uint8_t kAdcClockChannel = 0U;
constexpr uint32_t kAdcClockHz = 8192000U;
constexpr uint8_t kAdcClockDuty = 1U;
constexpr uint32_t kTelemetryPeriodMs = 100U;
constexpr uint32_t kLowerStaleTimeoutMs = 1000U;

constexpr size_t kAdcChannelCount = 4U;

// ADC channel for each UCM-local PT.
constexpr uint8_t kAdcChPt202    = 0U;
constexpr uint8_t kAdcChPtSpare1 = 1U;

static aim::Job s_adcJob = {100U, 0U};
static aim::Job s_voltSenseJob = {500U, 0U};
static aim::Job s_broadcastJob = {500U, 0U};

constexpr char kBoardQlcpConfigJson[] = R"json({
  "device_name": "PEGASUS-UPPER",
  "device_type": "Sensor Monitor",
  "sensor_info": {
    "pressure_transducer": {
      "PT202": {
        "unit": "PSI"
      },
      "PTSPARE1": {
        "unit": "PSI"
      },
      "PT204": {
        "unit": "PSI"
      },
      "PTSPARE2": {
        "unit": "PSI"
      }
    },
    "thermocouple": {
      "TcLowerValve": {
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
    "AV204": {
      "default_state": "OPEN",
      "type": "solenoid"
    },
    "AVSPARE": {
      "default_state": "CLOSED",
      "type": "solenoid"
    },
    "AV203": {
      "default_state": "OPEN",
      "type": "solenoid"
    },
    "AV205": {
      "default_state": "CLOSED",
      "type": "solenoid"
    },
    "PwrPtUpper": {
      "default_state": "OPEN",
      "type": "relay"
    },
    "PwrSolLower": {
      "default_state": "OPEN",
      "type": "relay"
    },
    "PwrPtLower": {
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

static void fillHeader(qlcp_header& header) {
  header.sequence = static_cast<uint8_t>(s_sequence++);
  header.timestamp = s_tsOffset + millis();
}

static void netTransition(QlcpNetState next, uint32_t nowMs) {
  LOG_DEBUG("QLCP net: %u -> %u", s_netState, next);
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
static bool setControlByIndex(uint8_t index, bool open, bool& confirmedOut) {
  if (index >= kCtrlCount) {
    return false;
  }
  NodeLock lock;
  controlSet(s_controls[index], open);
  confirmedOut = s_controls[index].confirmed;
  return true;
}

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
      bool confirmed = false;
      if (setControlByIndex(cmdId, open, confirmed) && confirmed) {
        sendAck(QLCP_PT_CONTROL, in.payload_data.header_only.sequence);
      } else {
        sendNack(QLCP_PT_CONTROL, in.payload_data.header_only.sequence, QLCP_ERR_HARDWARE_FAULT);
      }
      break;
    }
    case QLCP_PT_STATUS_REQUEST: {
      qlcp_control_data controlData[kCtrlCount] = {};
      {
        NodeLock lock;
        for (uint8_t i = 0U; i < kCtrlCount; i++) {
          controlData[i].control_id = i;
          controlData[i].control_state = controlGet(s_controls[i]) ? QLCP_CS_OPEN : QLCP_CS_CLOSED;
        }
      }
      qlcp_server_payload out = {};
      out.packet_type = QLCP_PT_STATUS;
      fillHeader(out.payload_data.status.header);
      out.payload_data.status.control_data = controlData;
      out.payload_data.status.control_count = kCtrlCount;
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
      LOG_DEBUG("WiFi Connecting to SSID: %s", WIFI_SSID);
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

  qlcp_sensor_data readings[kSenCount] = {};
  {
    NodeLock lock;
    for (uint8_t i = 0U; i < 4U; i++) {  // PTs occupy sensor_id 0..3 = s_sensors[0..3]
      readings[i].sensor_id = i;
      readings[i].unit = QLCP_UNIT_PSI;
      readings[i].value = sensorEng(s_sensors[i]);
    }
    readings[4].sensor_id = kSenTc;
    readings[4].unit = QLCP_UNIT_CELSIUS;
    readings[4].value = sensorEng(s_sensors[kSenTc]);

    readings[5].sensor_id = kSenVolt24;
    readings[5].unit = QLCP_UNIT_VOLTS;
    readings[5].value = sensorEng(s_sensors[kSenVolt24]);

    readings[6].sensor_id = kSenVsol;
    readings[6].unit = QLCP_UNIT_VOLTS;
    readings[6].value = sensorEng(s_sensors[kSenVsol]);
  }

  qlcp_data_packet pkt = {};
  fillHeader(pkt.header);
  pkt.sensor_data = readings;
  pkt.sensor_count = kSenCount;

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

static void updateLed(aim::NodeState state) {
  static aim::NodeState s_lastState = static_cast<aim::NodeState>(0xFF);

  if (aimConsoleIsActive()) {
    if (s_lastState != static_cast<aim::NodeState>(0xFE)) {
      s_lastState = static_cast<aim::NodeState>(0xFE);
      neopixelWrite(pins::kRgbData, 255, 191, 0);
    }
    return;
  }

  if (state == s_lastState) return;
  s_lastState = state;
  uint8_t r = 0, g = 0, b = 0;
  switch (state) {
    case aim::NodeState::Nominal: g = 255; break;
    case aim::NodeState::Fault:   r = 255; break;
    default:                      b = 255; break;
  }
  neopixelWrite(pins::kRgbData, r, g, b);
}

void nodeInit() {

  s_nodeMutex = xSemaphoreCreateMutex();

  const int indicatorLeds[] = {pins::kWifiLed, pins::kCanLed, pins::kDebugLed};
  for (size_t i = 0; i < (sizeof(indicatorLeds) / sizeof(indicatorLeds[0])); i++) {
    pinMode(indicatorLeds[i], OUTPUT);
    digitalWrite(indicatorLeds[i], LOW);
  }

  // Controls. defaultOpen = logical state at the de-energized rest:
  // All local controls boot de-energized (safe); remote controls assume the LCM's
  // own boot defaults.
  controlInitLocal (s_controls[kCtrlAv204],     "AV204_VENT", aim::subject::Av204,     pins::kSol1En, true);
  controlInitLocal (s_controls[kCtrlPwrPtUcm],  "PwrPtUcm",   aim::subject::PwrPtUcm,  pins::kVptEn,  true);
  controlInitRemote(s_controls[kCtrlAv203],     "AV203_FILL", aim::subject::Av203,     true);
  controlInitRemote(s_controls[kCtrlAv205],     "AV205_MAIN", aim::subject::Av205,     false);
  controlInitRemote(s_controls[kCtrlPwrSolLcm], "PwrSolLcm",  aim::subject::PwrSolLcm, true);
  controlInitRemote(s_controls[kCtrlPwrPtLcm],  "PwrPtLcm",   aim::subject::PwrPtLcm,  true);
  s_controls[kCtrlAvSpare].name = "AVSpare";  // do-not-energize spare; named for console only

  // Sensors. toEng converts the catalog-scaled wire integer to engineering units.
  sensorInitLocal (s_sensors[kSenPt202],    "Pt202 (Run Tank, local)", aim::subject::Pt202,        0.01f,  "PSI");
  sensorInitLocal (s_sensors[kSenPtSpare1], "PtSpare1 (local)",        aim::subject::PtSpare1,     0.01f,  "PSI");
  sensorInitRemote(s_sensors[kSenPt204],    "Pt204 (Chamber, remote)", aim::subject::Pt204,        0.01f,  "PSI");
  sensorInitRemote(s_sensors[kSenPtSpare2], "PtSpare2 (remote)",       aim::subject::PtSpare2,     0.01f,  "PSI");
  sensorInitRemote(s_sensors[kSenTc],       "TC (remote)",             aim::subject::TcLowerValve, 0.01f,  "C");
  sensorInitLocal (s_sensors[kSenVolt24],   "24V (Local)",             aim::subject::Volt24Ucm,    0.001f, "V");
  sensorInitRemote(s_sensors[kSenVsol],     "24V (Remote)",            aim::subject::VoltSolLcm,   0.001f, "V");

  ledcSetup(kAdcClockChannel, kAdcClockHz, kAdcClockDuty);
  ledcAttachPin(pins::kAdcClkin, kAdcClockChannel);
  ledcWrite(kAdcClockChannel, 1U);

  SPI.begin(pins::kAdcSclk, pins::kAdcMiso, pins::kAdcMosi, -1);
  s_adc.init();

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

void nodeUpdate(uint32_t nowMs) {
  AIM_ASSERT(s_boardHardwareReady);

  updateLed(nodeCurrentState());

  // Sample PTs every tick (ADC read is non-blocking; jobs gate only CAN sends)
  {
    int32_t rawData[kAdcChannelCount] = {0};
    if (s_adc.readChannels(rawData)) {
      float volts[kAdcChannelCount] = {0.0f};
      s_adc.computeVoltages(rawData, volts);
      NodeLock lock;
      sensorSampleEng(s_sensors[kSenPt202], processPT(volts[kAdcChPt202]));
      sensorSampleEng(s_sensors[kSenPtSpare1], processPT(volts[kAdcChPtSpare1]));
    }
  }

  // Voltage sense every tick
  {
    float local24v = (static_cast<float>(analogRead(pins::kSense24v)) / 4095.0f) * 3.3f * 11.0f;
    NodeLock lock;
    sensorSampleEng(s_sensors[kSenVolt24], local24v);
  }

  // LCM link staleness check
  {
    NodeLock lock;
    const bool currentlyUp = (nowMs - s_lowerLastRxMs) < kLowerStaleTimeoutMs;
    if (currentlyUp != s_lowerLinkUp) {
      s_lowerLinkUp = currentlyUp;
      LOG_DEBUG("Lower Control link %s", s_lowerLinkUp ? "UP" : "STALE");
    }
  }
}

void nodeServiceCanTx(uint32_t nowMs, AimNetwork& aim) {
  AIM_ASSERT(s_boardHardwareReady);

  // Sensor frame broadcasts at 10 Hz
  if (s_adcJob.due(nowMs)) {
    aim::Msg pt1Msg = {};
    aim::Msg pt2Msg = {};
    {
      NodeLock lock;
      sensorBuildFrame(s_sensors[kSenPt202], pt1Msg);
      sensorBuildFrame(s_sensors[kSenPtSpare1], pt2Msg);
    }
    (void)aim.send(pt1Msg);
    (void)aim.send(pt2Msg);
  }

  // Voltage frame at 2 Hz
  if (s_voltSenseJob.due(nowMs)) {
    aim::Msg solMsg = {};
    {
      NodeLock lock;
      sensorBuildFrame(s_sensors[kSenVolt24], solMsg);
    }
    (void)aim.send(solMsg);
  }

  // Local valve STATE broadcast at 2 Hz
  if (s_broadcastJob.due(nowMs)) {
    for (uint8_t i = kCtrlAv204; i <= kCtrlAvSpare; i++) {
      aim::Msg sm = {};
      {
        NodeLock lock;
        controlBuildState(s_controls[i], sm);
      }
      (void)aim.send(sm);
    }
  }

  // Service control CAN traffic: remote Cmd (re)sends.
  {
    NodeLock lock;
    for (uint8_t i = 0U; i < kCtrlCount; i++) {
      controlServiceTx(s_controls[i], nowMs, aim);
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
    for (uint8_t i = 0U; i < kSenCount; i++) {
      if (sensorOnRx(s_sensors[i], m)) {
        break;
      }
    }
  }
}

aim::NodeState nodeCurrentState() {
  return s_lowerLinkUp ? aim::NodeState::Nominal : aim::NodeState::Fault;
}

uint16_t nodeErrorBits() {
  return 0U;
}

#ifndef FLIGHT_BUILD
// "UNKNOWN" until the LCM's first State frame confirms a remote control.
static void hookStatusSnapshot(Stream& out) {
  NodeLock lock;
  for (uint8_t i = 0U; i < kCtrlCount; i++) {
    out.printf("%-12s %s\n", s_controls[i].name,
               aim::controlStr(s_controls[i]));
  }
  for (uint8_t i = 0U; i < kSenCount; i++) {
    const aim::Sensor& s = s_sensors[i];
    if (sensorFresh(s)) {
      out.printf("%-26s %.2f %s\n", s.name, sensorEng(s), s.unit);
    } else {
      out.printf("%-26s UNKNOWN\n", s.name);
    }
  }
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
  int index = out.read();
  if (index == ' ') {
    index = out.read();
  }
  if (index < '0' || index > '6') {
    NodeLock lock;
    for (uint8_t i = 0U; i < kCtrlCount; i++) {
      out.printf("  %u: %-12s %s\n", i, s_controls[i].name,
                 aim::controlStr(s_controls[i]));
    }
    out.println("Usage: v <0-6> <0|1>");
    return;
  }
  uint8_t ctrlIdx = index - '0';
  int state = out.read();
  if (state == ' ') {
    state = out.read();
  }
  if (state < '0' || state > '1') {
    out.println("Usage: v <0-6> <0|1>");
    return;
  }
  bool open = (state == '1');
  bool dummyConfirmed = false;
  if (setControlByIndex(ctrlIdx, open, dummyConfirmed)) {
    out.printf("%s -> %s\n", s_controls[ctrlIdx].name, open ? "OPEN/ON" : "CLOSED/OFF");
  } else {
    out.println("Set control failed");
  }
}

static const AimConsoleHook s_consoleHooks[] = {
  {'p', "status snapshot", hookStatusSnapshot},
  {'n', "network status", hookNetworkStatus},
  {'v', "valve/FET control", hookSetValve},
};

const AimConsoleHook* nodeConsoleHooks(uint8_t& count) {
  count = sizeof(s_consoleHooks) / sizeof(s_consoleHooks[0]);
  return s_consoleHooks;
}
#endif
