#include "node.h"

#include <ADS131M04.h>
#include <SPI.h>
#include <logger.h>

#include <cstring>
#include <WiFi.h>
#include <atomic>

extern "C" {
#include "wifi_tools.h"
#include <qlcp_lib.h>
}

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
  "device_type": "Sensor Monitor",
  "sensor_info": {
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

ADS131M04 g_adc(-1, ADC_DRDY_PIN, &SPI);

bool g_nodeHardwareReady = false;
uint32_t g_lastTelemetryMs = 0U;
bool g_valveStates[4] = {false, false, false, false};
float g_ptValues[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static AimNetwork* s_aimPtr = nullptr;

#ifndef FLIGHT_BUILD
struct QlcpStats {
  uint32_t tcpRxCount = 0;
  uint32_t tcpTxCount = 0;
  uint32_t udpTxCount = 0;
  uint32_t configSent = 0;
  uint32_t timesyncCount = 0;
  uint32_t heartbeatRx = 0;
  uint32_t controlRx = 0;
  uint32_t statusRequestRx = 0;
  uint32_t streamStartRx = 0;
  uint32_t streamStopRx = 0;
};
static QlcpStats g_qlcpStats = {};

struct CanStats {
  uint32_t rxCount = 0;
  uint32_t txCount = 0;
  uint32_t sensorRx = 0;
  uint32_t valveRx = 0;
  uint32_t valveTx = 0;
};
static CanStats g_canStats = {};
#endif

static network_ctx_t g_netCtx = {};
static std::atomic<uint16_t> g_sequence{0};
static std::atomic<uint32_t> g_tsOffset{0};
static std::atomic<uint16_t> g_streamFrequencyHz{0};
static EventGroupHandle_t g_qlcpEventGroup = nullptr;

#define QLCP_STREAM_ENABLE_BIT (1 << 0)
#define QLCP_SINGLE_READ_BIT (1 << 1)

int s_localValveIndexFromEndpoint(uint8_t endpointId) {
  if (endpointId == NODE_ENDPOINT_VALVE1) return 0;
  if (endpointId == NODE_ENDPOINT_VALVE2) return 1;
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

  uint32_t payload = 0U;
  static_assert(sizeof(payload) == sizeof(value), "float payload packing assumes 32-bit float");
  std::memcpy(&payload, &value, sizeof(payload));
  const bool ok = aim.sendTimedPktEx(endpointId, networkNowMs, payload, AIM_DEST_COMMS, AIM_TYP_SENSOR);
#ifndef FLIGHT_BUILD
  if (ok) {
    g_canStats.txCount++;
  }
#endif
  return ok;
}

}  // namespace

static void s_sendAck(uint8_t ackType, uint16_t ackSeq) {
  qlcp_server_payload out = {};
  out.packet_type = QLCP_PT_ACK;
  out.payload_data.ack.header.sequence = g_sequence++;
  out.payload_data.ack.header.timestamp = g_tsOffset + millis();
  out.payload_data.ack.ack_packet_type = ackType;
  out.payload_data.ack.ack_sequence = ackSeq;
  (void)xQueueSend(g_netCtx.tcp_send_queue_handle, &out, MESSAGE_QUEUE_TIMEOUT);
#ifndef FLIGHT_BUILD
  g_qlcpStats.tcpTxCount++;
#endif
}

static void s_sendNack(uint8_t nackType, uint16_t nackSeq, uint8_t errCode) {
  qlcp_server_payload out = {};
  out.packet_type = QLCP_PT_NACK;
  out.payload_data.nack.header.sequence = g_sequence++;
  out.payload_data.nack.header.timestamp = g_tsOffset + millis();
  out.payload_data.nack.nack_packet_type = nackType;
  out.payload_data.nack.nack_sequence = nackSeq;
  out.payload_data.nack.nack_error_code = errCode;
  (void)xQueueSend(g_netCtx.tcp_send_queue_handle, &out, MESSAGE_QUEUE_TIMEOUT);
#ifndef FLIGHT_BUILD
  g_qlcpStats.tcpTxCount++;
#endif
}

bool nodeInitHardware(void) {
  AIM_ASSERT(!g_nodeHardwareReady);

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
  g_nodeHardwareReady = true;
  g_lastTelemetryMs = millis();
  LOG_INFO("Node hardware initialized");

  AIM_ASSERT(g_nodeHardwareReady);
  return true;
}

void nodeServiceLocalTelemetry(uint32_t schedulerNowMs, uint32_t networkNowMs, AimNetwork& aim) {
  AIM_ASSERT(g_nodeHardwareReady);

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

bool nodeHandleCanPacket(const aimPkt& pkt, uint32_t networkNowMs, AimNetwork& aim) {
#ifndef FLIGHT_BUILD
  g_canStats.rxCount++;
#endif

  if (pkt.dest != NODE_ORIGIN && pkt.dest != AIM_DEST_BROADCAST) {
    return false;
  }

  if (pkt.type == AIM_TYP_VALVE) {
#ifndef FLIGHT_BUILD
    g_canStats.valveRx++;
#endif
    const uint8_t endpointId = pkt.getEndpointId();
    if (endpointId >= NODE_ENDPOINT_LOWER_BASE) {
      const bool openCommand = (pkt.getPayload() != NODE_ACTUATOR_CLOSED);
      // Lower valve state echo received back from Lower Control!
      int valveIdx = endpointId - (NODE_ENDPOINT_LOWER_BASE + 2U);
      if (valveIdx >= 0 && valveIdx < 2) {
        g_valveStates[2 + valveIdx] = openCommand;
        LOG_INFO("Lower valve %d state echo: %d", valveIdx, openCommand);
      }
      return true;
    }
  }

  if (pkt.type == AIM_TYP_SENSOR) {
#ifndef FLIGHT_BUILD
    g_canStats.sensorRx++;
#endif
    const uint8_t endpointId = pkt.getEndpointId();
    if (endpointId >= NODE_ENDPOINT_LOWER_BASE) {
      float val = 0.0f;
      const uint32_t payload = pkt.getPayload();
      std::memcpy(&val, &payload, sizeof(val));

      int ptIdx = endpointId - NODE_ENDPOINT_LOWER_BASE;
      if (ptIdx >= 0 && ptIdx < 2) {
        g_ptValues[2 + ptIdx] = val;
        LOG_DEBUG("Lower PT %d telemetry received: %.2f PSI", ptIdx, val);
      }
    }
    return true;
  }

  return false;
}

void nodeUpdate(uint32_t schedulerNowMs) {
  AIM_ASSERT(g_nodeHardwareReady);

  static uint32_t lastAdcReadMs = 0U;
  if ((schedulerNowMs - lastAdcReadMs) < kTelemetryPeriodMs) {
    return;
  }
  lastAdcReadMs = schedulerNowMs;

#ifdef MOCK_HARDWARE
  g_ptValues[0] = 50.0f + 10.0f * sin(schedulerNowMs / 1000.0f);
  g_ptValues[1] = 50.0f + 10.0f * cos(schedulerNowMs / 1000.0f);
  g_ptValues[2] = 40.0f + 5.0f * sin(schedulerNowMs / 2000.0f);
  g_ptValues[3] = 40.0f + 5.0f * cos(schedulerNowMs / 2000.0f);
#else
  int32_t rawData[kAdcChannelCount] = {0};
  if (!g_adc.readChannels(rawData)) {
    LOG_WARN("ADC sample timeout");
    return;
  }

  float volts[kAdcChannelCount] = {0.0f};
  g_adc.computeVoltages(rawData, volts);

  for (size_t i = 0; i < (sizeof(kLocalPtSensors) / sizeof(kLocalPtSensors[0])); i++) {
    const uint8_t channel = kLocalPtSensors[i].adcChannel;
    g_ptValues[i] = s_processPressurePsi(volts[channel]);
  }
#endif
}

static void qlcpManagerTask(void *pvParams) {
  (void)pvParams;

  LOG_INFO("WiFi Connecting to SSID: %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  LOG_INFO("WiFi Connected! IP: %s", WiFi.localIP().toString().c_str());

  g_netCtx.netif_handle = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");

  esp_err_t err = network_manager_init(&g_netCtx);
  if (err != ESP_OK) {
    LOG_ERROR("QLCP network_manager_init failed");
    vTaskDelete(NULL);
    return;
  }
  LOG_INFO("QLCP network manager started");
  xTaskNotify(g_netCtx.network_manager_handle, SIG_WIFI_CONN, eSetBits);

  qlcp_client_payload payloadIn = {};
  qlcp_server_payload payloadOut = {};

  while (1) {
    EventBits_t wifiBits = xEventGroupGetBits(g_netCtx.wifi_event_group_handle);

    if (!g_netCtx.config_sent && (wifiBits & SERVER_CONNECTED_BIT)) {
      payloadOut.packet_type = QLCP_PT_CONFIG;
      qlcp_config_packet config = {};
      config.header.sequence = g_sequence++;
      config.header.timestamp = g_tsOffset + millis();
      config.config_data = kNodeConfigJson;
      config.config_data_len = sizeof(kNodeConfigJson) - 1;
      payloadOut.payload_data.config = config;

      if (xQueueSend(g_netCtx.tcp_send_queue_handle, &payloadOut, 0) == pdTRUE) {
        g_netCtx.config_sent = true;
#ifndef FLIGHT_BUILD
        g_qlcpStats.configSent++;
        g_qlcpStats.tcpTxCount++;
#endif
        LOG_INFO("Sent CONFIG packet to server");
      }
    }

    if (xQueueReceive(g_netCtx.tcp_recv_queue_handle, &payloadIn, pdMS_TO_TICKS(50)) == pdTRUE) {
#ifndef FLIGHT_BUILD
      g_qlcpStats.tcpRxCount++;
#endif
      switch (payloadIn.packet_type) {
        case QLCP_PT_TIMESYNC: {
          uint32_t serverTime = payloadIn.payload_data.header_only.timestamp;
          g_tsOffset = serverTime - millis();
          LOG_INFO("QLCP timesync completed. Offset: %u ms", g_tsOffset.load());
#ifndef FLIGHT_BUILD
          g_qlcpStats.timesyncCount++;
#endif
          s_sendAck(QLCP_PT_TIMESYNC, payloadIn.payload_data.header_only.sequence);
          break;
        }

        case QLCP_PT_HEARTBEAT: {
#ifndef FLIGHT_BUILD
          g_qlcpStats.heartbeatRx++;
#endif
          s_sendAck(QLCP_PT_HEARTBEAT, payloadIn.payload_data.header_only.sequence);
          break;
        }

        case QLCP_PT_STREAM_START: {
          uint16_t freq = payloadIn.payload_data.stream_start.stream_frequency;
          if (freq > 0) {
            g_streamFrequencyHz = freq;
            xEventGroupSetBits(g_qlcpEventGroup, QLCP_STREAM_ENABLE_BIT);
            LOG_INFO("QLCP Stream Start at %u Hz", freq);
          }
#ifndef FLIGHT_BUILD
          g_qlcpStats.streamStartRx++;
#endif
          s_sendAck(QLCP_PT_STREAM_START, payloadIn.payload_data.header_only.sequence);
          break;
        }

        case QLCP_PT_STREAM_STOP: {
          xEventGroupClearBits(g_qlcpEventGroup, QLCP_STREAM_ENABLE_BIT);
          g_streamFrequencyHz = 0;
          LOG_INFO("QLCP Stream Stop");
#ifndef FLIGHT_BUILD
          g_qlcpStats.streamStopRx++;
#endif
          s_sendAck(QLCP_PT_STREAM_STOP, payloadIn.payload_data.header_only.sequence);
          break;
        }

        case QLCP_PT_CONTROL: {
          uint8_t cmdId = payloadIn.payload_data.control.command_id;
          uint8_t cmdState = payloadIn.payload_data.control.command_state;
          bool open = (cmdState == QLCP_CS_OPEN);
          bool ok = false;

#ifndef FLIGHT_BUILD
          g_qlcpStats.controlRx++;
#endif
          LOG_INFO("Received control cmdId=%u state=%u", cmdId, cmdState);

          if (cmdId < 2) {
            uint8_t endpoint = (cmdId == 0) ? NODE_ENDPOINT_VALVE1 : NODE_ENDPOINT_VALVE2;
            ok = s_setLocalValveState(endpoint, open);
          } else if (cmdId < 4) {
#ifdef MOCK_HARDWARE
            g_valveStates[cmdId] = open;
            ok = true;
#else
            uint8_t endpoint = (cmdId == 2) ? NODE_ENDPOINT_LOWER_BASE : (NODE_ENDPOINT_LOWER_BASE + 1);
            if (s_aimPtr != nullptr) {
               const uint32_t payload = open ? NODE_ACTUATOR_OPEN : NODE_ACTUATOR_CLOSED;
               ok = s_aimPtr->sendTimedPktEx(endpoint, s_aimPtr->syncedMillis(), payload, AIM_DEST_LPROP, AIM_TYP_VALVE);
#ifndef FLIGHT_BUILD
               if (ok) {
                 g_canStats.txCount++;
                 g_canStats.valveTx++;
               }
#endif
            }
#endif
          }

          if (ok) {
            s_sendAck(QLCP_PT_CONTROL, payloadIn.payload_data.header_only.sequence);
          } else {
            s_sendNack(QLCP_PT_CONTROL, payloadIn.payload_data.header_only.sequence, QLCP_ERR_HARDWARE_FAULT);
          }
          break;
        }

        case QLCP_PT_STATUS_REQUEST: {
#ifndef FLIGHT_BUILD
          g_qlcpStats.statusRequestRx++;
          g_qlcpStats.tcpTxCount++;
#endif
          payloadOut.packet_type = QLCP_PT_STATUS;
          
          qlcp_control_data controlData[4] = {};
          for (uint8_t i = 0; i < 4; i++) {
            controlData[i].control_id = i;
            controlData[i].control_state = g_valveStates[i] ? QLCP_CS_OPEN : QLCP_CS_CLOSED;
          }

          qlcp_status_packet status = {};
          status.header.sequence = g_sequence++;
          status.header.timestamp = g_tsOffset + millis();
          status.control_data = controlData;
          status.control_count = 4;
          status.device_status = QLCP_DS_ACTIVE;

          payloadOut.payload_data.status = status;
          (void)xQueueSend(g_netCtx.tcp_send_queue_handle, &payloadOut, MESSAGE_QUEUE_TIMEOUT);
          break;
        }

        default:
          break;
      }
    }
  }
}

static void qlcpTelemetryTask(void *pvParams) {
  (void)pvParams;

  TickType_t lastWakeTime = xTaskGetTickCount();

  while (1) {
    xEventGroupWaitBits(
        g_qlcpEventGroup,
        QLCP_STREAM_ENABLE_BIT | QLCP_SINGLE_READ_BIT,
        pdFALSE,
        pdFALSE,
        portMAX_DELAY
    );

    uint16_t freq = g_streamFrequencyHz;
    uint32_t periodMs = (freq > 0) ? (1000U / freq) : 1000U;

    qlcp_sensor_data readings[4] = {};
    for (uint8_t i = 0; i < 4; i++) {
      readings[i].sensor_id = i;
      readings[i].unit = QLCP_UNIT_PSI;
      readings[i].value = g_ptValues[i];
    }

    qlcp_data_packet pkt = {};
    pkt.header.sequence = g_sequence++;
    pkt.header.timestamp = g_tsOffset + millis();
    pkt.sensor_data = readings;
    pkt.sensor_count = 4;

    if (xQueueSend(g_netCtx.udp_send_queue_handle, &pkt, MESSAGE_QUEUE_TIMEOUT) == pdTRUE) {
#ifndef FLIGHT_BUILD
      g_qlcpStats.udpTxCount++;
#endif
      (void)xSemaphoreTake(g_netCtx.udp_send_semaphore_handle, pdMS_TO_TICKS(50));
    }

    if (xEventGroupGetBits(g_qlcpEventGroup) & QLCP_SINGLE_READ_BIT) {
      xEventGroupClearBits(g_qlcpEventGroup, QLCP_SINGLE_READ_BIT);
    } else {
      vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(periodMs));
    }
  }
}

static StaticEventGroup_t g_qlcpStaticEventGroup;
static StaticTask_t g_managerTaskBuffer;
static StackType_t g_managerTaskStack[8192];
static StaticTask_t g_telemetryTaskBuffer;
static StackType_t g_telemetryTaskStack[4096];

void nodeStartTasks(AimNetwork& aim) {
  s_aimPtr = &aim;

  g_qlcpEventGroup = xEventGroupCreateStatic(&g_qlcpStaticEventGroup);
  configASSERT(g_qlcpEventGroup);

  xTaskCreateStatic(
    qlcpManagerTask,
    "QLCP Manager",
    8192,
    NULL,
    3,
    g_managerTaskStack,
    &g_managerTaskBuffer
  );

  xTaskCreateStatic(
    qlcpTelemetryTask,
    "QLCP Telemetry",
    4096,
    NULL,
    2,
    g_telemetryTaskStack,
    &g_telemetryTaskBuffer
  );
}

#ifndef FLIGHT_BUILD
void nodePrintNetworkStatus(Print& out) {
  out.println("\n=== Wi-Fi & QLCP Gateway Status ===");
  out.print("SSID: "); out.println(WIFI_SSID);
  out.print("Local IP: "); out.println(WiFi.localIP().toString().c_str());
  out.print("RSSI: "); out.print(WiFi.RSSI()); out.println(" dBm");
  out.print("QLCP State: "); out.println((xEventGroupGetBits(g_netCtx.wifi_event_group_handle) & SERVER_CONNECTED_BIT) ? "CONNECTED" : "DISCONNECTED");
  out.print("Server IP: "); out.println(g_netCtx.server_ip);
  out.print("Server TCP/UDP: "); out.print(g_netCtx.server_tcp_port); out.print(" / "); out.println(g_netCtx.server_udp_port);
  out.println("QLCP Traffic Stats:");
  out.print("  TCP Frame Rx: "); out.println(g_qlcpStats.tcpRxCount);
  out.print("  TCP Frame Tx: "); out.println(g_qlcpStats.tcpTxCount);
  out.print("  UDP Frame Tx (Data): "); out.println(g_qlcpStats.udpTxCount);
  out.print("  CONFIG Frame Tx: "); out.println(g_qlcpStats.configSent);
  out.print("  TIMESYNC Syncs: "); out.println(g_qlcpStats.timesyncCount);
  out.print("  HEARTBEAT Rx: "); out.println(g_qlcpStats.heartbeatRx);
  out.print("  CONTROL Cmds: "); out.println(g_qlcpStats.controlRx);
  out.print("  STATUS REQs: "); out.println(g_qlcpStats.statusRequestRx);
  out.print("  STREAM STRT/STP Rx: "); out.print(g_qlcpStats.streamStartRx); out.print(" / "); out.println(g_qlcpStats.streamStopRx);
}

void nodePrintCanStatus(Print& out) {
  out.println("\n=== AimNetwork CAN Node Status ===");
  out.print("Primary Dest (Lower Node): 0x"); out.println(AIM_DEST_LPROP, HEX);
  out.println("Lower Telemetry:");
  out.print("  LOWER_PT1: "); out.print(g_ptValues[2]); out.println(" PSI");
  out.print("  LOWER_PT2: "); out.print(g_ptValues[3]); out.println(" PSI");
  out.println("Lower Valves:");
  out.print("  LOWER_VALVE1: "); out.println(g_valveStates[2] ? "OPEN" : "CLOSED");
  out.print("  LOWER_VALVE2: "); out.println(g_valveStates[3] ? "OPEN" : "CLOSED");
  out.println("AimNetwork Frame Stats:");
  out.print("  CAN Rx Count: "); out.println(g_canStats.rxCount);
  out.print("  CAN Tx Count: "); out.println(g_canStats.txCount);
  out.print("  SENSOR frame Rx: "); out.println(g_canStats.sensorRx);
  out.print("  VALVE frame Rx/Tx: "); out.print(g_canStats.valveRx); out.print("/"); out.println(g_canStats.valveTx);
}

void nodePrintSensorStatus(Print& out) {
  out.println("\n=== Local Sensor & Valve Status ===");
  out.println("Local Telemetry:");
  out.print("  UPPER_PT1: "); out.print(g_ptValues[0]); out.println(" PSI");
  out.print("  UPPER_PT2: "); out.print(g_ptValues[1]); out.println(" PSI");
  out.println("Local Valves:");
  out.print("  UPPER_VALVE1 (pin "); out.print(SOL1_EN_PIN); out.print("): "); out.println(g_valveStates[0] ? "OPEN" : "CLOSED");
  out.print("  UPPER_VALVE2 (pin "); out.print(SOL2_EN_PIN); out.print("): "); out.println(g_valveStates[1] ? "OPEN" : "CLOSED");
}

bool nodeSetValveStateDirect(uint8_t index, bool open) {
  if (index >= 4) {
    return false;
  }

  LOG_INFO("Direct console actuator override index=%u open=%d", index, open);

  if (index < 2) {
    uint8_t endpoint = (index == 0) ? NODE_ENDPOINT_VALVE1 : NODE_ENDPOINT_VALVE2;
    return s_setLocalValveState(endpoint, open);
  } else {
#ifdef MOCK_HARDWARE
    g_valveStates[index] = open;
    return true;
#else
    uint8_t endpoint = (index == 2) ? NODE_ENDPOINT_LOWER_BASE : (NODE_ENDPOINT_LOWER_BASE + 1);
    if (s_aimPtr != nullptr) {
      const uint32_t payload = open ? NODE_ACTUATOR_OPEN : NODE_ACTUATOR_CLOSED;
      const bool ok = s_aimPtr->sendTimedPktEx(endpoint, s_aimPtr->syncedMillis(), payload, AIM_DEST_LPROP, AIM_TYP_VALVE);
#ifndef FLIGHT_BUILD
      if (ok) {
        g_canStats.txCount++;
        g_canStats.valveTx++;
      }
#endif
      return ok;
    }
    return false;
#endif
  }
}

bool nodeGetValveState(uint8_t index) {
  if (index < 4) {
    return g_valveStates[index];
  }
  return false;
}
#endif
