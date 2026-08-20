#include "node.h"

#include <cstring>
#include <WiFi.h>
#include <esp_idf_version.h>
#include <esp_task_wdt.h>
#include <logger.h>
#include <aim_file_system.h>
#include <aim_flight_recorder.h>

static constexpr uint32_t kWatchdogTimeoutMs = 10000U; // 10 seconds timeout
static constexpr uint8_t kMaxRxFramesPerLoop = 8U;

static ESP32PartitionDriver s_flashHw("storage");
static AimFileSystem s_fs(&s_flashHw);
static AimFlightRecorder s_flightRecorder(s_fs, kLogCols, kLogOriginRefresh, kLogMaxSize, kLogHeaders);

static AimCanHardware g_canHw(node::kCanBaud, pins::kCanRx, pins::kCanTx);
AimNetwork g_aim(&g_canHw, node::kSource);

static bool s_watchdogReady = false;
static bool s_storageReady = false;
static Logger s_log(Serial, static_cast<uint8_t>(node::kSource), LogLevel::INFO);

static void initWatchdog(void) {
  const esp_err_t initStatus = esp_task_wdt_init(kWatchdogTimeoutMs / 1000U, true);
  const bool initOk = (initStatus == ESP_OK) || (initStatus == ESP_ERR_INVALID_STATE);
  const esp_err_t addStatus = esp_task_wdt_add(NULL);
  const bool addOk = (addStatus == ESP_OK) || (addStatus == ESP_ERR_INVALID_STATE);
  s_watchdogReady = initOk && addOk;
  if (s_watchdogReady) {
    LOG_INFO("Watchdog ready (%us timeout)", static_cast<unsigned>(kWatchdogTimeoutMs / 1000U));
  }
}

static void kickWatchdog(void) {
  if (s_watchdogReady) {
    (void)esp_task_wdt_reset();
  }
}

static void serviceCanRx(void) {
  const uint32_t nowMs = millis();
  for (uint8_t i = 0U; i < kMaxRxFramesPerLoop; i++) {
    aim::Msg m = {};
    if (!g_aim.receive(m)) break;
    nodeOnRx(m, nowMs);
  }
}

#ifndef FLIGHT_BUILD
#include <aim_console.h>

static void hookStatus(Stream& out) {
  out.print("name=");
  out.print(node::kName);
  out.print(" logMask=0x");
  out.print(static_cast<unsigned>(s_log.filterMask()), HEX);
  out.print(" syncedMs=");
  out.print(static_cast<unsigned long>(g_aim.syncedMillis()));
  out.print(" version=");
  out.print(aim::kNetworkVersionString);
  out.print(" schema=");
  out.print(static_cast<unsigned>(aim::kSchemaVersion));
  out.print(" build=");
  out.print(__DATE__);
  out.print(" ");
  out.println(__TIME__);
}
#endif  // FLIGHT_BUILD

void setup(void) {
  Serial.begin(node::kSerialBaud);
  g_logger = &s_log;
  s_log.setFilterMask(static_cast<uint8_t>(LogLevel::INFO));  // INFO only (bitmask; drops DEBUG/WARN/ERROR)
  LOG_INFO("Boot board origin=%u", static_cast<unsigned>(node::kSource));

  // Class accept mask: Ack, State, Sensor, Time, Heartbeat, Event
  if (!g_aim.begin(aim::classBit(aim::Class::Ack) |
                   aim::classBit(aim::Class::State) |
                   aim::classBit(aim::Class::Sensor) |
                   aim::classBit(aim::Class::Heartbeat) |
                   aim::classBit(aim::Class::Event))) {
    LOG_ERROR("CAN init failed");
  }

  // Storage initialization on ESP32 partition
  if (!s_fs.begin()) {
    LOG_WARN("Flash filesystem mount failed — formatting partition...");
    if (!s_fs.format() || !s_fs.begin()) {
      LOG_ERROR("Flash filesystem initialization failed");
    } else {
      LOG_INFO("Flash filesystem formatted and mounted successfully");
    }
  }

  if (s_fs.isReady()) {
    if (!s_flightRecorder.begin()) {
      LOG_WARN("Flight recorder init failed");
    } else {
      s_storageReady = true;
      LOG_INFO("Flight recorder ready on ESP32 storage partition");
    }
  }

#ifndef FLIGHT_BUILD
  uint8_t nodeHookCount = 0U;
  const AimConsoleHook* nodeHooks = nodeConsoleHooks(nodeHookCount);

  static AimConsoleHook combinedHooks[8];
  uint8_t totalHooks = 0;
  combinedHooks[totalHooks++] = {'s', "status", hookStatus};
  for (uint8_t i = 0; i < nodeHookCount && totalHooks < 8; i++) {
    combinedHooks[totalHooks++] = nodeHooks[i];
  }
  aimConsoleInit(Serial, s_fs, s_flightRecorder, node::kName, combinedHooks, totalHooks);
#endif

  nodeInit();
  initWatchdog();

#ifndef FLIGHT_BUILD
  Serial.println("Console ready. d=enter debug");
#endif
}

void loop(void) {
  // Main scheduler order: RX, updates, CAN, heartbeats, console, watchdog.
  serviceCanRx();
  const uint32_t nowMs = millis();
  nodeUpdate(nowMs);
  if (!aimConsoleIsActive()) {
    nodeServiceLog(nowMs, s_flightRecorder);
  }
  nodeServiceCanTx(nowMs, g_aim);
  g_aim.service(nowMs, nodeCurrentState(), nodeErrorBits());

#ifndef FLIGHT_BUILD
  aimConsoleService();
#endif

  kickWatchdog();
  delay(1U);
}
