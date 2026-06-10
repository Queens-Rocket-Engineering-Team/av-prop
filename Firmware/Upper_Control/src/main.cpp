#include "board.h"
#include "board_config.h"
#include "console.h"

#include <cstring>
#include <WiFi.h>
#include <esp_idf_version.h>
#include <esp_task_wdt.h>
#include <logger.h>
#include <AimFileSystem.h>
#include <AimFlightRecorder.h>
#include <AimConfigStore.h>

static constexpr uint32_t kWatchdogTimeoutMs = 2000U;
static constexpr uint8_t kMaxRxFramesPerLoop = 8U;
static constexpr uint32_t kLogIntervalMs = 100U;

struct BoardSchedulerState {
  BoardState value = INIT;
  uint32_t lastHeartbeatTxMs = 0U;
  uint32_t lastLogMs = 0U;
};

BoardConfig g_boardConfig = {BOARD_NAME, static_cast<uint8_t>(BOARD_ORIGIN)};
static ESP32PartitionDriver g_flashHw("storage");
static AimFileSystem g_fs(&g_flashHw);
static AimFlightRecorder g_flightRecorder(g_fs, BOARD_LOG_COL_COUNT, 100, 1024 * 1024);
static AimConfigStore g_configStore(g_fs);

static AimCanDriver g_canHw(BOARD_ORIGIN, BOARD_CAN_BAUD, CAN_RX_PIN, CAN_TX_PIN);
static AimNetwork g_aim(&g_canHw, BOARD_ORIGIN);

static BoardSchedulerState g_schedulerState = {};
static bool g_watchdogReady = false;
static Logger g_log(Serial, BOARD_ORIGIN, LogLevel::INFO);

void transitionTo(BoardState nextState) {
  AIM_ASSERT(nextState <= FAULT);
  AIM_ASSERT(nextState != g_schedulerState.value);
  LOG_INFO("State transition from %d to %d", static_cast<int>(g_schedulerState.value), static_cast<int>(nextState));
  g_schedulerState.value = nextState;
}

void initWatchdog(void) {
  const esp_err_t initStatus = esp_task_wdt_init(kWatchdogTimeoutMs / 1000U, true);

  const bool initOk = (initStatus == ESP_OK) || (initStatus == ESP_ERR_INVALID_STATE);
  const esp_err_t addStatus = esp_task_wdt_add(NULL);
  const bool addOk = (addStatus == ESP_OK) || (addStatus == ESP_ERR_INVALID_STATE);
  g_watchdogReady = initOk && addOk;
  if (!g_watchdogReady) {
    LOG_ERROR("Watchdog init failed (init=%d add=%d)", static_cast<int>(initStatus), static_cast<int>(addStatus));
    transitionTo(FAULT);
    return;
  }

  LOG_INFO("Watchdog ready");
}

void kickWatchdog(void) {
  if (!g_watchdogReady) {
    return;
  }

  const esp_err_t status = esp_task_wdt_reset();
  if ((status != ESP_OK) && (status != ESP_ERR_INVALID_STATE)) {
    LOG_ERROR("Watchdog reset failed (%d)", static_cast<int>(status));
    transitionTo(FAULT);
  }
}

void serviceCanRx(uint32_t networkNowMs) {
  for (uint8_t i = 0U; i < kMaxRxFramesPerLoop; i++) {
    aimPkt pkt = {};
    if (!g_aim.readPkt(pkt)) {
      break;
    }

    if (pkt.type == AIM_TYPE_TIME) {
      g_aim.syncTime(static_cast<uint32_t>(pkt.data));
      LOG_DEBUG("Time sync received: networkNowMs=%u", networkNowMs);
    }

    (void)boardHandleCanPacket(pkt, networkNowMs, g_aim);
  }
}

void serviceTx(uint32_t schedulerNowMs, uint32_t networkNowMs) {
  boardServiceLocalTelemetry(schedulerNowMs, networkNowMs, g_aim);
  aimPkt pkt = {};

  if ((schedulerNowMs - g_schedulerState.lastHeartbeatTxMs) >= AIM_HEARTBEAT_TX_INTERVAL_DEFAULT_MS) {
    g_schedulerState.lastHeartbeatTxMs = schedulerNowMs;
    const uint32_t payload = static_cast<uint32_t>(g_schedulerState.value);
    pkt.dest = AIM_DEST_COMMS;
    pkt.type = AIM_TYPE_HEARTBEAT;
    if (!pkt.packData(BOARD_ENDPOINT_SYSTEM, networkNowMs, payload) && g_aim.sendPkt(pkt)) {
      LOG_ERROR("Heartbeat TX failed");
    } else {
      LOG_DEBUG("Heartbeat TX ok");
    }
  }
}

void runStateMachine(uint32_t schedulerNowMs, uint32_t networkNowMs) {
  AIM_ASSERT(g_schedulerState.value <= FAULT);

  switch (g_schedulerState.value) {
    case OPERATIONAL: {
#ifndef FLIGHT_BUILD
      const ConsoleAction act = consoleCheckEntry();
      if (act == CONSOLE_ACTION_ENTER) {
        transitionTo(DEBUG_CONSOLE);
      }
#endif
      // Periodic logging
      if (schedulerNowMs - g_schedulerState.lastLogMs >= kLogIntervalMs) {
        g_schedulerState.lastLogMs = schedulerNowMs;
        extern float g_ptValues[];
        extern bool g_valveStates[];
        uint32_t row[BOARD_LOG_COL_COUNT];
        row[0] = networkNowMs;
        std::memcpy(&row[1], &g_ptValues[0], 4);
        std::memcpy(&row[2], &g_ptValues[1], 4);
        row[3] = g_valveStates[0];
        row[4] = g_valveStates[1];
        row[5] = g_valveStates[2];
        row[6] = g_valveStates[3];
        row[7] = AimFlightRecorder::unsignify(WiFi.RSSI());
        g_flightRecorder.writeRow(row);
      }
      break;
    }

#ifndef FLIGHT_BUILD
    case DEBUG_CONSOLE: {
      const ConsoleAction act = consoleService(
          static_cast<uint8_t>(g_schedulerState.value), networkNowMs);
      if (act == CONSOLE_ACTION_EXIT) {
        transitionTo(OPERATIONAL);
      }
      break;
    }
#endif

    case SAFE_MODE:
    case LOW_POWER:
    case FAULT:
      break;

    default:
      AIM_ASSERT(false);  // unreachable — all valid states handled above
      break;
  }

  boardUpdate(schedulerNowMs);
  serviceTx(schedulerNowMs, networkNowMs);
}

void setup(void) {
  AIM_ASSERT(BOARD_ORIGIN <= AIM_ORG_ADDR_MAX);
  Serial.begin(BOARD_SERIAL_BAUD);
  g_logger = &g_log;
  LOG_INFO("Boot board origin=%u", static_cast<unsigned>(BOARD_ORIGIN));
  initWatchdog();

  if (g_fs.begin()) {
    LOG_INFO("Storage ready.");
    configInit(g_fs, g_configStore, g_flightRecorder);

    const ConfigStatus schemaStatus = configEnsureTelemetrySchema();
    if (schemaStatus != ConfigStatus::OK && schemaStatus != ConfigStatus::NOT_PRESENT) {
      LOG_ERROR("Telemetry schema check failed (%d)", static_cast<int>(schemaStatus));
    }

    switch (configLoadBoard(g_boardConfig)) {
      case ConfigStatus::OK:
        LOG_INFO("Loaded config overlay: %s (CAN ID: %u)",
                 g_boardConfig.boardName,
                 g_boardConfig.canId);
        break;
      case ConfigStatus::NOT_PRESENT:
        LOG_INFO("No stored config — using compiled defaults");
        break;
      default:
        LOG_ERROR("Config load failed — using compiled defaults");
        break;
    }
  } else {
    LOG_ERROR("Storage init FAILED, using macro defaults.");
  }

  g_aim.begin();
  if (!boardInitHardware()) {
    LOG_ERROR("Hardware init failed");
    transitionTo(FAULT);
    return;
  }
#ifndef FLIGHT_BUILD
  if (!consoleInit(Serial, g_aim, g_canHw, g_log, g_fs, g_flightRecorder, g_boardConfig)) {
    LOG_ERROR("Console init failed");
    transitionTo(FAULT);
    return;
  }
  Serial.println("Console ready. d=enter debug");
#endif
  g_schedulerState.lastHeartbeatTxMs = millis();
  boardStartTasks(g_aim);
  transitionTo(OPERATIONAL);
}

void loop(void) {
  const uint32_t schedulerNowMs = millis();
  const uint32_t networkNowMs = g_aim.syncedMillis();
  // Main scheduler order: RX, state machine, watchdog.
  serviceCanRx(networkNowMs);
  runStateMachine(schedulerNowMs, networkNowMs);

  kickWatchdog();
}
