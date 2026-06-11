#include "board.h"
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
  uint32_t lastLogMs = 0U;
};

BoardConfig g_boardConfig = {BOARD_NAME, static_cast<uint8_t>(BOARD_ORIGIN)};
static ESP32PartitionDriver g_flashHw("storage");
static AimFileSystem g_fs(&g_flashHw);

// Storage partition: 384 blocks × 4096 B = 1,572,864 B (partitions.csv).
// Steady state holds log.bak + log.bin, so each file may be at most
// (partition − metadata reserve) / 2. Peak usage ≈ 1.2 MB, ~376 KB headroom.
static constexpr uint32_t kStoragePartitionBytes = 384U * 4096U;
static constexpr uint32_t kLfsMetadataReserve    = 320U * 1024U;
static constexpr uint32_t kMaxLogSize = (kStoragePartitionBytes - kLfsMetadataReserve) / 2U;

static AimFlightRecorder g_flightRecorder(g_fs, BOARD_LOG_COL_COUNT, 100, kMaxLogSize);
static AimConfigStore g_configStore(g_fs);

static AimNodeConfig g_nodeCfg(g_configStore, g_fs);
static AimCanDriver g_canHw(BOARD_CAN_BAUD, CAN_RX_PIN, CAN_TX_PIN);
static AimNetwork g_aim(&g_canHw, BOARD_ORIGIN);

static BoardSchedulerState g_schedulerState = {};
static bool g_watchdogReady = false;
static Logger g_log(Serial, static_cast<uint8_t>(BOARD_ORIGIN), LogLevel::INFO);

static void transitionTo(BoardState nextState) {
  AIM_ASSERT(nextState <= FAULT);
  AIM_ASSERT(nextState != g_schedulerState.value);
  LOG_INFO("State transition from %d to %d", static_cast<int>(g_schedulerState.value), static_cast<int>(nextState));
  g_schedulerState.value = nextState;
}

static void initWatchdog(void) {
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

static void kickWatchdog(void) {
  if (!g_watchdogReady) {
    return;
  }

  const esp_err_t status = esp_task_wdt_reset();
  if ((status != ESP_OK) && (status != ESP_ERR_INVALID_STATE)) {
    LOG_ERROR("Watchdog reset failed (%d)", static_cast<int>(status));
    transitionTo(FAULT);
  }
}

static void serviceCanRx(uint32_t networkNowMs) {
  for (uint8_t i = 0U; i < kMaxRxFramesPerLoop; i++) {
    aim::Pkt pkt = {};
    if (!g_aim.readPkt(pkt)) {
      break;
    }

    if (pkt.type == aim::PacketType::Time) {
      g_aim.syncTime(static_cast<uint32_t>(pkt.data));
      LOG_DEBUG("Time sync received: networkNowMs=%u", networkNowMs);
    }

    (void)boardHandleCanPacket(pkt, networkNowMs, g_aim);
  }
}


static void runStateMachine(uint32_t schedulerNowMs, uint32_t networkNowMs) {
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
        uint32_t row[BOARD_LOG_COL_COUNT];
        row[BOARD_LOG_TIME_MS]   = networkNowMs;
        std::memcpy(&row[BOARD_LOG_PT1_PSI],   &g_ptValues[0],      4);
        std::memcpy(&row[BOARD_LOG_PT2_PSI],   &g_ptValues[1],      4);
        std::memcpy(&row[BOARD_LOG_24V_SENSE], &g_24VoltageSense[0], 4);
        std::memcpy(&row[BOARD_LOG_HALL],      &g_hallEffect[0],    4);
        row[BOARD_LOG_RSSI]    = AimFlightRecorder::unsignify(WiFi.RSSI());
        row[BOARD_LOG_VPT_FET] = g_24VoltageFet[0];
        row[BOARD_LOG_V1_FET]  = g_valveStates[0];
        row[BOARD_LOG_V2_FET]  = g_valveStates[1];
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

    case LOW_POWER:
    case FAULT:
      break;

    default:
      AIM_ASSERT(false);  // unreachable — all valid states handled above
      break;
  }

  boardUpdate(schedulerNowMs);
  boardServiceTx(schedulerNowMs, networkNowMs, g_aim,
                 static_cast<uint32_t>(g_schedulerState.value));
}

void setup(void) {
  AIM_ASSERT(static_cast<uint8_t>(BOARD_ORIGIN) <= aim::kNodeMax);
  Serial.begin(BOARD_SERIAL_BAUD);
  g_logger = &g_log;
  LOG_INFO("Boot board origin=%u", static_cast<unsigned>(BOARD_ORIGIN));
  initWatchdog();

  if (g_fs.begin()) {
    LOG_INFO("Storage ready.");
    g_flightRecorder.begin();

    const AimConfigLoad schemaStatus = g_nodeCfg.ensureSchema(BOARD_LOG_COL_COUNT, kBoardTelemetryHeaders);
    if (schemaStatus != AimConfigLoad::OK && schemaStatus != AimConfigLoad::NOT_PRESENT) {
      LOG_ERROR("Telemetry schema check failed (%d)", static_cast<int>(schemaStatus));
    }

    switch (g_nodeCfg.load(g_boardConfig)) {
      case AimConfigLoad::OK:
        LOG_INFO("Loaded config overlay: %s (CAN ID: %u)",
                 g_boardConfig.boardName,
                 g_boardConfig.canId);
        break;
      case AimConfigLoad::NOT_PRESENT:
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
  if (!consoleInit(Serial, g_canHw, g_log, g_fs, g_flightRecorder, g_boardConfig, g_nodeCfg)) {
    LOG_ERROR("Console init failed");
    transitionTo(FAULT);
    return;
  }
  Serial.println("Console ready. d=enter debug");
#endif
  boardStartNetwork();
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
