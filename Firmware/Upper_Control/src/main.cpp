#include "node.h"
#include "console.h"

#include <esp_idf_version.h>
#include <esp_task_wdt.h>
#include <logger.h>

static constexpr uint32_t kWatchdogTimeoutMs = 2000U;
static constexpr uint8_t kMaxRxFramesPerLoop = 8U;

struct NodeSchedulerState {
  NodeState value = INIT;
  uint32_t lastHeartbeatTxMs = 0U;
};

static AimCanDriver g_canHw(NODE_ORIGIN, NODE_CAN_BAUD, CAN_RX_PIN, CAN_TX_PIN);
static AimNetwork g_aim(&g_canHw, NODE_ORIGIN);

static NodeSchedulerState g_schedulerState = {};
static bool g_watchdogReady = false;
static Logger g_log(Serial, NODE_ORIGIN, LogLevel::INFO);

void transitionTo(NodeState nextState) {
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

    if (pkt.type == AIM_TYP_TIME) {
      g_aim.syncTime(static_cast<uint32_t>(pkt.getPayload64()));
      LOG_DEBUG("Time sync received: networkNowMs=%u", networkNowMs);
    }

    (void)nodeHandleCanPacket(pkt, networkNowMs, g_aim);
  }
}

void serviceTx(uint32_t schedulerNowMs, uint32_t networkNowMs) {
  nodeServiceLocalTelemetry(schedulerNowMs, networkNowMs, g_aim);

  if ((schedulerNowMs - g_schedulerState.lastHeartbeatTxMs) >= AIM_HEARTBEAT_TX_INTERVAL_DEFAULT_MS) {
    g_schedulerState.lastHeartbeatTxMs = schedulerNowMs;
    const uint32_t payload = static_cast<uint32_t>(g_schedulerState.value);
    if (!g_aim.sendTimedPktEx(NODE_ENDPOINT_SYSTEM, networkNowMs, payload, AIM_DEST_COMMS, AIM_TYP_HEARTBEAT)) {
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
      break;
    }

#ifndef FLIGHT_BUILD
    case DEBUG_CONSOLE: {
      const ConsoleAction act = consoleService(
          static_cast<uint8_t>(g_schedulerState.value), networkNowMs);
      if (act == CONSOLE_ACTION_EXIT) {
        transitionTo(OPERATIONAL);
      } else if (act == CONSOLE_ACTION_FLASH_INFO) {
        // g_flashTable.commandInfo(&Serial);
      } else if (act == CONSOLE_ACTION_FLASH_DUMP) {
        // if (g_flashTable.commandDump(&Serial, 512U, nullptr, nullptr)) {
        //   g_schedulerState.value = FLASH_DUMP;
        //   Serial.print("state=");
        //   Serial.println(static_cast<unsigned>(FLASH_DUMP));
        // }
      } else if (act == CONSOLE_ACTION_FLASH_ERASE) {
        // g_flashTable.commandErase(&Serial);
        // g_schedulerState.value = FLASH_ERASE;
      }
      break;
    }

    case FLASH_DUMP: {
      // if (Serial.available() > 0) {
      //   const int c = Serial.read();
      //   if (c == 'q' || c == 'Q') {
      //     g_flashTable.cancelDump();
      //     Serial.println("flash dump canceled");
      //     g_schedulerState.value = DEBUG_CONSOLE;
      //     Serial.print("state=");
      //     Serial.println(static_cast<unsigned>(DEBUG_CONSOLE));
      //     consoleResume();
      //     break;
      //   }
      // }

      // const FlashTableServiceResult r = g_flashTable.serviceDump(&Serial, 16U);
      // if (r != FLASHTABLE_SERVICE_ACTIVE) {
      //   static const char* const kDumpMsg[] = {
      //     "flash dump idle",
      //     nullptr,
      //     "flash dump done",
      //     "flash dump aborted",
      //     "flash dump error"
      //   };
      //   const uint8_t idx = static_cast<uint8_t>(r);
      //   if (idx < 5U && kDumpMsg[idx] != nullptr) {
      //     Serial.println(kDumpMsg[idx]);
      //   }
      //   g_schedulerState.value = DEBUG_CONSOLE;
      //   Serial.print("state=");
      //   Serial.println(static_cast<unsigned>(DEBUG_CONSOLE));
      //   consoleResume();
      // }
      break;
    }

    case FLASH_ERASE: {
      // const FlashTableServiceResult r = g_flashTable.serviceErase();
      // if (r != FLASHTABLE_SERVICE_ACTIVE) {
      //   Serial.println(r == FLASHTABLE_SERVICE_DONE ? "flash erase done" : "flash erase error");
      //   g_schedulerState.value = DEBUG_CONSOLE;
      //   consoleResume();
      // }
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

  nodeUpdate(schedulerNowMs);
  serviceTx(schedulerNowMs, networkNowMs);
}

void setup(void) {
  AIM_ASSERT(NODE_ORIGIN <= AIM_ORG_ADDR_MAX);
  Serial.begin(NODE_SERIAL_BAUD);
  g_logger = &g_log;
  LOG_INFO("Boot node origin=%u", static_cast<unsigned>(NODE_ORIGIN));
  initWatchdog();

  g_aim.begin();
  if (!nodeInitHardware()) {
    LOG_ERROR("Hardware init failed");
    transitionTo(FAULT);
    return;
  }
#ifndef FLIGHT_BUILD
  consoleInit(Serial, g_aim, g_log);
  Serial.println("Console ready. d=enter debug");
#endif
  g_schedulerState.lastHeartbeatTxMs = millis();
  nodeStartTasks(g_aim);
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
