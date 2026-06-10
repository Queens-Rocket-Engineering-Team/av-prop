#include "console.h"

#ifndef FLIGHT_BUILD

#include "board.h"
#include <logger.h>
#include <aim_can_driver.h>
#include <AimFileSystem.h>
#include <AimFlightRecorder.h>
#include <AimConfigStore.h>

#include <cstdlib>
#include <cstring>

static constexpr char     kConsoleEntryKey       = 'd';
static constexpr char     kConsoleCtrlAEntryChar = '\x01'; // Ctrl-A, guarded by double press.
static constexpr uint32_t kCtrlAEntryGuardMs     = 750U;
static constexpr uint8_t  kInputBufLen           = 32U;
static constexpr uint8_t  kMenuStackDepthMax     = 4U;
static constexpr uint8_t  kDumpRowsPerTick       = 32U;
static constexpr char     kConfigPath[]          = "/config.json";

enum ConsoleMenu : uint8_t {
  CONSOLE_MENU_ROOT                = 0U,
  CONSOLE_MENU_LOG_MASK            = 1U,
  CONSOLE_MENU_FLASH               = 2U,
  CONSOLE_MENU_FLASH_ERASE_CONFIRM = 3U,
  CONSOLE_MENU_SENSOR_CONTROL      = 4U,
  CONSOLE_MENU_FLASH_DUMPING       = 5U,
  CONSOLE_MENU_CONFIG              = 6U,
  CONSOLE_MENU_CONFIG_NAME         = 7U,
  CONSOLE_MENU_CONFIG_CAN          = 8U
};

static Stream*            s_serial      = nullptr;
static AimNetwork*        s_aim         = nullptr;
static AimCanDriver*      s_canDriver   = nullptr;
static Logger*            s_log         = nullptr;
static AimFileSystem*     s_fs          = nullptr;
static AimFlightRecorder* s_recorder    = nullptr;
static AimConfigStore*    s_configStore = nullptr;
static BoardConfig*       s_boardConfig = nullptr;

static ConsoleMenu s_menu = CONSOLE_MENU_ROOT;
static ConsoleMenu s_menuStack[kMenuStackDepthMax];
static uint8_t     s_menuDepth = 0U;

static BoardConfig s_savedBoardConfig = {};
static bool        s_configDiscardArmed = false;
static uint32_t    s_lastCtrlAEntryMs = 0U;

static char    s_inputBuf[kInputBufLen];
static uint8_t s_inputLen = 0U;

static bool consoleReady(void) {
  return (s_serial != nullptr) && (s_aim != nullptr) && (s_canDriver != nullptr) &&
         (s_log != nullptr) && (s_fs != nullptr) && (s_recorder != nullptr) &&
         (s_configStore != nullptr) && (s_boardConfig != nullptr);
}

static void copyBoardConfig(BoardConfig& dst, const BoardConfig& src) {
  strlcpy(dst.boardName, src.boardName, sizeof(dst.boardName));
  dst.canId = src.canId;
}

static bool configIsDirty(void) {
  return (std::strcmp(s_boardConfig->boardName, s_savedBoardConfig.boardName) != 0) ||
         (s_boardConfig->canId != s_savedBoardConfig.canId);
}

static void resetInput(void) {
  s_inputLen = 0U;
  std::memset(s_inputBuf, 0, sizeof(s_inputBuf));
}

static void resetMenu(void) {
  s_menu = CONSOLE_MENU_ROOT;
  s_menuDepth = 0U;
  s_configDiscardArmed = false;
  s_lastCtrlAEntryMs = 0U;
  resetInput();
}

static void pushMenu(ConsoleMenu nextMenu) {
  if (s_menuDepth >= kMenuStackDepthMax) {
    s_serial->println("[ERR] menu stack full");
    resetMenu();
    return;
  }
  s_menuStack[s_menuDepth++] = s_menu;
  s_menu = nextMenu;
}

static void popMenu(void) {
  s_menu = (s_menuDepth > 0U) ? s_menuStack[--s_menuDepth] : CONSOLE_MENU_ROOT;
  s_configDiscardArmed = false;
}

static void beginInput(ConsoleMenu nextMenu) {
  resetInput();
  pushMenu(nextMenu);
}

static int readChar(void) {
  AIM_ASSERT(s_serial != nullptr);
  if (s_serial->available() <= 0) { return -1; }
  const int rxByte = s_serial->read();
  if (rxByte < 0) { return -1; }
  return static_cast<int>(static_cast<char>(rxByte));
}

static void printCanStatus(void) {
  AimEsp32CanCore::Stats stats = {};
  s_canDriver->getEsp32Stats(stats);
  s_serial->printf("txFrames=%u rxFrames=%u txErrors=%u rxErrors=%u\n",
                   stats.txFrames, stats.rxFrames, stats.txErrors, stats.rxErrors);
  s_serial->printf("filteredFrames=%u beginErrors=%u lastError=0x%X\n",
                   stats.filteredFrames, stats.beginErrors, stats.lastError);
}

static void printStatus(uint8_t currentState, uint32_t networkNowMs) {
  s_serial->printf("name=%s state=%u logMask=0x%X nowMs=%lu\n",
                   s_boardConfig->boardName,
                   static_cast<unsigned>(currentState),
                   static_cast<unsigned>(s_log->filterMask()),
                   static_cast<unsigned long>(networkNowMs));
  s_serial->printf("version=%s build=%s %s\n", AIM_NETWORK_VERSION_STRING, __DATE__, __TIME__);
}

static void setLogMask(uint8_t mask, const char* label) {
  s_log->setFilterMask(mask);
  s_serial->printf("[OK] logMask=0x%X (%s)\n", static_cast<unsigned>(s_log->filterMask()), label);
}

static bool parseCanId(uint8_t& outCanId) {
  s_inputBuf[s_inputLen] = '\0';
  char* end = nullptr;
  const long parsed = std::strtol(s_inputBuf, &end, 10);
  if ((end == s_inputBuf) || (*end != '\0') || (parsed < 1L) || (parsed > AIM_ORG_ADDR_MAX)) {
    return false;
  }
  outCanId = static_cast<uint8_t>(parsed);
  return true;
}

static void saveConfig(void) {
  JsonDocument doc;
  if (!s_configStore->load(kConfigPath, doc)) {
    doc.clear();
  }
  doc["boardName"] = s_boardConfig->boardName;
  doc["canId"]     = s_boardConfig->canId;

  if (s_configStore->save(kConfigPath, doc)) {
    copyBoardConfig(s_savedBoardConfig, *s_boardConfig);
    s_configDiscardArmed = false;
    s_serial->println("[OK] config saved — reboot required for CAN ID");
  } else {
    s_serial->println("[ERR] config save failed");
  }
}

static void showMenu(ConsoleMenu menu) {
  // Display-only: state transitions belong in consoleService helpers.
  switch (menu) {
    case CONSOLE_MENU_ROOT:
      s_serial->println("DBG [q:exit b:back] 1:sts 2:log 3:fls 4:net 5:can 6:ctl");
      break;
    case CONSOLE_MENU_LOG_MASK:
      s_serial->println("DBG > LOG [q:exit b:back] 0:off 1:dbg 2:inf 3:wrn 4:err 5:all 6:iwe");
      break;
    case CONSOLE_MENU_FLASH:
      s_serial->printf("DBG > FLS [q:exit b:back] 1:inf 2:dmp 3:ers 4:cfg  [%uB/%uB]\n",
                       s_fs->getUsedSize(), s_fs->getTotalSize());
      break;
    case CONSOLE_MENU_FLASH_ERASE_CONFIRM:
      s_serial->println("DBG > FLS > ERS [q:exit b:back] 1:confirm");
      break;
    case CONSOLE_MENU_CONFIG:
      s_serial->println("DBG > FLS > CFG [q:exit b:back] 1:nam 2:can 3:sav 4:rst 5:jsn");
      s_serial->printf("  name=%s can=%u\n", s_boardConfig->boardName, s_boardConfig->canId);
      break;
    case CONSOLE_MENU_CONFIG_NAME:
      s_serial->print("new name [b:cancel when empty, ESC/Ctrl-C:cancel]: ");
      break;
    case CONSOLE_MENU_CONFIG_CAN:
      s_serial->print("new can [b/ESC/Ctrl-C:cancel]: ");
      break;
    case CONSOLE_MENU_SENSOR_CONTROL: {
      const char v0 = boardGetValveState(0) ? 'O' : 'C';
      const char v1 = boardGetValveState(1) ? 'O' : 'C';
      const char v2 = boardGetValveState(2) ? 'O' : 'C';
      const char v3 = boardGetValveState(3) ? 'O' : 'C';
      s_serial->printf("DBG > CTL [q:exit b:back] 1:rfr 2:V1[%c] 3:V2[%c] 4:V3[%c] 5:V4[%c]\n",
                       v0, v1, v2, v3);
      boardPrintSensorStatus(*s_serial);
      break;
    }
    default:
      s_serial->println("DBG [q:exit b:back] 1:sts 2:log 3:fls 4:net 5:can 6:ctl");
      break;
  }
}

static bool leaveConfigMenu(void) {
  if (!configIsDirty()) {
    popMenu();
  } else if (!s_configDiscardArmed) {
    s_configDiscardArmed = true;
    s_serial->println("[?] unsaved changes — press 3:sav, or b again to discard");
  } else {
    copyBoardConfig(*s_boardConfig, s_savedBoardConfig);
    s_serial->println("[--] config changes discarded");
    popMenu();
  }
  return true;
}

static void appendInputChar(int c) {
  if (s_inputLen < (sizeof(s_inputBuf) - 1U)) {
    s_inputBuf[s_inputLen++] = static_cast<char>(c);
    s_serial->print(static_cast<char>(c));
  }
}

static void cancelInput(void) {
  s_serial->println();
  resetInput();
  popMenu();
  showMenu(s_menu);
}

static void finishInput(void) {
  if (s_menu == CONSOLE_MENU_CONFIG_NAME) {
    s_inputBuf[s_inputLen] = '\0';
    if (s_inputLen > 0U) {
      strlcpy(s_boardConfig->boardName, s_inputBuf, sizeof(s_boardConfig->boardName));
    }
  } else if (s_inputLen > 0U) {
    uint8_t canId = 0U;
    if (!parseCanId(canId)) {
      s_serial->println();
      s_serial->printf("[ERR] CAN ID must be 1-%u\n", static_cast<unsigned>(AIM_ORG_ADDR_MAX));
      resetInput();
      showMenu(s_menu);
      return;
    }
    s_boardConfig->canId = canId;
  }

  s_configDiscardArmed = false;
  s_serial->println();
  resetInput();
  popMenu();
  showMenu(s_menu);
}

static void serviceInput(int c) {
  const bool cancelKey = (c == 0x03) || (c == 0x1B) ||
                         ((s_menu == CONSOLE_MENU_CONFIG_CAN) && (c == 'b')) ||
                         ((s_menu == CONSOLE_MENU_CONFIG_NAME) && (c == 'b') && (s_inputLen == 0U));
  if (cancelKey) { cancelInput(); return; }
  if ((c == '\r') || (c == '\n')) { finishInput(); return; }
  if ((c == '\b') || (c == 0x7F)) {
    if (s_inputLen > 0U) {
      s_inputBuf[--s_inputLen] = '\0';
      s_serial->print("\b \b");
    }
    return;
  }

  if ((s_menu == CONSOLE_MENU_CONFIG_CAN) && (c >= '0') && (c <= '9')) {
    appendInputChar(c);
  } else if ((s_menu == CONSOLE_MENU_CONFIG_NAME) && (c >= 32) && (c <= 126)) {
    appendInputChar(c);
  }
}

static ConsoleAction serviceDump(void) {
  if ((s_serial->available() > 0) && (s_serial->peek() == 'b')) {
    s_serial->read();
    s_recorder->stopDump();
    s_serial->println("[--] dump canceled");
    s_menu = CONSOLE_MENU_FLASH;
    showMenu(s_menu);
    return CONSOLE_ACTION_NONE;
  }

  if (!s_recorder->serviceDump(kDumpRowsPerTick)) {
    s_serial->printf("[OK] dump complete — %uB\n", s_fs->getUsedSize());
    s_menu = CONSOLE_MENU_FLASH;
    showMenu(s_menu);
  }
  return CONSOLE_ACTION_NONE;
}

bool consoleInit(Stream& serial,
                 AimNetwork& aim,
                 AimCanDriver& canDriver,
                 Logger& log,
                 AimFileSystem& fs,
                 AimFlightRecorder& recorder,
                 AimConfigStore& configStore,
                 BoardConfig& boardConfig) {
  s_serial      = &serial;
  s_aim         = &aim;
  s_canDriver   = &canDriver;
  s_log         = &log;
  s_fs          = &fs;
  s_recorder    = &recorder;
  s_configStore = &configStore;
  s_boardConfig = &boardConfig;
  resetMenu();
  copyBoardConfig(s_savedBoardConfig, boardConfig);
  return consoleReady();
}

void consoleResume(void) {
  if (consoleReady()) { showMenu(s_menu); }
}

ConsoleAction consoleCheckEntry(void) {
  if (!consoleReady()) { return CONSOLE_ACTION_NONE; }

  const int c = readChar();
  if (c == kConsoleEntryKey) {
    resetMenu();
    showMenu(s_menu);
    return CONSOLE_ACTION_ENTER;
  }
  if (c == kConsoleCtrlAEntryChar) {
    const uint32_t nowMs = static_cast<uint32_t>(millis());
    if ((s_lastCtrlAEntryMs != 0U) && ((nowMs - s_lastCtrlAEntryMs) <= kCtrlAEntryGuardMs)) {
      s_lastCtrlAEntryMs = 0U;
      resetMenu();
      showMenu(s_menu);
      return CONSOLE_ACTION_ENTER;
    }
    s_lastCtrlAEntryMs = nowMs;
  }
  return CONSOLE_ACTION_NONE;
}

ConsoleAction consoleService(uint8_t currentState, uint32_t networkNowMs) {
  if (!consoleReady()) { return CONSOLE_ACTION_NONE; }
  if (s_menu == CONSOLE_MENU_FLASH_DUMPING) { return serviceDump(); }

  const int c = readChar();
  if (c < 0) { return CONSOLE_ACTION_NONE; }
  if (c == 'q') {
    resetMenu();
    return CONSOLE_ACTION_EXIT;
  }
  if ((s_menu == CONSOLE_MENU_CONFIG_NAME) || (s_menu == CONSOLE_MENU_CONFIG_CAN)) {
    serviceInput(c);
    return CONSOLE_ACTION_NONE;
  }

  bool handled = true;
  switch (s_menu) {
    case CONSOLE_MENU_ROOT:
      switch (c) {
        case 'b': popMenu(); break;
        case '1': printStatus(currentState, networkNowMs); break;
        case '2': pushMenu(CONSOLE_MENU_LOG_MASK); break;
        case '3': pushMenu(CONSOLE_MENU_FLASH); break;
        case '4': boardPrintNetworkStatus(*s_serial); break;
        case '5': printCanStatus(); break;
        case '6': pushMenu(CONSOLE_MENU_SENSOR_CONTROL); break;
        default: handled = false; break;
      }
      break;

    case CONSOLE_MENU_LOG_MASK:
      switch (c) {
        case 'b': popMenu(); break;
        case '0': setLogMask(0U, "off"); break;
        case '1': setLogMask(static_cast<uint8_t>(LogLevel::DEBUG), "dbg"); break;
        case '2': setLogMask(static_cast<uint8_t>(LogLevel::INFO),  "inf"); break;
        case '3': setLogMask(static_cast<uint8_t>(LogLevel::WARN),  "wrn"); break;
        case '4': setLogMask(static_cast<uint8_t>(LogLevel::ERROR), "err"); break;
        case '5': setLogMask(static_cast<uint8_t>(LogLevel::DEBUG) |
                             static_cast<uint8_t>(LogLevel::INFO)  |
                             static_cast<uint8_t>(LogLevel::WARN)  |
                             static_cast<uint8_t>(LogLevel::ERROR), "all"); break;
        case '6': setLogMask(static_cast<uint8_t>(LogLevel::INFO) |
                             static_cast<uint8_t>(LogLevel::WARN) |
                             static_cast<uint8_t>(LogLevel::ERROR), "iwe"); break;
        default: handled = false; break;
      }
      break;

    case CONSOLE_MENU_FLASH:
      switch (c) {
        case 'b': popMenu(); break;
        case '1': s_serial->printf("ready=%d total=%u used=%u\n",
                                   s_fs->isReady(), s_fs->getTotalSize(), s_fs->getUsedSize()); break;
        case '2':
          if (s_recorder->startDump(s_serial)) {
            s_menu = CONSOLE_MENU_FLASH_DUMPING;
          } else {
            s_serial->println("[ERR] dump failed to start");
          }
          break;
        case '3': pushMenu(CONSOLE_MENU_FLASH_ERASE_CONFIRM); break;
        case '4': pushMenu(CONSOLE_MENU_CONFIG); break;
        default: handled = false; break;
      }
      break;

    case CONSOLE_MENU_FLASH_ERASE_CONFIRM:
      switch (c) {
        case 'b': popMenu(); break;
        case '1':
          s_serial->println(s_fs->format() ? "[OK] flash erased" : "[ERR] erase failed");
          popMenu();
          break;
        default: handled = false; break;
      }
      break;

    case CONSOLE_MENU_CONFIG:
      if (c != 'b') { s_configDiscardArmed = false; }
      switch (c) {
        case 'b': leaveConfigMenu(); break;
        case '1': beginInput(CONSOLE_MENU_CONFIG_NAME); break;
        case '2': beginInput(CONSOLE_MENU_CONFIG_CAN); break;
        case '3': saveConfig(); break;
        case '4':
          s_fs->removeFile(kConfigPath);
          s_serial->println("[OK] config removed — reboot to apply");
          break;
        case '5':
          s_serial->println("[CFG]");
          s_serial->println(s_fs->streamFile(kConfigPath, *s_serial) ? "\n[/CFG]" : "err\n[/CFG]");
          break;
        default: handled = false; break;
      }
      break;

    case CONSOLE_MENU_SENSOR_CONTROL:
      switch (c) {
        case 'b': popMenu(); break;
        case '1': break;
        case '2': (void)boardSetValveStateDirect(0, !boardGetValveState(0)); break;
        case '3': (void)boardSetValveStateDirect(1, !boardGetValveState(1)); break;
        case '4': (void)boardSetValveStateDirect(2, !boardGetValveState(2)); break;
        case '5': (void)boardSetValveStateDirect(3, !boardGetValveState(3)); break;
        default: handled = false; break;
      }
      break;

    default:
      resetMenu();
      break;
  }

  if (handled && (s_menu != CONSOLE_MENU_FLASH_DUMPING)) {
    showMenu(s_menu);
  }
  return CONSOLE_ACTION_NONE;
}

#endif // FLIGHT_BUILD
