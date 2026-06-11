#ifndef CONSOLE_H
#define CONSOLE_H

#ifndef FLIGHT_BUILD

#include <Arduino.h>
#include <cstdint>

class AimCanDriver;
class Logger;
class AimFileSystem;
class AimFlightRecorder;
class AimNodeConfig;
struct AimNodeCfg;

enum ConsoleAction : uint8_t {
  CONSOLE_ACTION_NONE        = 0U,
  CONSOLE_ACTION_ENTER       = 1U,
  CONSOLE_ACTION_EXIT        = 2U
  // add more console actions here as needed
};

bool consoleInit(Stream& serial,
                 AimCanDriver& canDriver,
                 Logger& log,
                 AimFileSystem& fs,
                 AimFlightRecorder& recorder,
                 AimNodeCfg& boardConfig,
                 AimNodeConfig& nodeCfg);

ConsoleAction consoleCheckEntry(void);
ConsoleAction consoleService(uint8_t currentState, uint32_t networkNowMs);

#endif // FLIGHT_BUILD

#endif  // CONSOLE_H
