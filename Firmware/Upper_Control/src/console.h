#ifndef CONSOLE_H
#define CONSOLE_H

#ifndef FLIGHT_BUILD

#include <Arduino.h>
#include <cstdint>

class AimNetwork;
class AimCanDriver;
class Logger;
class AimFileSystem;
class AimFlightRecorder;
class AimConfigStore;
struct BoardConfig;

enum ConsoleAction : uint8_t {
  CONSOLE_ACTION_NONE        = 0U,
  CONSOLE_ACTION_ENTER       = 1U,
  CONSOLE_ACTION_EXIT        = 2U
  // add more console actions here as needed
};

bool consoleInit(Stream& serial,
                 AimNetwork& aim,
                 AimCanDriver& canDriver,
                 Logger& log,
                 AimFileSystem& fs,
                 AimFlightRecorder& recorder,
                 AimConfigStore& configStore,
                 BoardConfig& boardConfig);

ConsoleAction consoleCheckEntry(void);
ConsoleAction consoleService(uint8_t currentState, uint32_t networkNowMs);
void consoleResume(void);

#endif // FLIGHT_BUILD

#endif  // CONSOLE_H
