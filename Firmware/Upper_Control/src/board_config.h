#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <Arduino.h>
#include <cstdint>

#include "board.h"

class AimFileSystem;
class AimConfigStore;
class AimFlightRecorder;

// Sole owner of the on-flash board configuration. The config file path and
// JSON key layout live only in board_config.cpp — console and main must go
// through this API.
//
// Key ownership within the single /config.json document:
//   boardName, canId  — operator identity (AimNetwork/CAN domain), written
//                       only by configSaveBoard()/configResetBoard().
//   telemetry{cols, headers} — generated log schema (QLCP/extract tooling
//                       domain), written only by configEnsureTelemetrySchema().
// Each writer read-modify-writes through AimConfigStore's atomic save, so it
// preserves the other domain's section. The merged document must stay under
// AimConfigStore's 512-byte serialization cap (currently ~230 bytes).

enum class ConfigStatus : uint8_t {
  OK = 0,
  NOT_PRESENT,   // no config on flash — normal, compiled defaults apply
  READ_FAILED,
  PARSE_FAILED,
  WRITE_FAILED,
  STORAGE_DOWN
};

// Binds the module to the storage objects. Returns false if any is missing.
bool configInit(AimFileSystem& fs, AimConfigStore& store, AimFlightRecorder& recorder);

// Overlays stored boardName/canId onto `out` (callers pre-fill compiled
// defaults). NOT_PRESENT leaves `out` untouched and is not an error.
// NOTE: the loaded canId is informational until reboot — g_aim/g_canHw are
// constructed from the compile-time BOARD_ORIGIN before config is read.
ConfigStatus configLoadBoard(BoardConfig& out);

// Persists boardName/canId, preserving the telemetry section.
ConfigStatus configSaveBoard(const BoardConfig& cfg);

// Removes boardName/canId so compiled defaults apply on next boot; keeps the
// telemetry section. Falls back to deleting the file if it is unreadable.
ConfigStatus configResetBoard(void);

// Injects telemetry{cols, headers} from kBoardTelemetryHeaders if missing,
// creating the file on a fresh filesystem.
ConfigStatus configEnsureTelemetrySchema(void);

// Prints the stored config between "[CFG]" / "[/CFG]" markers. The content
// between the markers is always valid JSON ("{}" when absent or unreadable)
// — the extract tool parses this block over serial.
ConfigStatus configPrintBoard(Stream& out);

// Quiesces the flight recorder (stop dump, close log), then formats storage.
// After OK the caller must treat storage as reset-pending-reboot.
ConfigStatus storageFormatForMaintenance(void);

#endif  // BOARD_CONFIG_H
