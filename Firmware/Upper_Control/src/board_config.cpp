#include "board_config.h"

#include <cstring>
#include <ArduinoJson.h>
#include <logger.h>
#include <AimFileSystem.h>
#include <AimConfigStore.h>
#include <AimFlightRecorder.h>

static constexpr char kConfigPath[] = "/config.json";

static AimFileSystem* s_fs = nullptr;
static AimConfigStore* s_store = nullptr;
static AimFlightRecorder* s_recorder = nullptr;

static ConfigStatus fromLoad(AimConfigLoad result) {
  switch (result) {
    case AimConfigLoad::OK:                return ConfigStatus::OK;
    case AimConfigLoad::NOT_PRESENT:       return ConfigStatus::NOT_PRESENT;
    case AimConfigLoad::READ_FAILED:       return ConfigStatus::READ_FAILED;
    case AimConfigLoad::PARSE_FAILED:      return ConfigStatus::PARSE_FAILED;
    case AimConfigLoad::STORAGE_NOT_READY: return ConfigStatus::STORAGE_DOWN;
    default:                               return ConfigStatus::READ_FAILED;
  }
}

static void writeTelemetrySchema(JsonDocument& doc) {
  JsonObject telemetry = doc["telemetry"].to<JsonObject>();
  telemetry["cols"] = BOARD_LOG_COL_COUNT;
  JsonArray headers = telemetry["headers"].to<JsonArray>();
  for (uint8_t i = 0U; i < BOARD_LOG_COL_COUNT; i++) {
    headers.add(kBoardTelemetryHeaders[i]);
  }
}

bool configInit(AimFileSystem& fs, AimConfigStore& store, AimFlightRecorder& recorder) {
  s_fs = &fs;
  s_store = &store;
  s_recorder = &recorder;
  return true;
}

ConfigStatus configLoadBoard(BoardConfig& out) {
  if (s_store == nullptr) {
    return ConfigStatus::STORAGE_DOWN;
  }

  JsonDocument doc;
  const ConfigStatus status = fromLoad(s_store->loadDetailed(kConfigPath, doc));
  if (status != ConfigStatus::OK) {
    return status;
  }

  if (doc["boardName"].is<const char*>()) {
    strlcpy(out.boardName, doc["boardName"], sizeof(out.boardName));
  }
  if (doc["canId"].is<uint8_t>()) {
    out.canId = doc["canId"];
  }
  return ConfigStatus::OK;
}

ConfigStatus configSaveBoard(const BoardConfig& cfg) {
  if (s_store == nullptr) {
    return ConfigStatus::STORAGE_DOWN;
  }

  JsonDocument doc;
  const ConfigStatus loadStatus = fromLoad(s_store->loadDetailed(kConfigPath, doc));
  if (loadStatus == ConfigStatus::STORAGE_DOWN) {
    return loadStatus;
  }
  if (loadStatus != ConfigStatus::OK) {
    // Unreadable or absent: start fresh, but keep the telemetry contract.
    doc.clear();
    writeTelemetrySchema(doc);
  }

  doc["boardName"] = cfg.boardName;
  doc["canId"] = cfg.canId;

  return s_store->save(kConfigPath, doc) ? ConfigStatus::OK : ConfigStatus::WRITE_FAILED;
}

ConfigStatus configResetBoard(void) {
  if (s_store == nullptr || s_fs == nullptr) {
    return ConfigStatus::STORAGE_DOWN;
  }

  JsonDocument doc;
  const ConfigStatus loadStatus = fromLoad(s_store->loadDetailed(kConfigPath, doc));
  if (loadStatus == ConfigStatus::NOT_PRESENT) {
    return ConfigStatus::OK;  // nothing stored — defaults already apply
  }
  if (loadStatus == ConfigStatus::STORAGE_DOWN) {
    return loadStatus;
  }
  if (loadStatus != ConfigStatus::OK) {
    // Unreadable file: deleting it restores defaults, telemetry is lost but
    // configEnsureTelemetrySchema() recreates it on next boot.
    return s_fs->removeFile(kConfigPath) ? ConfigStatus::OK : ConfigStatus::WRITE_FAILED;
  }

  doc.remove("boardName");
  doc.remove("canId");
  return s_store->save(kConfigPath, doc) ? ConfigStatus::OK : ConfigStatus::WRITE_FAILED;
}

ConfigStatus configEnsureTelemetrySchema(void) {
  if (s_store == nullptr) {
    return ConfigStatus::STORAGE_DOWN;
  }

  JsonDocument doc;
  const ConfigStatus loadStatus = fromLoad(s_store->loadDetailed(kConfigPath, doc));
  if (loadStatus == ConfigStatus::READ_FAILED ||
      loadStatus == ConfigStatus::PARSE_FAILED ||
      loadStatus == ConfigStatus::STORAGE_DOWN) {
    return loadStatus;  // don't clobber a corrupt file — operator resets it
  }

  if (loadStatus == ConfigStatus::OK && doc["telemetry"].is<JsonObject>()) {
    return ConfigStatus::OK;  // already present
  }

  writeTelemetrySchema(doc);
  return s_store->save(kConfigPath, doc) ? ConfigStatus::OK : ConfigStatus::WRITE_FAILED;
}

ConfigStatus configPrintBoard(Stream& out) {
  if (s_store == nullptr) {
    out.println("[CFG]");
    out.println("{}");
    out.println("[/CFG]");
    return ConfigStatus::STORAGE_DOWN;
  }

  JsonDocument doc;
  const ConfigStatus status = fromLoad(s_store->loadDetailed(kConfigPath, doc));

  out.println("[CFG]");
  if (status == ConfigStatus::OK) {
    serializeJson(doc, out);
    out.println();
  } else {
    out.println("{}");
  }
  out.println("[/CFG]");
  return status;
}

ConfigStatus storageFormatForMaintenance(void) {
  if (s_fs == nullptr || s_recorder == nullptr) {
    return ConfigStatus::STORAGE_DOWN;
  }

  if (s_recorder->isDumping()) {
    s_recorder->stopDump();
  }
  if (!s_recorder->closeLog()) {
    LOG_WARN("Log close failed before format");
  }

  return s_fs->format() ? ConfigStatus::OK : ConfigStatus::WRITE_FAILED;
}
