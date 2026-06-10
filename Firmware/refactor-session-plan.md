# Implementation Plan: Phased Simplification (Upper_Control + av-libraries)

## Background & Motivation
The original version of this plan targeted a broad de-abstraction pass across the firmware.
Part of that work is already done: `node.cpp` is gone, and `Upper_Control/src/main.cpp` is now
the target architecture — a flat `switch` state machine with bounded CAN RX (8 frames/loop),
a 2 s task watchdog, and board-specific behavior isolated in `board.cpp`.

Two problem areas remain, in priority order:

1. **Config/storage ownership is smeared across modules.** `console.cpp` hard-codes
   `/config.json`, builds config JSON itself, removes/streams the file directly, and calls
   `s_fs->format()` while `AimFlightRecorder` still holds open LittleFS handles (the recorder
   writes a row every 100 ms from the main loop). This is the root of the operator-triggerable
   `assert failed: lfs_file_size` crash (`FLS -> ers -> confirm`, then any config/dump op).
   `main.cpp` independently mutates the same `/config.json` at boot to inject the telemetry
   schema, so the file has two owners and two meanings.
2. **Hidden concurrency in the WiFi/QLCP path.** `wifi/network_manager.c` spawns four FreeRTOS
   tasks with `portMAX_DELAY` waits (`tcp_tools.c`, `udp_tools.c`), and `board.cpp` adds
   `qlcpManagerTask`/`qlcpTelemetryTask`, sharing state with the main loop via event groups and
   an atomic `g_tsOffset`. This is where debugging is hardest.

## Scope & Constraints
- **In scope**: `Firmware/Upper_Control/` and `Firmware/av-libraries/` only.
- **Lower_Control (STM32F103) is a compatibility constraint, not a target.** Any `av-libraries`
  change must keep building for `ststm32` (the `stm32_canbus` example is the canary) and must
  not break the `SerialFlashDriver` path. `av-libraries` is a shared submodule on
  `stable-v0.4.x`; API-breaking changes there require a version bump and a coordinated
  submodule-pin update, never a side effect of an Upper_Control cleanup.
- **Explicitly dropped from the original plan** (recorded so we don't relitigate):
  - *Removing the `AimBlockDevice` virtual interface.* Two bounded driver implementations
    across two platforms is exactly the polymorphism our style allows; storage is not a hot
    path, and `#ifdef` forks would make the library harder to build for both targets.
  - *Merging `AimCanDriver` into `AimNetwork`.* It is two layers, not a wrapper stack, and the
    CAN design is already intentionally narrow (mailbox TX, bounded RX polling, static queues).
  - *"Eliminating FreeRTOS."* The ESP32 WiFi/lwIP stack runs in its own tasks regardless. The
    goal is eliminating **our** tasks and the synchronization between them.

## Phase 1: Config & Storage Ownership (Upper_Control, small av-libraries additions)

Goal: one module owns "what config means," one function owns "how to safely format," and
`console.cpp` becomes a view/controller that only dispatches and prints results.

1. **Introduce a board-config owner** (`board_config.cpp/.h` or similar):
   - Owns the path, JSON keys, defaults, and merge behavior. `console.cpp` and `main.cpp` never
     see the string `"/config.json"` again.
   - API shape (status-returning, no hidden state):
     `configLoadBoard(BoardConfig&)`, `configSaveBoard(const BoardConfig&)`,
     `configResetBoard()`, `configPrintBoard(Stream&)`.
   - Move the boot-time telemetry-schema injection out of `main.cpp` into this owner. Decide
     here whether to split `/board.json` (operator identity) from `/telemetry.json` (generated
     schema), or keep one merged `/config.json` built by the owner. Either is fine; the console
     building it is not.
   - Load/print results distinguish three states instead of one boolean: **not present (normal,
     compiled defaults)**, **read failed**, **parse failed**.
2. **Introduce a storage-maintenance function** that owns format safety:
   - `storageFormatForMaintenance()` stops/flushes/closes the flight recorder and config store
     before calling `format()`, then leaves storage in a "reset pending reboot" state.
   - May require a small `AimFlashStorage` addition (e.g., `AimFlightRecorder::close()`/flush
     hook). That is an additive, platform-neutral library change — minor version bump.
3. **Console becomes restricted after erase**: after a successful format, print
   `[OK] flash erased — reboot required before dump/config operations` and disable `dmp`,
   `cfg`, `inf` until reboot (allow `b`/`q`/`sts`). This deletes post-format edge-case handling
   rather than adding to it.
4. **Make displayed state match affected state**: after `rst`, show RAM vs flash explicitly
   (`ram: name=... can=...` / `flash: not present — defaults on next boot`), or mark config
   pending-reboot and stop offering edits. Only print the "reboot required" warning on save
   when `canId` actually changed.
5. **Deletion targets in `console.cpp`** (the measure of success is net code removal):
   `kConfigPath`, all `JsonDocument` construction, direct `removeFile`/`streamFile`/`format()`
   calls, all knowledge of telemetry fields, and post-format config/dump support.

## Phase 2: De-Risking Concurrency (WiFi/QLCP task flattening)

1. **Refactor `wifi/network_manager.c`**: remove the four FreeRTOS tasks and event groups.
   Replace with a non-blocking `network_service()` state machine polled from the main loop
   (`begin`/`service`/`cancel`).
2. **Convert `qlcpManagerTask` / `qlcpTelemetryTask`** (`board.cpp`) into non-blocking
   `service()` functions called from `boardUpdate()`. WiFi connection wait becomes a state, not
   a `while (WiFi.status() != WL_CONNECTED)` spin. The cross-task queues, event groups, and the
   atomic `g_tsOffset` disappear — timesync offset becomes plain single-threaded state.
3. **Watchdog budget is a hard requirement**: the main loop runs under a 2 s task WDT. All
   socket work moved into the loop must be genuinely non-blocking — non-blocking sockets or
   zero-timeout `select()`, bounded bytes per tick. Document the worst-case tick duration for
   `network_service()` and the QLCP services; a default `connect()` timeout alone would trip
   the WDT.

## Phase 3 (optional, descoped): av-libraries thin-HAL items

Only if still justified after Phases 1–2, and only as a coordinated `av-libraries` change with
a version bump tested against both `examples/esp32_canbus` and `examples/stm32_canbus`:

1. Replace the STM32 CAN bit-timing search in `aim_stm32_can_core.cpp` with fixed, explicit
   timing parameters (legitimate thin-HAL; benefits Lower_Control directly).
2. Make the node health monitor an explicitly-called `service()` with no hidden timing, rather
   than moving it into every application (which would duplicate it per node).

## Verification & Testing
There are **no unit tests** in this repo and `Upper_Control_test` does not build — do not cite
them. Real verification is:

- **Build gate**: `.claude/skills/run-av-prop/smoke.sh` — both targets must compile clean under
  `-Wall -Wextra -Werror` after every step. For any `av-libraries` change, additionally build
  `AimNetwork/examples/stm32_canbus` to prove STM32 compatibility.
- **Console regression walkthrough (Phase 1)**: re-run the exact operator sequence that crashes
  today (`FLS -> dmp -> cancel`, `FLS -> ers -> confirm`, `FLS -> cfg -> jsn`, `FLS -> cfg ->
  sav`) and confirm the restricted post-erase mode instead of the `lfs_file_size` assert.
  Verify the three distinct config states (not present / read failed / parse failed) and the
  RAM-vs-flash display after `rst`.
- **HIL (Phase 2)**: CAN traffic, WiFi telemetry streaming, and QLCP timesync/heartbeat/stream
  start-stop on real hardware with `MOCK_HARDWARE` removed. Confirm no watchdog trips during
  WiFi disconnect/reconnect and server-unreachable scenarios (these previously blocked in
  tasks; now they must not block the loop).
- **Observability**: every new state machine logs its transitions via `LOG_*`; storage
  maintenance and config operations report status, never `void`.

## Migration & Rollback
- One feature branch per phase: `fw/config-ownership`, `fw/wifi-task-flattening`,
  `fw/av-lib-thin-hal` (if Phase 3 happens).
- `av-libraries` changes land as PRs in that repo with a version bump (`library.json` and
  `AIM_NETWORK_VERSION_STRING` kept in sync), then the submodule pin is updated here.
- No phase merges until its verification section passes. If a phase fails, abandon the branch;
  the previous pin/commit remains the known-good state.
