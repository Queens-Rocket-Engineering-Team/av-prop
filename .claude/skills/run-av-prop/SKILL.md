---
name: run-av-prop
description: Build, compile, verify, and smoke-test av-prop firmware. Use when asked to build, compile, run, check, or verify the Upper Control or Lower Control firmware targets.
---

# av-prop firmware build skill

This repo contains two PlatformIO firmware projects cross-compiled for hardware targets
(no native simulation). The driver is `smoke.sh` — it builds both targets and reports
firmware sizes. Flash and serial-monitor require physical hardware connected over USB/ST-Link.

All paths are relative to the repo root (`/home/tristan/qret/av-prop` or wherever you cloned it).

## Prerequisites

PlatformIO is installed in a project-local venv — no system install needed:

```bash
source .venv/bin/activate
pio --version   # PlatformIO Core, version 6.1.19
```

Submodules must be initialized (already done if `Firmware/ctl-qlcp-lib/` is non-empty):

```bash
git submodule update --init --recursive
```

## Build (agent path — smoke driver)

```bash
bash .claude/skills/run-av-prop/smoke.sh
```

This builds both targets in sequence and exits 0 on success, 1 on any failure.
Output ends with a summary table:

```
==============================
Build summary:
  PASS  Upper_Control (ESP32-S3)
  PASS  Lower_Control_test (STM32F103)
==============================
RESULT: All 2 target(s) passed
```

## Build individual targets

From repo root, activate the venv once, then `cd` into the project and run `pio run`:

```bash
source .venv/bin/activate

# ESP32-S3 upper control board (main flight firmware)
cd Firmware/Upper_Control && pio run -e Upper_Control

# STM32F103 lower control board
cd Firmware/Lower_Control_test && pio run -e lower_test
```

Incremental rebuilds are fast (~5 s for Upper_Control, ~12 s for Lower_Control_test on
a warm cache).

## Flash to hardware (human path — requires physical board)

```bash
source .venv/bin/activate
# Upper Control (USB, esptool):
cd Firmware/Upper_Control && pio run -e Upper_Control -t upload
# Lower Control (ST-Link):
cd Firmware/Lower_Control_test && pio run -e lower_test -t upload
# Serial monitor after flashing:
pio device monitor
```

## Gotchas

- **`MOCK_HARDWARE` is on by default** in `Upper_Control/platformio.ini`. With it set,
  the ADC read path is bypassed and synthetic sine/cosine PT values are generated instead.
  Remove or comment out `-DMOCK_HARDWARE` in `platformio.ini` before a real hardware flash.
- **`Upper_Control_test` does not build** — it references `<FastLED.h>` but FastLED is absent
  from its `lib_deps`. Skip it; the main `Upper_Control` environment is the flight target.
- **pio must run from within the project directory**, not from repo root — each
  `Firmware/<project>/` has its own `platformio.ini`. The smoke script `cd`s for you.
- **CMake is not installed** in this environment; the standalone `ctl-qlcp-lib` CMake build
  (`cmake -S . -B build/`) will fail. The library source is included directly in
  `Upper_Control` via `build_src_filter`.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `ERROR: venv not found` | Run `pip install platformio` or restore `.venv/` |
| `pio: command not found` | `source .venv/bin/activate` first |
| `Submodule ... not initialized` | `git submodule update --init --recursive` |
| `FastLED.h not found` in Upper_Control_test | Known issue — use `Upper_Control` env instead |
