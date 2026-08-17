#include "node.h"
#include <logger.h>
#include <ADS131M04.h>
#include <Adafruit_NeoPixel.h>
#include <aim_job.h>
#include <aim_control.h>
#include <aim_sensor.h>
#include <aim_flight_recorder.h>
#include <prop_testing.h>

static ADS131M04 s_adc(pins::kAdcCs, pins::kAdcDrdy, &SPI);

static Adafruit_NeoPixel s_rgbLeds(1U, pins::kRgbData, NEO_GRB + NEO_KHZ800);

// LCM owns four local controls. Index order is fixed for the loops below.
enum LcmControl : uint8_t { kCtrlAv203, kCtrlAv205, kCtrlPwrSolLcm, kCtrlPwrPtLcm, kCtrlCount };
static aim::Control s_controls[kCtrlCount];

enum LcmSensor : uint8_t { kSenVsol, kSenCount };
static aim::Sensor s_sensors[kSenCount];

void nodeServiceLog(uint32_t nowMs, AimFlightRecorder& recorder) {
  (void)nowMs;
  (void)recorder;
}

static aim::Job s_voltSenseJob(2000U, 200U); // 0.5 Hz idle, 5 Hz active voltage sense
static aim::Job s_broadcastJob(1000U, 500U); // 1 Hz idle, 2 Hz active CAN broadcast

// VSOL sense: 12-bit ADC over 3.3 V ref through an 11:1 divider.
constexpr float kVoltSenseScale = (3.3f / 4095.0f) * 11.0f;

// Link watchdog timeout: 2.0s tolerates 1 missed heartbeat while safing in <2.0s on link loss
constexpr uint32_t kUpperStaleTimeoutMs = 2000U;
constexpr uint32_t kUpperEStopTimeoutMs = 300000U; // 5-minute CAN loss E-Stop watchdog
static uint32_t s_upperLastRxMs = 0U;
static bool     s_upperLinkUp   = false;
static bool     s_canEStopLatched = false;

// Full-scale range of the transducer physically installed on PT204. PtSpare2 is
// an unpopulated spare and keeps the prop_testing default.
constexpr float kPt204MaxPsi = 1000.0f;

void nodeInit() {

  // ADS131M04 requires an external MCLK. TIM2 CH1 drives PA0 at 8.192 MHz.
  __HAL_RCC_GPIOA_CLK_ENABLE();
  static HardwareTimer s_adcClock(TIM2);
  s_adcClock.setPWM(1, pins::kAdcClkin, 8192000, 50);
  s_adcClock.resume();

  s_adc.init();

  // Local controls. defaultOpen = logical state at the de-energized rest (LOW):
  //   Av203 NO valve → true  (rest open);  Av205 NC valve → false (rest closed);
  //   power relays   → true  (rest open = off). All boot de-energized (safe).
  controlInitLocal(s_controls[kCtrlAv203],     "AV203_FILL", aim::subject::Av203,     pins::kSol1En, true);
  controlInitLocal(s_controls[kCtrlAv205],     "AV205_COAX", aim::subject::Av205,     pins::kSol2En, false);
  controlInitLocal(s_controls[kCtrlPwrSolLcm], "PwrSolLcm",  aim::subject::PwrSolLcm, pins::kVsolEn, true);
  controlInitLocal(s_controls[kCtrlPwrPtLcm],  "PwrPtLcm",   aim::subject::PwrPtLcm,  pins::kVptEn,  true);

  sensorInitLocal(s_sensors[kSenVsol],     "VSOL",                   aim::subject::VoltSolLcm,   0.001f, "V");

  delay(100);

  s_upperLastRxMs = millis();

  analogReadResolution(12);

  s_rgbLeds.begin();
  s_rgbLeds.setPixelColor(0, s_rgbLeds.Color(0, 0, 0));
  s_rgbLeds.show();
}

static void updateLed(aim::NodeState state) {
  static aim::NodeState s_lastState = static_cast<aim::NodeState>(0xFF);
  if (state == s_lastState) return;
  s_lastState = state;
  uint8_t r = 0, g = 0, b = 0;
  switch (state) {
    case aim::NodeState::Nominal:   g = 255; break;           // Green (Healthy / Communicating)
    case aim::NodeState::Fault:     r = 255; g = 180; break; // Yellow (CAN Link Stale >2s)
    case aim::NodeState::SafeState: r = 255; break;           // Red (Timeout >5m / Hardware Safed)
    default:                        b = 255; break;           // Blue (Boot/Init)
  }
  s_rgbLeds.setPixelColor(0, s_rgbLeds.Color(r, g, b));
  s_rgbLeds.show();
}

static void executeSafingSequence() {
  // 1. Safe solenoid valves first (AV203, AV205)
  controlSetDefault(s_controls[kCtrlAv203]);
  controlSetDefault(s_controls[kCtrlAv205]);

  // 2. Allow coils to collapse magnetic field (100 ms)
  delay(100);

  // 3. Power off FETs (PwrSolLcm, PwrPtLcm)
  controlSetDefault(s_controls[kCtrlPwrSolLcm]);
  controlSetDefault(s_controls[kCtrlPwrPtLcm]);
}

void nodeUpdate(uint32_t nowMs) {
  updateLed(nodeCurrentState());

  // 5-minute CAN loss E-Stop check
  if (!s_canEStopLatched && ((nowMs - s_upperLastRxMs) >= kUpperEStopTimeoutMs)) {
    s_canEStopLatched = true;
    LOG_ERROR("UCM CAN link timeout (5m) — E-Stop triggered, safing valves then power FETs");
    executeSafingSequence();
  }

  // Voltage sense every tick
  {
    const uint32_t raw = analogRead(pins::kVsolSense);
    sensorSampleEng(s_sensors[kSenVsol], raw * kVoltSenseScale);
  }

  // UCM link staleness check
  {
    const bool currentlyUp = (nowMs - s_upperLastRxMs) < kUpperStaleTimeoutMs;
    if (currentlyUp != s_upperLinkUp) {
      s_upperLinkUp = currentlyUp;
      LOG_INFO("Upper Control link %s", s_upperLinkUp ? "UP" : "STALE");
    }
  }
}

void nodeServiceCanTx(uint32_t nowMs, AimNetwork& aim) {
  // Service control CAN traffic: pending ACKs / state re-sends.
  for (uint8_t i = 0U; i < kCtrlCount; i++) {
    controlServiceTx(s_controls[i], nowMs, aim);
  }

  // Voltage frame at 0.5-5 Hz
  if (s_voltSenseJob.due(nowMs)) {
    aim::Msg vMsg = {};
    sensorBuildFrame(s_sensors[kSenVsol], vMsg);
    (void)aim.send(vMsg);
  }

  // Local control STATE broadcast at 2 Hz
  if (s_broadcastJob.due(nowMs)) {
    for (uint8_t i = 0U; i < kCtrlCount; i++) {
      aim::Msg m = {};
      controlBuildState(s_controls[i], m);
      (void)aim.send(m);
    }
  }
}

void nodeOnRx(const aim::Msg& m, uint32_t nowMs) {
  if (m.source == aim::Source::Ucm) {
    s_upperLastRxMs = nowMs;
    s_canEStopLatched = false;
  }

  if (m.cls == aim::Class::Event) {
    return;  // Events are not control messages; skip control routing below
  }

  bool actuated = false;
  for (uint8_t i = 0U; i < kCtrlCount; i++) {
    actuated |= controlOnRx(s_controls[i], m);
  }
  if (actuated) {
    s_broadcastJob.lastMs = 0U; // force state broadcast in next nodeServiceCanTx
  }
}

aim::NodeState nodeCurrentState() {
  const uint32_t nowMs = millis();
  if ((nowMs - s_upperLastRxMs) >= kUpperEStopTimeoutMs) {
    return aim::NodeState::SafeState; // Red
  }
  return s_upperLinkUp ? aim::NodeState::Nominal : aim::NodeState::Fault; // Green vs Yellow
}

uint16_t nodeErrorBits() {
  return 0U;
}

#ifndef FLIGHT_BUILD
// Command a control by its index. Every LCM control is local, so this actuates
// immediately. No mutex here: the LCM runs single-threaded off the main loop.
static bool setControlByIndex(uint8_t index, bool open) {
  if (index >= kCtrlCount) {
    return false;
  }
  controlSet(s_controls[index], open);
  return true;
}

static void hookStatusSnapshot(Stream& out) {
  for (uint8_t i = 0U; i < kCtrlCount; i++) {
    out.printf("%-12s %s\n", s_controls[i].name,
               aim::controlStr(s_controls[i]));
  }
  for (uint8_t i = 0U; i < kSenCount; i++) {
    const aim::Sensor& s = s_sensors[i];
    if (sensorFresh(s)) {
      out.printf("%-22s %.2f %s\n", s.name, sensorEng(s), s.unit);
    } else {
      out.printf("%-22s UNKNOWN\n", s.name);
    }
  }
  out.printf("Upper link=%s\n", s_upperLinkUp ? "OK" : "STALE");
}

static void hookSetValve(Stream& out) {
  int index = aimConsoleWaitRead(out);
  if (index == ' ') {
    index = aimConsoleWaitRead(out);
  }
  if (index < '0' || index > '3') {
    for (uint8_t i = 0U; i < kCtrlCount; i++) {
      out.printf("  %u: %-12s %s\n", i, s_controls[i].name,
                 aim::controlStr(s_controls[i]));
    }
    out.println("Usage: v <0-3> <0|1>");
    return;
  }
  uint8_t ctrlIdx = index - '0';
  int state = aimConsoleWaitRead(out);
  if (state == ' ') {
    state = aimConsoleWaitRead(out);
  }
  if (state < '0' || state > '1') {
    out.println("Usage: v <0-3> <0|1>");
    return;
  }
  bool open = (state == '1');
  if (setControlByIndex(ctrlIdx, open)) {
    out.printf("%s -> %s\n", s_controls[ctrlIdx].name, aim::controlStr(s_controls[ctrlIdx]));
  } else {
    out.println("Set control failed");
  }
}

static const AimConsoleHook s_consoleHooks[] = {
  {'p', "status snapshot", hookStatusSnapshot},
  {'v', "valve/FET control", hookSetValve},
};

const AimConsoleHook* nodeConsoleHooks(uint8_t& count) {
  count = sizeof(s_consoleHooks) / sizeof(s_consoleHooks[0]);
  return s_consoleHooks;
}
#endif
