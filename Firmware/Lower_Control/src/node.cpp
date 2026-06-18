#include "node.h"
#include <logger.h>
#include <ADS131M04.h>
#include <TMAG5273.h>
#include <Adafruit_NeoPixel.h>
#include <prop_testing.h>
#include <aim_job.h>
#include <aim_control.h>

static ADS131M04 s_adc(pins::kAdcCs, pins::kAdcDrdy, &SPI);
static TwoWire s_wire1(pins::kHallSda1, pins::kHallScl1);
static TwoWire s_wire2(pins::kHallSda2, pins::kHallScl2);
static TMAG5273 s_hall1;
static TMAG5273 s_hall2;

static Adafruit_NeoPixel s_rgbLeds(1U, pins::kRgbData, NEO_GRB + NEO_KHZ800);

static bool s_adcPolling = true;
static bool s_hallPolling = true;

// LCM owns four local controls. Index order is fixed for the loops below.
enum LcmControl : uint8_t { kCtrlAv203, kCtrlAv205, kCtrlPwrSolLcm, kCtrlPwrPtLcm, kCtrlCount };
static aim::Control s_controls[kCtrlCount];

static aim::Job s_adcJob{500U};
static aim::Job s_hallJob{250U};
static aim::Job s_voltSenseJob{500U};
static aim::Job s_broadcastJob{500U};
static aim::Job s_ledJob{30U};

// VSOL sense: 12-bit ADC over 3.3 V ref through an 11:1 divider.
constexpr float kVoltSenseScale = (3.3f / 4095.0f) * 11.0f;

// ADC channel for each LCM-local sensor (both PTs are LCM-owned per the catalog).
constexpr uint8_t kAdcChPt204    = 0U;
constexpr uint8_t kAdcChPtSpare2 = 1U;
constexpr uint8_t kAdcChTc       = 2U;

void nodeInit(uint32_t nowMs) {
  (void)nowMs;

  // ADS131M04 requires an external MCLK. TIM2 CH1 drives PA0 at 8.192 MHz.
  __HAL_RCC_GPIOA_CLK_ENABLE();
  static HardwareTimer s_adcClock(TIM2);
  s_adcClock.setPWM(1, pins::kAdcClkin, 8192000, 50);
  s_adcClock.resume();

  s_wire1.begin();
  s_wire2.begin();

  s_adc.init();

  if (!s_hall1.init(0x35, s_wire1)) {
    LOG_ERROR("Hall sensor 1 init failed");
  }
  if (!s_hall2.init(0x35, s_wire2)) {
    LOG_ERROR("Hall sensor 2 init failed");
  }

  // Local controls. openLevel = the GPIO level that means logical-open:
  //   Av203 NO valve → LOW (open de-energized);  Av205 NC valve → HIGH;
  //   power rails → HIGH (on). All boot de-energized; VPT is switched on below.
  controlInitLocal(s_controls[kCtrlAv203], aim::subject::Av203,     pins::kSol1En, LOW);
  controlInitLocal(s_controls[kCtrlAv205], aim::subject::Av205,     pins::kSol2En, HIGH);
  controlInitLocal(s_controls[kCtrlPwrSolLcm],  aim::subject::PwrSolLcm, pins::kVsolEn, HIGH);
  controlInitLocal(s_controls[kCtrlPwrPtLcm],   aim::subject::PwrPtLcm,  pins::kVptEn,  HIGH);

  // PT power rail rests on; switch it up once configured (brief inrush settle).
  controlSet(s_controls[kCtrlPwrPtLcm], true);
  delay(100);

  analogReadResolution(12);

  s_rgbLeds.begin();
  s_rgbLeds.setPixelColor(0, s_rgbLeds.Color(0, 0, 0));
  s_rgbLeds.show();
}

static void pollADC(uint32_t nowMs, AimNetwork& aim) {
  if (!s_adcPolling || !s_adcJob.due(nowMs)) {
    return;
  }

  int32_t raw[4];
  float volts[4];
  if (!s_adc.readChannels(raw)) {
    return;
  }
  s_adc.computeVoltages(raw, volts);

  const float pt204    = processPT(volts[kAdcChPt204]);
  const float ptSpare2 = processPT(volts[kAdcChPtSpare2]);
  const float tempC    = processTC(volts[kAdcChTc], pins::kCjcSense);

  // CAN telemetry broadcasts
  aim::Msg pt1Msg = {};
  pt1Msg.cls = aim::Class::Sensor;
  pt1Msg.subject = aim::subject::Pt204;
  pt1Msg.setSensorValue(static_cast<int32_t>(pt204 * 100.0f));
  (void)aim.send(pt1Msg);

  aim::Msg pt2Msg = {};
  pt2Msg.cls = aim::Class::Sensor;
  pt2Msg.subject = aim::subject::PtSpare2;
  pt2Msg.setSensorValue(static_cast<int32_t>(ptSpare2 * 100.0f));
  (void)aim.send(pt2Msg);

  aim::Msg tcMsg = {};
  tcMsg.cls = aim::Class::Sensor;
  tcMsg.subject = aim::subject::TcLowerValve;
  tcMsg.setSensorValue(static_cast<int32_t>(tempC * 100.0f));
  (void)aim.send(tcMsg);
}

static void pollHall(uint32_t nowMs) {
  if (!s_hallPolling || !s_hallJob.due(nowMs)) {
    return;
  }

  (void)s_hall1.getFluxMagnitude();
  (void)s_hall2.getFluxMagnitude();
}

static void pollVoltSense(uint32_t nowMs, AimNetwork& aim) {
  if (!s_voltSenseJob.due(nowMs)) {
    return;
  }

  const uint32_t raw = analogRead(pins::kVsolSense);
  const float vActual = raw * kVoltSenseScale;

  aim::Msg vMsg = {};
  vMsg.cls = aim::Class::Sensor;
  vMsg.subject = aim::subject::VoltSolLcm;
  vMsg.setSensorValue(static_cast<int32_t>(vActual * 1000.0f)); // mV
  (void)aim.send(vMsg);
}

static void broadcastStates(uint32_t nowMs, AimNetwork& aim) {
  if (!s_broadcastJob.due(nowMs)) {
    return;
  }

  for (uint8_t i = 0U; i < kCtrlCount; i++) {
    aim::Msg m = {};
    controlBuildState(s_controls[i], m);
    (void)aim.send(m);
  }
}

void nodeUpdate(uint32_t nowMs) {
  pollHall(nowMs);

  // Oscillate brightness between 20 and 90 at ~35 BPM, refreshed at ~33 Hz.
  // 35 BPM is 35/60 Hz = 0.5833 Hz. Angular frequency omega = 2 * pi * 0.5833 = 3.6652 rad/s.
  // In ms: 0.0036652 rad/ms.
  if (s_ledJob.due(nowMs)) {
    uint8_t brightness = 20 + static_cast<uint8_t>(35.0f * (1.0f + sinf(nowMs * 0.0036652f)));
    s_rgbLeds.setPixelColor(0, s_rgbLeds.ColorHSV(34560, 255, brightness));
    s_rgbLeds.show();
  }
}

void nodeServiceCanTx(uint32_t nowMs, AimNetwork& aim) {
  // Emit any pending control ACKs.
  for (uint8_t i = 0U; i < kCtrlCount; i++) {
    controlServiceTx(s_controls[i], nowMs, aim);
  }

  pollADC(nowMs, aim);
  pollVoltSense(nowMs, aim);
  broadcastStates(nowMs, aim);
}

void nodeOnRx(const aim::Msg& m, uint32_t nowMs) {
  (void)nowMs;

  bool actuated = false;
  for (uint8_t i = 0U; i < kCtrlCount; i++) {
    actuated |= controlOnRx(s_controls[i], m);
  }
  if (actuated) {
    s_broadcastJob.lastMs = 0U; // force state broadcast in next nodeServiceCanTx
  }
}

aim::NodeState nodeCurrentState() {
  return aim::NodeState::Nominal;
}

uint16_t nodeErrorBits() {
  return 0U;
}

#ifndef FLIGHT_BUILD
static void hookSol1Test(Stream& out) {
  out.println("Actuating Solenoid 1 for 5 seconds...");
  enablePower(pins::kVsolEn);
  enableValve(pins::kSol1En);
  delay(5000);
  disableValve(pins::kSol1En);
  disablePower(pins::kVsolEn);
  out.println("Done.");
}

static void hookSol2Test(Stream& out) {
  out.println("Actuating Solenoid 2 for 5 seconds...");
  enablePower(pins::kVsolEn);
  enableValve(pins::kSol2En);
  delay(5000);
  disableValve(pins::kSol2En);
  disablePower(pins::kVsolEn);
  out.println("Done.");
}

static void hookTelemetryDump(Stream& out) {
  int32_t raw[4];
  float volts[4];
  if (s_adc.readChannels(raw)) {
    s_adc.computeVoltages(raw, volts);
    out.printf("Pt204: %.1f PSI (V=%.4f)\n", processPT(volts[kAdcChPt204]), volts[kAdcChPt204]);
    out.printf("PtSpare2: %.1f PSI (V=%.4f)\n", processPT(volts[kAdcChPtSpare2]), volts[kAdcChPtSpare2]);
    out.printf("TcLowerValve: %.1f C\n", processTC(volts[kAdcChTc], pins::kCjcSense));
  } else {
    out.println("ADC read error");
  }

  uint32_t rawV = analogRead(pins::kVsolSense);
  out.printf("VSOL: %.2f V (raw=%u)\n", rawV * kVoltSenseScale, rawV);
}

static const AimConsoleHook s_consoleHooks[] = {
  {'1', "actuate Solenoid 1 (5s)", hookSol1Test},
  {'2', "actuate Solenoid 2 (5s)", hookSol2Test},
  {'t', "telemetry snapshot", hookTelemetryDump},
};

const AimConsoleHook* nodeConsoleHooks(uint8_t& count) {
  count = sizeof(s_consoleHooks) / sizeof(s_consoleHooks[0]);
  return s_consoleHooks;
}
#endif
