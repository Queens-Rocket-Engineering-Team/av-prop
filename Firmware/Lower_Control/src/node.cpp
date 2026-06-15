#include "node.h"
#include <logger.h>
#include <ADS131M04.h>
#include <TMAG5273.h>
#include <Adafruit_NeoPixel.h>
#include <prop_testing.h>
#include <aim_job.h>

static ADS131M04 s_adc(pins::kAdcCs, pins::kAdcDrdy, &SPI);
static TwoWire s_wire1(pins::kHallSda1, pins::kHallScl1);
static TwoWire s_wire2(pins::kHallSda2, pins::kHallScl2);
static TMAG5273 s_hall1;
static TMAG5273 s_hall2;

static Adafruit_NeoPixel s_rgbLeds(1U, pins::kRgbData, NEO_GRB + NEO_KHZ800);

static bool s_adcPolling = true;
static bool s_hallPolling = true;

static bool s_valve1State = true;  // Av203: default open (de-energized)
static bool s_valve2State = false; // Av205: default closed (de-energized)
static bool s_vsolState   = false; // PwrSolLcm: default off (de-energized)
static bool s_vptState    = true;  // PwrPtLcm: default on (de-energized)

static aim::Job s_adcJob{500U};
static aim::Job s_hallJob{250U};
static aim::Job s_voltSenseJob{500U};
static aim::Job s_broadcastJob{500U};
static aim::Job s_ledJob{30U};

static bool     s_pendingAckDirty = false;
static uint8_t  s_pendingAckSubject = 0U;
static uint8_t  s_pendingAckSeq = 0U;

// VSOL sense: 12-bit ADC over 3.3 V ref through an 11:1 divider.
constexpr float kVoltSenseScale = (3.3f / 4095.0f) * 11.0f;

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

  pinMode(pins::kVptEn, OUTPUT);
  pinMode(pins::kVsolEn, OUTPUT);
  pinMode(pins::kSol1En, OUTPUT);
  pinMode(pins::kSol2En, OUTPUT);

  enablePower(pins::kVptEn); // default on
  disablePower(pins::kVsolEn); // default off
  disableValve(pins::kSol1En); // default open
  disableValve(pins::kSol2En); // default closed

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

  const float psi1 = processPT(volts[0]);
  const float psi2 = processPT(volts[1]);
  const float tempC = processTC(volts[2], pins::kCjcSense);

  // CAN telemetry broadcasts
  aim::Msg pt1Msg = {};
  pt1Msg.cls = aim::Class::Sensor;
  pt1Msg.subject = aim::subject::Pt204;
  pt1Msg.setSensorValue(static_cast<int32_t>(psi1 * 100.0f));
  (void)aim.send(pt1Msg);

  aim::Msg pt2Msg = {};
  pt2Msg.cls = aim::Class::Sensor;
  pt2Msg.subject = aim::subject::PtSpare2;
  pt2Msg.setSensorValue(static_cast<int32_t>(psi2 * 100.0f));
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

  float a1[3], a2[3];
  (void)s_hall1.getAllFlux(a1);
  (void)s_hall2.getAllFlux(a2);
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

  aim::Msg m1 = {};
  m1.cls = aim::Class::State;
  m1.subject = aim::subject::Av203;
  m1.b[0] = s_valve1State ? 1U : 0U;
  m1.b[1] = s_vsolState ? 1U : 0U;
  (void)aim.send(m1);

  aim::Msg m2 = {};
  m2.cls = aim::Class::State;
  m2.subject = aim::subject::Av205;
  m2.b[0] = s_valve2State ? 1U : 0U;
  (void)aim.send(m2);

  aim::Msg m3 = {};
  m3.cls = aim::Class::State;
  m3.subject = aim::subject::PwrSolLcm;
  m3.b[0] = s_vsolState ? 1U : 0U;
  (void)aim.send(m3);

  aim::Msg m4 = {};
  m4.cls = aim::Class::State;
  m4.subject = aim::subject::PwrPtLcm;
  m4.b[0] = s_vptState ? 1U : 0U;
  (void)aim.send(m4);
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
  // Process pending ACK
  if (s_pendingAckDirty) {
    aim::Msg ack = {};
    ack.cls = aim::Class::Ack;
    ack.subject = s_pendingAckSubject;
    ack.b[0] = s_pendingAckSeq;
    ack.b[1] = 0; // Accepted
    if (aim.send(ack)) {
      s_pendingAckDirty = false;
    }
  }

  pollADC(nowMs, aim);
  pollVoltSense(nowMs, aim);
  broadcastStates(nowMs, aim);
}

void nodeOnRx(const aim::Msg& m, uint32_t nowMs) {
  (void)nowMs;

  if (m.cls == aim::Class::Cmd) {
    bool open = (m.b[1] == 1);
    bool actuate = false;

    if (m.subject == aim::subject::Av203) {
      s_valve1State = open;
      if (open) {
        disableValve(pins::kSol1En);
      } else {
        enableValve(pins::kSol1En);
      }
      actuate = true;
    } else if (m.subject == aim::subject::Av205) {
      s_valve2State = open;
      if (open) {
        enableValve(pins::kSol2En);
      } else {
        disableValve(pins::kSol2En);
      }
      actuate = true;
    } else if (m.subject == aim::subject::PwrSolLcm) {
      s_vsolState = open;
      if (open) {
        enablePower(pins::kVsolEn);
      } else {
        disablePower(pins::kVsolEn);
      }
      actuate = true;
    } else if (m.subject == aim::subject::PwrPtLcm) {
      s_vptState = open;
      if (open) {
        enablePower(pins::kVptEn);
      } else {
        disablePower(pins::kVptEn);
      }
      actuate = true;
    }

    if (actuate) {
      s_pendingAckSubject = m.subject;
      s_pendingAckSeq = m.b[0];
      s_pendingAckDirty = true;
      s_broadcastJob.lastMs = 0U; // force state broadcast in next nodeServiceCanTx
    }
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
    out.printf("PT1: %.1f PSI (V=%.4f)\n", processPT(volts[0]), volts[0]);
    out.printf("PT2: %.1f PSI (V=%.4f)\n", processPT(volts[1]), volts[1]);
    out.printf("TC: %.1f C\n", processTC(volts[2], pins::kCjcSense));
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
