#include <prop_testing.h>

// --- Pressure transducer ---

float processPT(float voltagePt) {
  const float currentMa = (voltagePt / kPtShuntOhms) * 1000.0f;
  return (currentMa - 4.0f) * (kPtMaxPsi / 16.0f);
}

// --- Thermocouple ---

float readColdJunction(int thermPin) {
  const uint32_t raw = analogRead(thermPin);
  const float vOut = (raw / kAdcMaxCount) * kAdcVref;
  const float rTherm = kThermSeriesOhms * vOut / (kAdcVref - vOut);
  const float tKelvin = 1.0f / (1.0f / kThermT0Kelvin + (1.0f / kThermBeta) * log(rTherm / kThermR0Ohms));
  return tKelvin - 273.15f;
}

float readDeltaTemp(float voltage) {
  return (voltage * 1e6f) / kSeebeckUvPerC;
}

float processTC(float voltageTc, int thermPin) {
  return readColdJunction(thermPin) + readDeltaTemp(voltageTc);
}

// --- Power control ---

void enablePower(int enablePin) {
  digitalWrite(enablePin, HIGH);
  delay(100);
}

void disablePower(int enablePin) {
  digitalWrite(enablePin, LOW);
}

// --- Valve control ---

void enableValve(int solEnPin) {
  digitalWrite(solEnPin, HIGH);
}

void disableValve(int solEnPin) {
  digitalWrite(solEnPin, LOW);
}

// --- LED utilities ---

void blinkLed(int ledPin, int delayMs) {
  digitalWrite(ledPin, HIGH);
  delay(delayMs);
  digitalWrite(ledPin, LOW);
  delay(delayMs);
}

void flashLeds(const int ledArray[], int ledCount) {
  for (int i = 0; i < ledCount; i++) {
    digitalWrite(ledArray[i], HIGH);
    delay(800);
    digitalWrite(ledArray[i], LOW);
  }
}
