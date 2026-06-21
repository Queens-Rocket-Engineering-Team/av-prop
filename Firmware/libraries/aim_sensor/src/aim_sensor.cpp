#include "aim_sensor.h"

#include <math.h>  // lroundf

namespace aim {

void sensorInitLocal(Sensor& s, const char* name, uint8_t subject, float toEng, const char* unit) {
  s = Sensor{};
  s.kind    = SensorKind::Local;
  s.name    = name;
  s.subject = subject;
  s.toEng   = toEng;
  s.unit    = unit;
}

void sensorInitRemote(Sensor& s, const char* name, uint8_t subject, float toEng, const char* unit) {
  s = Sensor{};
  s.kind    = SensorKind::Remote;
  s.name    = name;
  s.subject = subject;
  s.toEng   = toEng;
  s.unit    = unit;
}

void sensorSampleEng(Sensor& s, float eng) {
  s.value = static_cast<int32_t>(lroundf(eng / s.toEng));
  s.fresh = true;
}

float sensorEng(const Sensor& s) {
  return static_cast<float>(s.value) * s.toEng;
}

bool sensorFresh(const Sensor& s) {
  return s.fresh;
}

bool sensorOnRx(Sensor& s, const aim::Msg& m) {
  if (s.kind != SensorKind::Remote || m.cls != Class::Sensor || m.subject != s.subject) {
    return false;
  }
  s.value = m.sensorValue();
  s.fresh = true;
  return true;
}

void sensorBuildFrame(const Sensor& s, aim::Msg& out) {
  out         = aim::Msg{};
  out.cls     = Class::Sensor;
  out.subject = s.subject;
  out.setSensorValue(s.value);
}

}  // namespace aim
