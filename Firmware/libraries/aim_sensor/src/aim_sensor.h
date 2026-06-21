#ifndef AIM_SENSOR_H
#define AIM_SENSOR_H

// aim_sensor — one model for a scalar telemetry channel (pressure transducer,
// thermocouple, voltage sense, …) on the AIM CAN bus. It mirrors aim_control's
// Local/Remote split, but a sensor has no command machine: there is no
// Cmd/Ack/State, no retry, no GPIO — it is sampled and broadcast, or received
// and stored.
//
//   Local  — this node samples the value and broadcasts a Class::Sensor frame.
//   Remote — the value is produced by another node; this node receives + stores it.
//
// Values on the wire are catalog-scaled signed integers (no floats on the bus).
// Each sensor carries `toEng`, the factor that turns that integer into human
// engineering units for the console and ground-station telemetry, so the scaling
// magic numbers live in one place per sensor instead of scattered at every edge.

#include <Arduino.h>
#include <cstdint>

#include <aim_network.h>

namespace aim {

enum class SensorKind : uint8_t { Local, Remote };

struct Sensor {
  SensorKind  kind    = SensorKind::Local;
  const char* name    = "";      // human label for console + telemetry
  uint8_t     subject = 0U;      // aim::subject:: id matched on Sensor frames
  int32_t     value   = 0;       // last value, catalog-scaled wire integer
  float       toEng   = 1.0f;    // value * toEng = engineering units
  const char* unit    = "";      // engineering unit label (console display)
  bool        fresh   = false;   // a value is known (Local: sampled, Remote: received)
};

// Configure a sensor. `toEng` converts the wire integer to engineering units
// (e.g. 0.01f for a value scaled x100); `unit` is the display label ("PSI").
void sensorInitLocal(Sensor& s, const char* name, uint8_t subject, float toEng, const char* unit);
void sensorInitRemote(Sensor& s, const char* name, uint8_t subject, float toEng, const char* unit);

// Local: latch a freshly-sampled reading given in engineering units.
void sensorSampleEng(Sensor& s, float eng);

// Last value in engineering units.
float sensorEng(const Sensor& s);

// Whether a value is known yet (Local after first sample; Remote after first frame).
bool sensorFresh(const Sensor& s);

// Remote: store the value from a matching Class::Sensor frame. True if it was ours.
bool sensorOnRx(Sensor& s, const aim::Msg& m);

// Local: fill a Class::Sensor frame for the node to broadcast.
void sensorBuildFrame(const Sensor& s, aim::Msg& out);

}  // namespace aim

#endif  // AIM_SENSOR_H
