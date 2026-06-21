#include "aim_control.h"

#include <aim_safety.h>  // AIM_ASSERT

namespace aim {

// Energize (drive HIGH) exactly when the commanded state differs from the
// de-energized rest. De-energized (LOW) is always the safe boot position.
static void localActuate(Control& c, bool open) {
  const bool energize = (open != c.defaultOpen);
  digitalWrite(c.pin, energize ? HIGH : LOW);
  c.energized = energize;
  c.state     = open;
}

void controlInitLocal(Control& c, const char* name, uint8_t subject, uint8_t pin, bool defaultOpen) {
  c = Control{};
  c.kind        = ControlKind::Local;
  c.name        = name;
  c.subject     = subject;
  c.pin         = pin;
  c.defaultOpen = defaultOpen;
  c.confirmed   = true;

  pinMode(pin, OUTPUT);
  localActuate(c, c.defaultOpen);  // boot de-energized = safe
}

void controlInitRemote(Control& c, const char* name, uint8_t subject, bool defaultOpen) {
  c = Control{};
  c.kind        = ControlKind::Remote;
  c.name        = name;
  c.subject     = subject;
  c.defaultOpen = defaultOpen;
  c.state       = defaultOpen;  // assume the owner also boots to its default
  c.desiredOpen = defaultOpen;
}

void controlSet(Control& c, bool open) {
  if (c.kind == ControlKind::Local) {
    localActuate(c, open);
    return;
  }

  // Remote: queue a Cmd only when it changes the target or starts a fresh exchange.
  if (c.dirty || c.desiredOpen != open || !c.awaitingAck) {
    c.desiredOpen = open;
    c.seq++;
    c.dirty       = true;
    c.awaitingAck = true;
  }
}

void controlSetDefault(Control& c) {
  controlSet(c, c.defaultOpen);
}

bool controlGet(const Control& c) {
  return c.state;
}

bool controlConfirmed(const Control& c) {
  return c.confirmed;
}

bool controlOnRx(Control& c, const aim::Msg& m) {
  if (m.subject != c.subject) {
    return false;
  }

  if (c.kind == ControlKind::Local) {
    if (m.cls == Class::Cmd) {
      controlSet(c, m.b[1] == 1U);
      c.ackSeq     = m.b[0];
      c.pendingAck = true;
      return true;
    }
    return false;
  }

  // Remote commander: consume the owner's Ack and State.
  if (m.cls == Class::Ack) {
    if (m.b[0] == c.seq) {
      c.awaitingAck = false;
    }
    return true;
  }
  if (m.cls == Class::State) {
    c.state     = (m.b[0] == 1U);
    c.energized = (m.b[1] == 1U);
    c.confirmed = true;  // owner physically reported — only State confirms, not Ack
    return true;
  }
  return false;
}

void controlServiceTx(Control& c, uint32_t nowMs, AimNetwork& aim) {
  if (c.kind == ControlKind::Local) {
    if (!c.pendingAck) {
      return;
    }
    aim::Msg ack = {};
    ack.cls     = Class::Ack;
    ack.subject = c.subject;
    ack.b[0]    = c.ackSeq;
    ack.b[1]    = 0U;  // accepted
    if (aim.send(ack)) {
      c.pendingAck = false;
    }
    return;
  }

  // Remote: send a fresh Cmd, or resend one that has gone un-Acked past the window.
  const bool resend = c.awaitingAck && ((nowMs - c.lastSentMs) >= kCmdRetryMs);
  if (!c.dirty && !resend) {
    return;
  }
  aim::Msg cmd = {};
  cmd.cls     = Class::Cmd;
  cmd.subject = c.subject;
  cmd.b[0]    = c.seq;
  cmd.b[1]    = c.desiredOpen ? 1U : 0U;
  if (aim.send(cmd)) {
    c.dirty      = false;
    c.lastSentMs = nowMs;
  }
}

void controlBuildState(const Control& c, aim::Msg& out) {
  out         = aim::Msg{};
  out.cls     = Class::State;
  out.subject = c.subject;
  out.b[0]    = c.state ? 1U : 0U;
  out.b[1]    = c.energized ? 1U : 0U;
}

const char* controlStr(const aim::Control& c) {
  if (!controlConfirmed(c)) {
    return "UNKNOWN";
  }
  if (controlGet(c)) {
    return c.energized ? "OPEN/ON" : "OPEN/OFF";
  }
  return c.energized ? "CLOSED/ON" : "CLOSED/OFF";
}

}  // namespace aim
