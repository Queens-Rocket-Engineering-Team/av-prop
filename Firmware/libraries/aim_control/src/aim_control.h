#ifndef AIM_CONTROL_H
#define AIM_CONTROL_H

// aim_control — one model for an actuated control (solenoid valve, power relay/FET)
// on the AIM CAN bus. A control plays one of two roles:
//
//   Local  — this node owns the GPIO. It actuates the pin, answers a Cmd with an
//            Ack, and reports State. De-energized (LOW) is always the safe boot
//            state.
//   Remote — the control lives on another node. This node is the commander: it
//            sends Cmd frames (with seq + retry) and tracks the owner's Ack/State.
//
// Only the UCM→LCM edge carries commands, so only Remote controls emit Class::Cmd.
// The library fills frames but never schedules them — the node owns broadcast
// cadence and calls controlServiceTx()/controlBuildState() on its own tick.
//
// One polarity knob: `defaultOpen` is the logical state at the de-energized rest
// (LOW). The pin is energized (HIGH) exactly when the commanded state differs from
// defaultOpen. This is the QLCP control model — a control is just OPEN/CLOSED and
// its de-energized rest is its default:
//   - normally-open valve   : defaultOpen = true   (rest open,   energize to close)
//   - normally-closed valve : defaultOpen = false  (rest closed, energize to open)
//   - power relay/FET        : defaultOpen = true   (rest open = off; energize = closed = on)
// `energized` ("pin is HIGH") is the physical truth reported in State b[1].

#include <Arduino.h>
#include <cstdint>

#include <aim_network.h>

namespace aim {

enum class ControlKind : uint8_t { Local, Remote };

// Resend an un-Acked Cmd after this window (Remote controls only).
constexpr uint32_t kCmdRetryMs = 500U;
constexpr uint8_t  kPinNone    = 0xFFU;  // Remote controls own no GPIO.

struct Control {
  ControlKind kind        = ControlKind::Local;
  const char* name        = "";      // human label for console/status/telemetry
  uint8_t     subject     = 0U;      // aim::subject:: id matched on Cmd/Ack/State
  bool        defaultOpen = false;   // logical state at de-energized rest (safe boot)
  bool        state       = false;   // last known logical state (commanded / reported)
  bool        energized   = false;   // Local: pin is HIGH; Remote: from State b[1]
  bool        confirmed   = false;   // state is known: Local always; Remote after first State

  // Local-owner bookkeeping (kind == Local):
  uint8_t     pin         = kPinNone;
  bool        pendingAck  = false;   // a Cmd was actuated and owes an Ack
  uint8_t     ackSeq      = 0U;      // seq to echo in that Ack

  // Remote-commander bookkeeping (kind == Remote):
  uint8_t     seq         = 0U;      // command sequence counter
  bool        desiredOpen = false;   // last commanded target
  bool        dirty       = false;   // Cmd needs (re)send now
  bool        awaitingAck = false;   // outstanding Cmd not yet Acked
  uint32_t    lastSentMs  = 0U;      // for retry timeout
};

// Configure a Local control. `defaultOpen` is the logical state at the de-energized
// rest (LOW); the control boots there (safe) and energizes (HIGH) only when commanded
// off its default.
void controlInitLocal(Control& c, const char* name, uint8_t subject, uint8_t pin, bool defaultOpen);

// Configure a Remote control; `defaultOpen` is this node's assumed boot state for it.
void controlInitRemote(Control& c, const char* name, uint8_t subject, bool defaultOpen);

// Command a control open/closed.
//   Local  — actuate the GPIO now and latch state.
//   Remote — queue a Cmd for the next controlServiceTx().
void controlSet(Control& c, bool open);

// Command the control back to its safe default (de-energized for Local).
void controlSetDefault(Control& c);

// Last known logical state.
bool controlGet(const Control& c);

// Whether `state` reflects the control's actual position. Local controls are
// always confirmed; a Remote control becomes confirmed on its owner's first State
// frame (an Ack does not confirm — only physical State does).
bool controlConfirmed(const Control& c);

// Feed every received frame here; returns true if it was for this control.
//   Local  — a Cmd actuates the pin and arms a pending Ack.
//   Remote — a matching Ack clears the await; a State updates state/energized.
bool controlOnRx(Control& c, const aim::Msg& m);

// Emit due CAN traffic for this control.
//   Local  — sends a pending Ack.
//   Remote — sends a fresh or timed-out Cmd.
void controlServiceTx(Control& c, uint32_t nowMs, AimNetwork& aim);

// Fill a State frame (b[0]=commanded, b[1]=energized) for the owner to broadcast.
void controlBuildState(const Control& c, aim::Msg& out);

// Human-readable "<position>/<power>" label: OPEN|CLOSED from logical state, plus
// ON|OFF from energized. "UNKNOWN" until confirmed.
const char* controlStr(const Control& c);

}  // namespace aim

#endif  // AIM_CONTROL_H
