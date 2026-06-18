# AimControl

One model for an actuated control (solenoid valve, power relay/FET) on the AIM CAN bus.

A `Control` is either **Local** — this node owns the GPIO, actuates it (de-energized = safe
default), Acks commands, and reports State — or **Remote** — the control lives on another node and
this node commands it over CAN with sequence/retry and tracks the owner's Ack/State.

The library fills CAN frames but never schedules them: the node owns broadcast cadence and calls
`controlServiceTx()` / `controlBuildState()` on its own tick.

QRET Avionics 25/26
