# AV Propulsion Fleet Architecture

An approach for Upper_Control, Lower_Control, and the planned GPS/Alt/Power nodes,
derived from what the codebase already wants to be: a fleet of small CAN nodes with
one gateway, sharing a protocol contract and a service library, with per-board code
reduced to hardware and behavior.

## The shape of the system

Upper_Control is not "a board with extra networking" — it is the **launch-ops
gateway**: during ground/launch operations it is the bridge between the QLCP/WiFi
ground station and exactly one peer, Lower_Control. That scoping matters as much as
the role itself. It is not a fleet gateway; GPS/Alt/Power are planned CAN nodes,
but nothing yet says their data flows through this QLCP link, so nothing here is
designed for them. The gateway role still answers the questions that felt
ambiguous: Why does Upper advertise Lower's PTs? Because representing Lower to the
ground station is its launch-ops job. Why a cache? Because QLCP reads must be
answered from local state, never by blocking on a CAN round-trip. Why do endpoint
IDs feel like they belong to no one board? Because Upper's and Lower's are
inter-board contract.

## Four layers

**1. Contract (aim_protocol).** One header (or small set) in av-libraries holding
everything two boards must agree on to interoperate: `aim::Node`, `aim::PacketType`,
the `Pkt` encoding, the network version string, and — newly promoted — the
**per-node endpoint tables**. `BoardEndpointId` dissolves into this layer: each node
gets a namespaced enum (`aim::uprop::Endpoint`, `aim::lprop::Endpoint`) whose values
are typed as `aim::EndpointId`. The gateway includes Lower's table to mirror it;
Lower includes its own to serve it. A mismatch becomes a compile error instead of a
wire bug. This layer also keeps the roadmap surface: GPS/Alt/Power nodes, their
packet types, LOW_POWER/FAULT states. Contract entries are cheap to hold
and expensive to churn, so they may exist before their implementations — each marked
with a one-line note naming the trigger that will make it real.

**2. Platform.** Hardware-facing drivers with no opinions: CAN cores (ESP32/STM32),
block devices, the filesystem. Already in good shape. The rule here is the thin-HAL
rule: translate hardware to calls, push policy (retries, timing, safety) up.

**3. Services.** Cross-board mechanisms with policy but no hardware:
logger, AimConfigStore, AimFlightRecorder, AimNetwork — and, promoted from
board code, **AimNodeConfig**: the fleet-generic config struct (name, CAN ID,
telemetry schema) with load/save/print built on AimConfigStore, returning
AimConfigLoad. Every board today either has a board_config or will copy one; this
is the deduplication that justifies pushing down. What does *not* move down:
anything mentioning a pin, a specific sensor, or the flight recorder
(format-for-maintenance is console orchestration and lives with the console).

**4. Application (per board).** What remains per-board after the layers above is
deliberately small, and its size is the health metric for the whole architecture:
the pin map, the hardware service functions, the board's state-machine behavior,
its transmit behaviors, and — gateway only — the QLCP stack and the mirror. A leaf
node's application should fit in two files a newcomer can read in one sitting.

## The node skeleton (identical on every board)

Every node, gateway or leaf, runs the same bounded loop: drain CAN RX (capped
frames per tick), run the state machine, service the board's hardware, fire due
transmits, kick the watchdog. States come from the shared contract
(INIT, OPERATIONAL, DEBUG_CONSOLE, LOW_POWER, FAULT); each board
implements only the states it has behavior for, and unimplemented planned states
are a logged transition plus a TODO naming their trigger (LOW_POWER: power
board sends shutoff command → flip 24V FET; FAULT: fault handling).
Keeping the state enum shared means ground-station tooling and the flight recorder
interpret state numbers identically across the fleet.

## The gateway mirror

This is the next real feature, and it deserves to be designed rather than accreted.
The mirror is a fixed-size table with one row per Lower_Control endpoint the
gateway represents — sized from Lower's contract table, not a generic
(node, endpoint) space. If a second mirrored peer ever appears, widening the key is
a small change; designing for it today is not warranted.

```c
struct MirrorRow {
    aim::EndpointId endpoint;   // Lower_Control endpoint (owner of the truth)
    uint32_t        value;      // last received payload
    uint32_t        rxMs;       // synced time of last update
    bool            valid;      // ever received?
};
```

Three flows touch it. **Inbound:** every CAN packet from Lower updates its row —
this is the only writer for mirrored rows; local sensors keep their existing
globals, and QLCP serving reads both uniformly.
**Serving:** QLCP telemetry and reads come from the table, never from a CAN
round-trip; a row older than its staleness budget reports stale/invalid rather than
a frozen value, because a gateway that serves stale data as fresh is lying with
extra steps. **Controls:** a command targeting a local endpoint actuates GPIO; a
command targeting a remote endpoint becomes a latched CAN transmit (retained until
acked or superseded) and the mirror reports the *observed* state echoed back by Lower, not the commanded state — the ground station should see what the valve did,
not what the gateway hoped.

kBoardQlcpConfigJson is the static face of this table. It stays as-is: it already
describes the mirror the gateway is committed to providing. Implementation catches
up to the contract; the contract does not shrink to match the implementation.

## Transmit policy

AimTxPolicy is deleted from Upper_Control in favor of explicit inline timers —
five behaviors do not justify a scheduler, and the current design's ownership of
payload formatting is why 64-bit payloads are impossible. The mirror is the event
that will earn a successor: retransmitting N mirrored endpoints with per-row
periods and staleness is a genuine table-driven problem. When that day comes, the
v2 scheduler owns *when* and never *what*: rows hold fully-built `aim::Pkt`s (any
encoding, including raw 64-bit data) and the scheduler just compares timestamps and
calls sendPkt. Until then, the library keeps no scheduler at all.

## Subtract-first rules of the road for this repo

A mechanism earns library status on the rule of three across *boards*, not uses:
the second board that needs it is a coincidence, the third is a pattern — with one
exception, the contract layer, where sharing is the entire point and a single
planned consumer suffices. Planned contract surface (enums, endpoints, states,
QLCP advertisements) stays and is annotated with its trigger; speculative
implementation waits for its trigger to exist. When something feels rigid, remove
the knowledge it shouldn't have before adding a mode. And the standing health
check: if a leaf board's application layer stops fitting in two readable files,
something is living at the wrong layer.

## Migration order (each step builds and flies on its own)

First, hardware truth in Upper_Control: thermocouple endpoint/column/global out,
Hall array to [2], FET array out — small, safe, immediate. Second, promote
endpoints: move endpoint definitions into the contract header as per-node tables,
delete BoardEndpointId, build both boards. Third, promote config: AimNodeConfig
into services, board_config shrinks to defaults plus console-owned maintenance,
delete ConfigStatus. Fourth, delete AimTxPolicy usage: inline the five timers in
boardServiceTx, keep latched-retry semantics for valves, then remove the module
once Lower_Control confirms it has no second consumer. Fifth, build the mirror —
on top of the contract tables from step two, serving QLCP from the table and
routing controls local-vs-remote. Each step ends with -Wall -Wextra -Werror on
both board environments and a QLCP client check.

## Open questions to settle before step two

Whether Lower_Control's endpoint numbering already conflicts with Upper's
assumptions (audit before promoting tables); whether g_24VoltageSense[2] means
local-plus-mirrored (in which case the second slot becomes a mirror row, not a
local global); and what the ack/echo packet for remote valve commands looks like —
the mirror's "observed state" rule depends on Lower echoing actuator state, which
may already be true via the existing valve telemetry.
