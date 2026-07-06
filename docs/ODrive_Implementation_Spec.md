# ODrive v3.6 USB Servo Integration — Implementation Spec

This document specifies a full design for driving one or two axes of a
3D printer (or other CNC-style machine) using an
[ODrive v3.6](https://odriverobotics.com/) brushless servo controller
(including clone boards) in closed-loop position mode, connected to the
Klipper **host** over USB — no step/dir wiring required. All
configuration, calibration, and tuning is achievable from a web browser
(the Mainsail/Fluidd console) with no `odrivetool` install on the host.

This is a design document for a **greenfield feature**: no ODrive or
generic closed-loop-servo support exists anywhere in Klipper or Kalico
today. Upstream Klipper has twice declined ODrive support requests
([Klipper3d/klipper#3151](https://github.com/Klipper3d/klipper/issues/3151),
[#4302](https://github.com/Klipper3d/klipper/issues/4302)) on the grounds
that USB-streamed motion is a poor fit for its hard-real-time step
generation model. Kalico, which already accepts more experimental/"danger"
features than mainline, is a reasonable home for it. This spec treats the
tension head-on: **Klipper's step/dir motion pipeline is not reused for
ODrive axes.** Instead, the ODrive itself performs closed-loop control,
and Klipper streams it position setpoints derived from the same trapezoid
motion queue (`trapq`) that ordinary steppers consume.

## Design goals and non-goals

Goals:

- Plug an ODrive v3.6 (or a clone) into the host over USB.
- Use one or both of its motor channels as printer axes (X/Y/Z, or any
  axis in any of Kalico's kinematics: cartesian, corexy, delta, etc.)
  in ODrive closed-loop position-control mode.
- Support clone boards running any firmware in the 0.5.1–0.5.6 range,
  without assuming a specific firmware build or a working `odrivetool`
  installation.
- Make every step of setup — property inspection, motor/encoder
  calibration, gain tuning, homing method selection, error diagnosis —
  possible from gcode issued through a web console, with results
  persisted either to the ODrive's own NVM or to `printer.cfg` via the
  normal `SAVE_CONFIG` mechanism.
- Behave safely: an ODrive that stops responding while it is a live,
  homed motion axis must not silently under-perform — it must halt the
  machine, the same way an unresponsive MCU would.

Non-goals (out of scope for this spec, mentioned only where relevant):

- Hard-real-time step/dir wiring to the ODrive. This is documented as a
  supported *alternative* wiring (the ODrive's GPIOs can be configured
  for step/dir input in 0.5.x), but the module described here targets
  USB-streamed position control specifically, per the request that
  motivated this design.
- CAN-based ODrive control. The ODrive v3.6's CAN transceiver support in
  the 0.5.x line is limited, and CAN in the Kalico/Klipper codebase is
  currently only used to connect Klipper-firmware MCUs, not third-party
  CAN devices with a different protocol.
- Support for ODrive Pro/S1 or firmware 0.6.x. That firmware line is
  closed-source and does not run on v3.6 hardware or its clones.

## Prior art review

Grep across the tree confirms there is nothing to build on for the motor
control itself, but four modules provide directly reusable patterns and
are referenced throughout this spec:

| File | What it demonstrates |
|---|---|
| `klippy/extras/palette2.py` | Opening a **host-side serial device** (not behind an MCU) and driving reads/writes cooperatively via `reactor` timers/queues, with graceful handling of a device that may be absent or disconnect. |
| `klippy/extras/manual_stepper.py` | A **self-contained motion source**: owns its own `trapq`, drives `generate_steps` itself, and implements the minimal "toolhead-like" interface (`flush_step_generation`, `get_position`, `set_position`, `get_last_move_time`, `dwell`, `drip_move`, `get_kinematics`, `get_steppers`, `calc_position`) that `homing.py` needs to home a stepper that isn't part of the main toolhead. |
| `klippy/extras/probe_eddy_current.py` | A **host-driven virtual endstop** (`EddyEndstopWrapper`): implements `home_start`/`home_wait`/`query_endstop`/`add_stepper`/`get_steppers` using host-side sensor sampling rather than firmware trigger-sync hardware. This is the direct template for triggering `G28` from an ODrive's own position-error or encoder-index signal. |
| `klippy/extras/tmc.py` | The mux-command-family pattern (`SET_TMC_FIELD`/`DUMP_TMC`/`INIT_TMC`), the `TMCVirtualPinHelper` pattern for registering `foo:virtual_endstop` pin chips, and the `homing:homing_move_begin/end` event hooks used to temporarily change drive current during a homing move (sensorless homing) — directly reusable for ODrive's own sensorless-style homing. |

Also load-bearing to this design, and verified directly against this
repository:

- `klippy/extras/motion_report.py:164` (`DumpTrapQ.get_trapq_position`)
  proves that **exact analytic commanded position and velocity** for any
  `print_time` can be computed on the host with no MCU round-trip at
  all — it walks the `trapq`'s cartesian move segments and evaluates the
  standard `x = x0 + v0*t + 0.5*a*t²` kinematics in Python/C. This is the
  seed of the setpoint-streaming design in [Motion streaming](#motion-streaming-design).
  Critically, that function only reads `tq->history`
  (`klippy/chelper/trapq.c:237`, `trapq_extract_old`), i.e. only *completed*
  move segments — moves still in progress are not there yet. A homing
  move or a long axis travel can be several seconds long, so a real-time
  sampler must read the **live** queue instead; see
  [`trapq_extract_pending`](#c-helper-trapq_extract_pending).
- `klippy/stepper.py:618` (`LookupMultiRail`) is called by **every**
  kinematics module to build a rail from a `[stepper_x]`-style config
  section — verified in `cartesian.py`, `corexy.py`, `corexz.py`,
  `delta.py`, `deltesian.py`, `colinear_tripteron.py`, `hybrid_corexy.py`,
  `hybrid_corexz.py`, `polar.py`, and `extras/trad_rack.py`. This is the
  single choke point where an ODrive-backed rail can be substituted with
  **no changes to any kinematics class**.
- `klippy/extras/homing.py:100` (`HomingMove.homing_move`) defines the
  duck-typed endstop interface (`home_start`, `home_wait`,
  `query_endstop`, `add_stepper`, `get_steppers`) that any custom
  endstop, ODrive-backed or not, must satisfy.

## Architecture overview

```
                     ┌───────────────────────────────────────────┐
                     │              klippy/toolhead.py            │
                     │   trapq (cartesian, kinematics-agnostic)    │
                     └───────────────┬─────────────────────────────┘
                                     │ sampled by
                     ┌───────────────▼─────────────────────────────┐
                     │      klippy/extras/odrive/streamer.py        │
                     │  trapq sample → itersolve → mm → turns       │
                     └───────────────┬─────────────────────────────┘
                                     │ "p <motor> <pos> <vel_ff> 0*NN"
                     ┌───────────────▼─────────────────────────────┐
                     │      klippy/extras/odrive/transport.py       │
                     │   ASCII-over-USB-CDC, checksums, reconnect   │
                     └───────────────┬─────────────────────────────┘
                                     │ /dev/serial/by-id/...
                              ┌──────▼──────┐
                              │  ODrive v3.6 │  (closed-loop control
                              │   or clone   │   runs entirely on-board)
                              └─────────────┘
```

Two motors, two encoders — an `[odrive_axis]` object per motor, sharing
one `[odrive]` transport object per board (one USB connection, one
serial link, both `p`-stream lines interleaved on it).

### Module layout

An extras **package**, following the precedent of `klippy/extras/display/`
and `klippy/extras/load_cell/` (both multi-file extras packages loaded
the same way a single-file module would be):

```
klippy/extras/odrive/
    __init__.py      # load_config_prefix for [odrive ...] and [odrive_axis ...]
    transport.py      # ODriveSerial: ASCII framing, checksums, request/response FIFO, reconnect FSM
    properties.py      # PropertyMap: fw-version shim, enum tables, error-bitfield decode tables
    axis.py            # ODriveAxis: per-motor config push, calibration sequencer, arm/disarm, error/telemetry poll
    odrv_stepper.py    # ODriveStepper + ODriveRail: the rail/stepper duck-types bound into kinematics
    streamer.py        # SetpointStreamer: trapq sampling, unit conversion, TX pacing
    endstop.py          # ODriveVirtualEndstop (sensorless) + PhysicalEndstopWrapper
    telemetry.py        # webhooks: odrive/telemetry bulk stream, odrive/property_read
```

Two small patches to core Klipper/Kalico files (everything else lives in
the extras package above):

1. **`klippy/stepper.py`**, in `LookupMultiRail` (line 618): a ~10-line
   dispatch — if the `[stepper_x]`-style config section defines
   `odrive_axis:`, delegate rail construction to the `odrive` package
   instead of building a normal `PrinterRail`. This is what lets every
   existing kinematics class use an ODrive axis with **zero** changes to
   `klippy/kinematics/*.py`.

   ```python
   def LookupMultiRail(config, need_position_minmax=True,
                        default_position_endstop=None, units_in_radians=False):
       if config.get('odrive_axis', None) is not None:
           odrv_mod = config.get_printer().load_object(config, 'odrive')
           return odrv_mod.lookup_rail(config, need_position_minmax,
                                        default_position_endstop)
       # ...existing PrinterRail construction...
   ```

2. **`klippy/chelper/trapq.c`** (+ declaration in
   `klippy/chelper/__init__.py`): a new `trapq_extract_pending()`
   function, described below.

#### C helper: `trapq_extract_pending`

`trapq_extract_old` (`klippy/chelper/trapq.c:232`) walks `tq->history` —
a linked list populated only by `trapq_finalize_moves` once a move's end
time has passed (`klippy/chelper/trapq.c:184`). A homing move or a long
print move can be in flight for seconds; during that whole window it is
**not** in `history`, so `get_trapq_position`-style sampling returns
nothing for it. `trapq_extract_pending` is a ~25-line mirror of
`trapq_extract_old` that walks the **live** `tq->moves` list instead,
using the same `struct pull_move` output shape so `streamer.py` can reuse
the exact position/velocity math already proven correct by
`motion_report.py`:

```c
int
trapq_extract_pending(struct trapq *tq, struct pull_move *p, int max
                       , double start_time, double end_time)
{
    int res = 0;
    struct move *m;
    list_for_each_entry(m, &tq->moves, node) {
        if (m->print_time + m->move_t < start_time)
            continue;
        if (m->print_time > end_time)
            break;
        p[res].print_time = m->print_time;
        p[res].move_t = m->move_t;
        p[res].start_v = m->start_v;
        p[res].accel = m->half_accel * 2.;
        p[res].start_x = m->start_pos.x;
        p[res].start_y = m->start_pos.y;
        p[res].start_z = m->start_pos.z;
        p[res].x_r = m->axes_r.x;
        p[res].y_r = m->axes_r.y;
        p[res].z_r = m->axes_r.z;
        res++;
        if (res >= max)
            break;
    }
    return res;
}
```

`streamer.py` calls this once per TX tick with a small `(start_time,
end_time)` window around the sampling frontier (see
[Motion streaming design](#motion-streaming-design)), caches the returned
segments, and evaluates position/velocity in Python using the identical
formulas already used by `DumpTrapQ.get_trapq_position`
(`klippy/extras/motion_report.py:164-179`). Falling back to
`trapq_extract_old` for any time already retired to history keeps the
sampler correct across the moment a move is finalized mid-sample.

### Config sections

```ini
[odrive drive0]
# one physical board / one USB connection, up to two motors
serial: /dev/serial/by-id/usb-ODrive_...
# ...see "Configuration reference" below for the full option list

[odrive_axis x_motor]
odrive: drive0
axis: 0                         # 0 = M0, 1 = M1
# motor / encoder / controller parameters — see reference

[stepper_x]
odrive_axis: x_motor            # replaces step_pin / dir_pin / microsteps
rotation_distance: 40           # mm per ODrive-motor revolution (belt/pulley/leadscrew)
endstop_pin: odrive_axis_x_motor:virtual_endstop   # or a normal MCU pin
# position_min / position_max / homing_speed / etc. — unchanged rail options
```

`[stepper_x]` keeps its ordinary name and the ordinary rail options
(`position_min`, `position_max`, `homing_speed`, `second_homing_speed`,
`homing_retract_dist`, `min_home_dist`, `use_sensorless_homing`, …) — a
kinematics author or user reading a config file sees a normal rail
section, just with `odrive_axis:` where `step_pin:`/`dir_pin:` would
otherwise be.

`[odrive_axis]` is a separate section from `[odrive]` (the board) and
from `[stepper_x]` (the rail) because it owns data that belongs to
neither: the motor/encoder electrical and calibration parameters. This
also lets an `[odrive_axis]` be armed and driven standalone (e.g. for a
non-kinematic spindle or as a bench-test target before it is ever wired
into a `[stepper_x]`) — a `[stepper_x] odrive_axis:` reference is optional,
not required, for an `[odrive_axis]` to be usable.

Rejected alternatives:

- **Folding motor/encoder options into `[stepper_x]` directly.** This
  conflates rail-level config (homing, travel limits) with motor-level
  config (calibration, tuning) and prevents standalone use of an
  `[odrive_axis]` outside kinematics.
- **A single `[odrive drive0]` section with `axis0_*`/`axis1_*` prefixed
  options for both motors.** Produces unreadable 50+ option sections and
  breaks the one-section-per-device convention used throughout Kalico
  (compare `[tmc2209 stepper_x]`, one per driver).
- **Virtual step/dir pins** (e.g. `step_pin: odrive_drive0:step0`) that
  let `[stepper_x]` keep using the stock `PrinterRail`/`MCU_stepper`
  machinery unmodified. Rejected because `MCU_stepper.__init__`
  (`klippy/stepper.py:25-70`) requires its pin's `chip` to be a
  full `MCU`-like object providing `create_oid`,
  `register_config_callback`, `register_stepqueue`, `lookup_command`,
  and clock conversion — faking an entire MCU transport just to satisfy
  that constructor is considerably more code and risk than the rail
  duck-type approach below, for no benefit (there would still be no real
  stepcompress/serialqueue to send steps to).

## The `ODriveStepper` / `ODriveRail` duck-types

The core trick that keeps every kinematics class untouched: `ODriveStepper`
still allocates a **real** itersolve `stepper_kinematics` object — the
exact one a kinematics class asks for via `setup_itersolve("cartesian_stepper_alloc",
b"x")` and friends — but it never attaches a `stepcompress` sink and
never calls `itersolve_generate_steps`. Checking the C source confirms
this is safe: every itersolve entry point a kinematics or homing routine
calls during normal operation —
`itersolve_calc_position_from_coord`, `itersolve_set_position`,
`itersolve_get_commanded_pos`, `itersolve_is_active_axis`,
`itersolve_set_trapq` — only touches the `stepper_kinematics` struct
itself; only `itersolve_generate_steps` (never called here) touches the
`stepcompress` field. This gets exactly correct coordinate math (X/Y/Z
↔ this-axis position) for cartesian, corexy, delta, or any future
kinematics, entirely for free, with no per-kinematics-type code in the
ODrive module.

`ODriveStepper` implements the same surface that Kalico's own homing and
kinematics code calls on a real `MCU_stepper`:

| Method | Behavior |
|---|---|
| `get_name(short=False)` | The `[stepper_x]` section name, with the same shortening rule as `MCU_stepper.get_name`. |
| `get_step_dist()` | `rotation_distance / virtual_steps_per_rotation` (config option, default 4000 steps/rev — i.e. 40 mm/rev ÷ 4000 ≈ 10 µm virtual step, a resolution fine enough that Kalico's existing distance-moved bookkeeping (`check_no_movement`, rehoming tolerance) behaves indistinguishably from a real stepper). |
| `get_commanded_position` / `set_position` / `calc_position_from_coord` / `is_active_axis` | Delegated straight to itersolve, as above. |
| `get_mcu_position(cmd_pos=None)` | `round((cmd_pos_or_commanded + mcu_pos_offset) / step_dist)` — mirrors the real `MCU_stepper.get_mcu_position` formula (`klippy/stepper.py:217-224`) so downstream distance-moved math is unchanged. |
| `mcu_to_commanded_position(mcu_pos)` | Inverse of the above (mirrors `klippy/stepper.py:236-237`). |
| `get_past_mcu_position(print_time)` | Interpolated from the streamer's **sent-sample ring buffer** (see below) rather than a firmware query — there is no firmware position register to ask. |
| `setup_itersolve` / `set_stepper_kinematics` / `get_stepper_kinematics` | Allocate/attach the itersolve object; preserve an `mcu_pos_offset` the same way `MCU_stepper.set_stepper_kinematics` does. |
| `set_trapq(tq)` / `get_trapq()` | Forwarded to itersolve *and* recorded so the `SetpointStreamer` knows which trapq to sample. |
| `generate_steps(flush_time)` | **Emits no steps.** Forwards `flush_time` to the bound `SetpointStreamer` as its sampling frontier. This is the same call site (`toolhead._advance_flush_time`, called once per flush cycle before `trapq_finalize_moves`) a real stepper uses to generate step pulses; here it is repurposed as "how far ahead has the toolhead planned." |
| `get_mcu()` | Returns the owning `ODriveBoard`, which implements the small subset of the `MCU` interface that generic code paths touch: `estimated_print_time(eventtime)`, `is_fileoutput()`, `get_name()`. This keeps `ODriveStepper` usable anywhere code does `stepper.get_mcu().estimated_print_time(...)`. |
| `add_active_callback`, `note_homing_end`, `get_tmc_current_helper` | No-ops / `None`, matching the optional-feature contract other stepper-likes satisfy. |

`ODriveRail` mirrors `PrinterRail`'s public surface (`klippy/stepper.py:412`):
`get_steppers`, `get_endstops`, `get_range`, `get_homing_info` (building
the same namedtuple Kalico's `PrinterRail.get_homing_info` builds,
including the Kalico-specific fields `use_sensorless_homing`,
`min_home_dist`, `homing_accel`, `homing_retract_speed` —
`klippy/stepper.py:465-553`), `setup_itersolve`, `generate_steps`,
`set_trapq`, `set_position`, `get_commanded_position`,
`calc_position_from_coord`, `get_name`, and
`get_tmc_current_helpers() -> [None]` (called by
`homing.py`'s `_set_homing_current`, `klippy/extras/homing.py:279-298`,
so it must exist even when there is no TMC driver on this rail).
`add_extra_stepper` raises `config.error` — multi-stepper rails backed by
one ODrive motor are not supported in this design.

## Transport design

### Protocol choice: ASCII over USB-CDC

The ODrive exposes two independent USB interfaces simultaneously: a
CDC-ACM virtual serial port (protocol selectable; default is the
line-oriented **ASCII protocol**) and a vendor-specific interface that
always speaks the **native (Fibre) protocol**
([ascii-protocol.rst](https://github.com/odriverobotics/ODrive/blob/master/docs/ascii-protocol.rst),
[protocol.rst](https://github.com/odriverobotics/ODrive/blob/master/docs/protocol.rst)).

This design uses **ASCII exclusively**, and does not use the `odrive`
pip package (which wraps the native protocol via `libfibre`). Reasons:

- The native protocol addresses every property/function by a numeric
  **endpoint ID** resolved from a JSON object-tree definition read from
  endpoint 0 at connect time, validated by a **CRC16 of that JSON as the
  packet trailer**. This ties the protocol to the exact firmware build.
  Since this module must tolerate arbitrary clone firmware in the
  0.5.1–0.5.6 range with no advance knowledge of the exact build, a
  protocol whose addressing scheme is a hash of the firmware's own
  schema is the wrong foundation.
- The `odrive` Python package is versioned in lockstep with firmware
  (ODrive's own guidance: "odrivetool 0.6.11 requires firmware 0.6.11 or
  newer") and pulls in `libfibre` + `libusb` as native dependencies —
  more moving parts than a from-scratch clone needs to tolerate.
- ASCII's `r <property>` / `w <property> <value>` commands reach every
  numeric property by **name** (a dotted path string), which degrades
  gracefully: an unknown/renamed property on an unusual clone firmware
  just gets an error response for that one property, not a protocol
  handshake failure for the whole connection. This is exploited directly
  in the [firmware tolerance layer](#firmware-version-tolerance-layer).

Every line, in both directions, may carry a GCode-style checksum suffix:
`<command> *NN`, where `NN` is the XOR of every character before the `*`.
`use_checksums: True` is the default; the ODrive echoes a checksum back
only when the request carried one. Framing:

- `p <motor> <pos> <vel_ff> <torque_ff>` — streams a position setpoint
  (turns) with velocity/torque feed-forward; **feeds the watchdog**; no
  reply.
- `v <motor> <vel> <torque_ff>`, `c <motor> <torque>`, `t <motor> <dest>`
  — velocity/torque/trapezoidal setpoints; also feed the watchdog; no
  reply. Used only for `ODRIVE_AXIS_MOVE` (standalone jogging), never
  while an axis is bound to kinematics.
- `f <motor>` — feedback request; single-line reply `<pos> <vel>`.
- `u <motor>` — feed the watchdog **without** changing the setpoint.
  Confirmed on real 0.5.1 hardware (ODrive v3.6) that this works fine
  regardless of firmware version, so it is not version-gated.
- `r <property>` / `w <property> <value>` — single-line reply (`r`) or no
  reply (`w`, verified by a follow-up `r`).
- `ss` (save — **reboots and re-enumerates USB on firmware ≥ 0.5.2**,
  per ODrive's CHANGELOG), `se` (erase — also reboots), `sr` (reboot),
  `sc` (clear errors).

### Request/response matching

Only `r` and `f` ever produce a reply; `p`/`v`/`c`/`t`/`w`/`u`/`ss`/`se`/`sr`/`sc`
never do. Because ODrive processes ASCII lines strictly in the order
received and replies (when it replies at all) in that same order, a
**FIFO queue of pending requests** is sufficient and correct — no
sequence numbers are needed (unlike the native protocol). Every line the
transport sends that expects a reply pushes a `PendingRequest(kind,
completion, deadline)` onto a deque; every received line pops the head
and completes it. This lets `p`-stream lines interleave freely with
occasional `r`/`f`/config-write traffic without any special-casing.

Resync policy: on a checksum mismatch or a request timeout, flush the RX
buffer, send a marker read (`r vbus_voltage`), and discard incoming lines
until one parses as a plausible float with a valid checksum (if
checksums are enabled). Three consecutive resync failures declare the
link lost and enter the reconnect state machine below.

### Reactor integration

- **RX**: `reactor.register_fd(serial.fileno(), rx_callback)` on a
  non-blocking `pyserial` handle. This gives byte-level latency (no
  polling delay) for time-sensitive reads — in particular the `f`/Iq
  polling used for sensorless homing trigger detection, where added
  latency directly becomes homing position error.
- **TX**: a `reactor.register_timer` firing at `sample_period` (default
  5 ms / 200 Hz) drives the setpoint stream and feeds the watchdog; on
  each tick, after sending the setpoint line, at most one queued command
  line (a `w`, `r`, or system command) is also sent, so that a bulk
  config dump or property browse from the web UI cannot starve motion
  setpoints.
- This mirrors `palette2.py`'s reactor-timer-driven host-serial model,
  with the RX side upgraded to `register_fd` for lower latency (palette2
  polls on a 0.1 s timer, which is fine for its filament-load use case
  but too coarse for homing-trigger timestamping here).
- A dedicated writer thread (`register_async_callback`) was considered
  and rejected as a first cut: measured USB Full-Speed scheduling jitter
  dominates over reactor-timer jitter at 200 Hz, so a thread buys little.
  It is documented as a fallback if `get_status().streaming.jitter_ms`
  proves problematic on heavily loaded single-board computers.

### Connection lifecycle

States: `DISCONNECTED → PROBING → CONFIGURING → READY →` (`LOST` or
`REBOOT_PENDING`) `→ PROBING → ...`

- **Device identification.** `serial:` accepts either a
  `/dev/serial/by-id/...` or a `/dev/serial/by-path/...` path — the
  latter is explicitly documented and recommended for clone boards,
  which are known to ship with **missing or duplicated USB serial
  numbers** (VID:PID `1209:0D32`), which breaks by-id udev rules when
  more than one clone is attached. An optional `serial_number:` (hex) is
  cross-checked against `r serial_number` after connecting *only if*
  that read returns a non-zero, parseable value — a clone that reports
  `0` or garbage there is logged as a warning and the check is skipped
  rather than treated as an error, since we cannot distinguish "clone
  doesn't implement this" from "wrong device" with certainty.
- **PROBING**: drain any stale RX data, `r vbus_voltage` as a sanity
  check, then `r fw_version_major/minor/revision` and
  `r hw_version_major/minor` to select a firmware-version overlay (see
  the tolerance layer), then read `axis0.current_state` /
  `axis1.current_state` and the four error registers per axis.
- **CONFIGURING**: push every Klipper-owned volatile ODrive setting
  (control mode, input mode + filter bandwidth, velocity/current limits,
  watchdog timeout, gains if set in `printer.cfg`) and verify each with a
  follow-up read. This runs on every connect — including after a
  save-triggered reboot — so a clone whose NVM was erased, or that never
  had these values persisted, still converges to a working configuration
  from `printer.cfg` alone.
- **REBOOT_PENDING**: entered by `ODRIVE_SAVE_CONFIG` (`ss`, which
  reboots on firmware ≥ 0.5.2) or `ODRIVE_REBOOT` (`sr`, which always
  reboots). The transport closes its handle immediately after sending
  the command (rather than waiting for a response that will never come
  once the device resets), then polls for the device path to reappear
  every 0.5 s for up to `reconnect_timeout` (default 15 s). On success,
  runs the full PROBING → CONFIGURING sequence again, and — because
  encoder position estimates and any un-persisted calibration state do
  not survive a reboot — clears the homed state of any kinematics rail
  bound to this board (mirrors Klipper's own behavior when an MCU
  restarts mid-print).
- **LOST** (unexpected `EOF`/`EIO`, or a run of resync failures with no
  pending `ss`/`sr`): if any axis on this board is a **bound, homed
  kinematics axis with queued toolhead activity**, this is a
  motion-integrity failure and Klippy shuts down
  (`printer.invoke_shutdown`), exactly as an unresponsive MCU would.
  Otherwise (board not yet bound to kinematics, or idle), this follows
  `palette2.py`'s graceful-degradation model: log, mark the board
  disconnected, expose that in `get_status`, and — if
  `auto_reconnect: True` (default) — keep retrying in the background
  without interrupting anything else Klippy is doing.
- Connection is attempted automatically once at `klippy:ready`, but a
  failure there is **not fatal to Klippy startup** — `ODRIVE_CONNECT` is
  also registered with `when_not_ready=True` so a user can plug in the
  board and connect after the fact. This, plus the device-less test
  requirement below, is why the whole module must tolerate never having
  a live device.

## Motion streaming design

### Sampling and unit conversion

On each TX tick, the target sample time is:

```
print_time = board.estimated_print_time(reactor.monotonic()) + latency_compensation
```

`estimated_print_time` delegates to the primary MCU's clock sync (as
`ODriveBoard.estimated_print_time` does), so ODrive axes and ordinary
MCU-driven steppers share exactly the same timebase — essential for
coordinated multi-axis moves where some axes are steppers and others are
ODrive motors.

`latency_compensation` (config, default `0.010` s) intentionally samples
**ahead of "now"**, compensating for USB transfer time, ASCII parse time,
and the ODrive's own `POS_FILTER` group delay, so that by the time the
setpoint physically reaches the motor's control loop it corresponds to
the *intended* instant rather than one that has already passed. This is
exactly why the sampler must read the **live**, not-yet-finalized trapq
queue (`trapq_extract_pending`, above) — the position it needs is one
that Klipper has already planned but has not yet reached.

The sample time is clamped to the current flush frontier (`sample_horizon`,
advanced by `ODriveStepper.generate_steps(flush_time)` on every toolhead
flush cycle, exactly the callback a real stepper uses to generate steps).
If the requested `print_time` is ahead of the horizon (toolhead has
nothing more planned yet, or is momentarily behind on flushing), the last
computed setpoint is held — the `p` line is still sent every tick, so the
watchdog stays fed regardless.

Given a sampled cartesian position (for the fraction of the axis this
stepper represents, via the same itersolve `calc_position_from_coord`
used for step generation), conversion to a turns setpoint is:

```
turns = axis_direction * position_mm / rotation_distance + odrive_offset_turns
```

`rotation_distance` is the same familiar config option every Kalico rail
already uses; `axis_direction` (`odrive_direction: 1|-1`) replaces the
role a `dir_pin` inversion would normally play. `odrive_offset_turns` is
recomputed (see below) whenever Klipper's notion of position is
redefined without the physical motor moving — homing completion, `SET_KINEMATIC_POSITION`, etc.

**Velocity feed-forward is always sent** (the second field of `p`),
computed analytically from the same trapq segment
(`v = start_v + accel * move_time`, exactly `DumpTrapQ`'s formula) —
this is what lets a fairly modest 200 Hz position-only stream still
track a fast move well, because the ODrive's own control loop is told
the intended velocity between setpoints rather than having to infer it
by differentiating a staircase of positions.

### Sample rate

Default `sample_period: 0.005` (200 Hz per motor). At 200 Hz, a two-motor
board sends roughly 200 × 2 × ~28 bytes/line ≈ 11 KB/s — trivial for USB
Full-Speed CDC. The option accepts 2–20 ms; rates above roughly 400–500 Hz
are permitted but documented as increasingly jitter-sensitive, per the
[ODrive team's own findings](https://discourse.odriverobotics.com/t/decreasing-latency/1463)
that USB bulk-transfer scheduling, not the ODrive firmware, is the
limiting factor for setpoint-streaming latency on this hardware.

### Input shaping

Kalico's `[input_shaper]` mechanism has zero effect on an ODrive-driven
rail. It works by swapping a stepper's `stepper_kinematics` for a
wrapper (`klippy/chelper/kin_shaper.c`) whose `calc_position_cb`
convolves the *original* kinematics' position at several time offsets
— but that convolution is only ever invoked from
`itersolve_gen_steps_range` (`klippy/chelper/itersolve.c`), the MCU
step-generation loop, which this module never calls into (see
`odrv_stepper.py`'s module docstring: no stepcompress sink, no
`itersolve_generate_steps`). `SetpointStreamer` samples the trapq
directly instead.

`shaper_type`/`shaper_freq`/`damping_ratio` on the rail section
(`[stepper_x]`, etc.) reimplement the same convolution as a host-side
pre-filter over the setpoint stream: on each TX tick, instead of one
`_sample_trapq(print_time)` call, one call per shaper impulse at
`print_time + mean_t - t_i`, weighted by each impulse's normalized
amplitude — exactly `kin_shaper.c`'s `init_shaper`/`shift_pulses`/
`calc_position` math (`mean_t` is the amplitude-weighted mean of the
impulse times, which is what makes the filter an identity transform
for constant-velocity motion rather than a pure delay), with impulse
coefficients reused directly from `shaper_defs.py` rather than
reimplemented. Both position *and* velocity feed-forward are shaped
this way (shaping is linear, so shaping position and differentiating
commutes with shaping velocity directly).

This is a **per-axis, ODrive-only** option, separate from the global
`[input_shaper]` section (which still applies normally to any
MCU-stepper axes on the same printer) — an ODrive axis is not
necessarily part of the same X/Y resonance system `[input_shaper]`
assumes, so shaping it independently, per motor, is the more natural
fit here.

### ODrive-side mode

At CONFIGURING time this module sets:

- `<axis>.controller.config.control_mode = 3` (`CONTROL_MODE_POSITION_CONTROL`)
- `<axis>.controller.config.input_mode = 3` (`INPUT_MODE_POS_FILTER`) by
  default, with `<axis>.controller.config.input_filter_bandwidth` set from
  the `filter_bandwidth` config option (default `0.5 / sample_period`,
  i.e. 100 Hz at the default 200 Hz stream rate) — smoothing between
  discrete setpoints without materially softening the commanded
  dynamics. `input_mode: passthrough` is available in config for streams
  at ≥ ~400 Hz where filtering is unnecessary and its added phase lag is
  undesirable. `INPUT_MODE_TRAP_TRAJ` is used only by the standalone
  `ODRIVE_AXIS_MOVE` command (a single on-device move to a destination),
  never while an axis is bound to a kinematics trapq.
- `circular_setpoints` left **off** — linear printer axes have travel
  ranges well within float32 turns precision; this option exists in
  ODrive firmware primarily for continuously-rotating axes (e.g. paired
  with step/dir), which is out of scope here.

### Idle, homing, and position resynchronization

- **Idle** (armed, no toolhead activity): the stream continues at a
  slower `idle_feed_period` (default 100 ms), resending the last
  setpoint — this keeps the watchdog fed and the motor holding position
  without the TX overhead of full-rate streaming while nothing is moving.
- **`set_position` / homing completion**: whenever kinematics calls
  `rail.set_position(coord)` (itersolve reset, no physical motion
  implied), `odrive_offset_turns` is recomputed so that the *next*
  streamed setpoint equals the *last actually-sent* setpoint even though
  Klipper's internal coordinate for this axis just changed —
  the same invariant a real stepper's MCU position offset preserves
  across a homing-triggered `set_position`.
- **Drip moves** (used during homing): no special-casing is required.
  `toolhead.drip_move` advances the flush frontier in small increments
  close to real time either way, and the sampler already tolerates a
  moving frontier. When an endstop trigger truncates the drip move, the
  subsequent `toolhead.set_position(haltpos)` call re-syncs the offset
  exactly as above. If a cache lookup misses because a segment was
  truncated mid-sample, the sampler falls back to `trapq_extract_old`
  (history) for that one query, then holds if that also misses.
- **Underrun / USB stall**: if a TX tick fails outright (write error or
  timeout) three times in a row (`stall_shutdown_ticks`, default 3)
  *while the axis has queued toolhead motion*, Klippy shuts down — the
  same reasoning as the LOST connection-state policy above, since a
  stalled setpoint stream during active motion is a position-integrity
  failure. The ODrive's own watchdog (fed only by successfully
  transmitted lines) is the hardware-side backstop if the host process
  itself dies outright: the motor disarms out of closed-loop control
  rather than continuing to chase a stale setpoint indefinitely.

### Sent-sample ring buffer

Every setpoint actually transmitted is recorded as `(print_time,
position_mm)` in a small per-axis ring buffer (~4 seconds of history at
the default rate). This buffer is the ODrive-axis analog of a real
stepper's MCU step queue, and backs three things: `get_past_mcu_position`
(distance-moved bookkeeping, used by Kalico's rehoming logic),
homing-trigger position attribution (below), and the host-side
following-error check (see [Safety design](#safety-design)). A `set_position`
call writes a discontinuity marker into the buffer so interpolation never
bridges across a coordinate redefinition.

## Homing design

Homing method is selected per-rail via the ordinary `endstop_pin:`
option, exactly as with any other Kalico stepper:

### 1. Physical endstop on a Klipper MCU pin

`endstop_pin: PC1` (a normal MCU pin) is fully supported and is the
**recommended default when a physical switch is available**, because MCU
firmware timestamps the trigger with hardware precision, giving better
homing accuracy than any host-side detection method.

This requires a `PhysicalEndstopWrapper` because the real trigger-sync
mechanism (`MCU_trsync.add_stepper`, `klippy/mcu.py:187,298`) identifies
steppers to stop by their firmware `oid` — which an `ODriveStepper` does
not have. The wrapper delegates `home_start`/`home_wait`/`query_endstop`
to the real `MCU_endstop`, but keeps ODrive steppers **out of** the
trsync's stepper list (tracking them separately so `get_steppers()` still
returns them for position-attribution purposes). Motion still halts
correctly on trigger: the trigger completes the `drip_completion` object
that `HomingMove.homing_move` waits on regardless of which steppers are
attached to the trsync hardware (`klippy/extras/homing.py:100-160`), which
ends the drip move and therefore the ODrive's streamed motion at the
same instant as any MCU-driven axes in the same homing move.

### 2. Sensorless (position-error / current-spike) virtual endstop

`endstop_pin: odrive_axis_x_motor:virtual_endstop` — for setups with no
physical switch wired to the ODrive or the host MCU. `ODriveVirtualEndstop`
registers a pin chip named `odrive_axis_<name>` (the same
`ppins.register_chip` pattern `TMCVirtualPinHelper` uses,
`klippy/extras/tmc.py:608-629`) so this syntax resolves.

During the homing move, a fast poll loop (reactor timer at
`homing_poll_period`, default 10 ms) requests `f <motor>` and
`r axisN.motor.current_control.Iq_measured`, and declares a trigger when
either:

- `|commanded_position − pos_estimate| > homing_pos_error_threshold` (mm,
  default 0.5 mm), or
- `|Iq_measured| > homing_current_threshold` (amps; `0`, the default,
  disables this check and relies on position error alone)

for `homing_trigger_count` (default 2) consecutive samples. A reduced
`homing_current` (config option, analogous to TMC's homing-current
reduction) is applied for the duration of the move via the standard
`homing:homing_move_begin`/`homing:homing_move_end` events — the same
hook `tmc.py` uses to lower driver current during sensorless homing.

Trigger-time attribution: `trigger_print_time = board.estimated_print_time(rx_time)
- trigger_latency`, where `rx_time` is the reactor time the triggering
`f` reply line arrived and `trigger_latency` (config, default 0.005 s)
approximates half the poll period plus ASCII round-trip time.
`HomingMove` then resolves the halt position via
`stepper.get_past_mcu_position(trigger_print_time)` against the sent-sample
ring buffer — the identical mechanism a real stepper uses
(`klippy/extras/homing.py:32-90`), so the standard overshoot/`trig_pos`
math in `homing.py` requires no changes.

This rail should set `use_sensorless_homing: True` (the same option TMC
sensorless-homing rails use) so Kalico's rehoming logic
(`moved_less_than_dist`, `min_home_dist`, `homing_elapsed_distance_tolerance`
in `klippy/extras/homing.py`) treats it identically to a TMC sensorless
rail — no changes needed there either, since it already keys off that
flag rather than the endstop's concrete type.

### 3. Encoder index search (pre-arm step, not a G28 method)

`ODRIVE_INDEX_SEARCH AXIS=x_motor` runs on-device state 6
(`AXIS_STATE_ENCODER_INDEX_SEARCH`) for encoders with an index/Z pulse
(`encoder.config.use_index: True`), and gates `ODRIVE_ARM` on it having
succeeded. It establishes a repeatable absolute reference each power-up
but is not itself a homing-to-an-endstop operation, so it is exposed as
its own command rather than folded into `G28`.

### 4. On-device homing (state 11) — convenience command only

ODrive firmware ≥ 0.5.2 supports its own homing state using GPIO
min/max-endstop configuration. This is exposed as a convenience command
(`ODRIVE_DEVICE_HOME AXIS=...`, which runs state 11, waits for `IDLE`,
reads `pos_estimate`, and instructs the user to follow up with
`SET_KINEMATIC_POSITION`), but is **not** wired up as a `G28` backend:
it moves the motor entirely outside Klipper's motion/trapq model, so
none of the standard trigger-time attribution, rehoming, or drip-move
machinery applies to it.

### Step/dir as a documented alternative

Nothing in this design prevents wiring an ODrive axis for **step/dir**
input instead of USB-streamed position control (0.5.x supports GPIO
step/dir input, renamed `axis.config.enable_step_dir` →
`step_dir_active` on firmware 0.5.6). In that case, `[stepper_x]` would
use ordinary `step_pin`/`dir_pin` options and a real MCU, while
`[odrive_axis]` is used purely for the browser-based calibration and
tuning workflow described below, with the USB link idle during actual
motion. This is out of scope to implement as a first-class mode, but the
calibration/config gcode command set is identical either way, so it
requires no separate design.

## Configuration and calibration UX

### Full config reference

```ini
[odrive drive0]
serial: /dev/serial/by-id/usb-ODrive...   # required; by-path also accepted (clone-friendly)
serial_number:                             # optional hex; verified only if the device reports a non-zero value
use_checksums: True
auto_reconnect: True
reconnect_timeout: 15.0                    # seconds to wait for USB re-enumeration after save/reboot
sample_period: 0.005                       # setpoint stream period, seconds (0.002-0.02)
latency_compensation: 0.010                # seconds, forward-sampling offset
error_poll_period: 0.5                     # round-robin error/vbus/thermal poll period, seconds
watchdog_timeout: 1.0                      # seconds; 0 disables (strongly discouraged)
vbus_min: 10.0                             # volts; refuse to arm below this, error if it collapses while armed
vbus_max: 26.0
idle_feed_period: 0.1                      # seconds, reduced-rate idle setpoint hold
stall_shutdown_ticks: 3                    # consecutive failed TX ticks during motion before shutdown

[odrive_axis x_motor]
odrive: drive0
axis: 0                                    # 0 (M0) or 1 (M1)
# --- motor ---
pole_pairs:                                # required, integer
torque_constant:                           # required; 8.27 / KV
current_lim: 20.0
calibration_current: 10.0
motor_type: high_current                   # high_current | gimbal
# --- encoder ---
encoder_cpr:                               # required; counts per revolution (4x quadrature)
encoder_use_index: False
encoder_bandwidth: 1000.0
# --- controller ---
pos_gain: 20.0
vel_gain: 0.16
vel_integrator_gain: 0.32
vel_limit:                                 # default: derived from the bound rail's max_velocity
input_mode: pos_filter                     # pos_filter | passthrough
filter_bandwidth:                          # default: 0.5 / sample_period
anticogging_enabled: False                 # apply the calibrated cogging-torque compensation map, once calibrated
# --- safety ---
enable_thermistor: False                   # ODrive-side motor thermistor, if wired
motor_temp_limit: 90.0
following_error: 1.0                       # mm; host-side check against the sent-sample ring buffer; 0 disables

[stepper_x]
odrive_axis: x_motor                       # replaces step_pin / dir_pin / microsteps entirely
rotation_distance: 40.0                    # mm per ODrive-motor revolution
odrive_direction: 1                        # 1 | -1, replaces dir_pin polarity
virtual_steps_per_rotation: 4000           # resolution of the virtual "mcu position" used for bookkeeping
endstop_pin: odrive_axis_x_motor:virtual_endstop   # or a normal MCU pin, or another rail's endstop for shared homing
homing_current: 5.0                        # amps; sensorless homing only
homing_pos_error_threshold: 0.5            # mm; sensorless homing only
homing_current_threshold: 0                # amps; 0 = position-error trigger only
homing_trigger_count: 2
homing_poll_period: 0.010
# position_min / position_max / position_endstop / homing_speed / second_homing_speed /
# homing_retract_dist / min_home_dist / use_sensorless_homing / homing_accel: standard rail options, unchanged
```

### GCode command set

All commands are mux'd on `ODRIVE=<name>` (board-level) or
`AXIS=<name>` (motor-level), following the `SET_TMC_FIELD`/`DUMP_TMC`
mux-command convention in `klippy/extras/tmc.py`.

| Command | Key parameters | Behavior |
|---|---|---|
| `ODRIVE_CONNECT` | `ODRIVE=` | (Re)connect and run the PROBING/CONFIGURING sequence. Registered `when_not_ready=True`. |
| `ODRIVE_DISCONNECT` | `ODRIVE=` | Disarm all axes on this board, close the serial port. |
| `ODRIVE_STATUS` | `[ODRIVE=]` | Print firmware/hardware version, serial number, vbus, per-axis state/temperature/errors, streaming statistics. |
| `ODRIVE_CALIBRATE` | `AXIS=` `TYPE=full\|motor\|encoder_offset\|index` | Runs the calibration wizard (below). |
| `ODRIVE_ANTICOGGING_CALIBRATE` | `AXIS=` `[SAVE=1]` | Runs the on-device anti-cogging calibration sweep (below). Requires the axis to already be armed with motor and encoder calibration complete. |
| `ODRIVE_INDEX_SEARCH` | `AXIS=` | Runs state 6, waits, reports success/failure. |
| `ODRIVE_ARM` / `ODRIVE_DISARM` | `AXIS=` | Enter/leave `CLOSED_LOOP_CONTROL`, subject to the interlocks below. |
| `ODRIVE_CLEAR_ERRORS` | `ODRIVE=` | `sc`, then re-reads and reports all error registers. |
| `ODRIVE_ERRORS` | `ODRIVE=` `[VERBOSE=1]` | Decodes every error bitfield to human-readable names via the version-specific decode table. |
| `ODRIVE_READ` | `ODRIVE=` `PROPERTY=` | `r <property>`, reports the value. |
| `ODRIVE_WRITE` | `ODRIVE=` `PROPERTY=` `VALUE=` | `w <property> <value>`, then verifies by read-back. Refuses to write any property the streamer owns (`input_pos`, `requested_state`, control/input mode) on an armed, bound axis unless `FORCE=1`. |
| `ODRIVE_DUMP_CONFIG` | `ODRIVE=` | Walks the version's known-property list and prints the full config tree. |
| `ODRIVE_TUNE` | `AXIS=` `[POS_GAIN=]` `[VEL_GAIN=]` `[VEL_INTEGRATOR_GAIN=]` `[FILTER_BANDWIDTH=]` `[CURRENT_LIM=]` `[VEL_LIMIT=]` `[SAVE=1]` | Live-applies any given gains/limits. `SAVE=1` additionally stages them into `printer.cfg` via `configfile.set`, surfacing Mainsail's normal `SAVE_CONFIG` prompt. |
| `ODRIVE_SAVE_CONFIG` | `ODRIVE=` | `ss` — persists to ODrive NVM, manages the resulting reboot/reconnect. Warns that bound kinematics axes become unhomed. |
| `ODRIVE_ERASE_CONFIG` | `ODRIVE=` `CONFIRM=1` | `se` — wipes ODrive NVM, manages reboot. |
| `ODRIVE_REBOOT` | `ODRIVE=` | `sr`, manages reconnect. |
| `ODRIVE_AXIS_MOVE` | `AXIS=` `POS=`\|`TURNS=` `[VEL=]` | Standalone on-device trapezoidal move (`TRAP_TRAJ`). `POS` (mm) requires a bound rail; `TURNS` moves in raw motor turns. Refused if the axis is bound to a homed kinematics rail with a print in progress. |
| `ODRIVE_WATCHDOG` | `AXIS=` `ENABLE=` `[TIMEOUT=]` | Diagnostic override of watchdog behavior for one motor axis. |

### Calibration wizard (`ODRIVE_CALIBRATE`)

Runs synchronously inside the gcode command handler, using
`reactor.pause`-based polling (the same style existing manual-calibration
flows use), so every phase transition is visible as it happens in the
Mainsail/Fluidd console:

1. **Preconditions.** Board connected, vbus within
   `[vbus_min, vbus_max]`, axis currently `IDLE`, toolhead idle. The
   watchdog is disabled for the duration of calibration — calibration
   states legitimately run longer than a typical `watchdog_timeout`, and
   a trip mid-calibration would abort it with a confusing error.
2. **Push parameters** (`calibration_current`, `pole_pairs`,
   `encoder_cpr`, etc.) from `printer.cfg`.
3. **Run the requested state** (`requested_state = 3/4/6/7` depending on
   `TYPE=`), polling `current_state` at 10 Hz and printing a status line
   on every state transition — e.g. `// ODrive x_motor: motor
   calibration... OK  R=0.041 ohm  L=17.2 uH`.
4. **Check every error register** on completion; on any non-zero error,
   decode it and abort with an actionable message rather than a raw
   bitmask.
5. **On success**, set `motor.config.pre_calibrated = 1` and/or
   `encoder.config.pre_calibrated = 1` as appropriate, print the measured
   electrical parameters, and prompt the user to run
   `ODRIVE_SAVE_CONFIG` to persist them to the ODrive's own NVM.
6. Stage a Klipper-side breadcrumb (`configfile.set(section,
   "calibrated_at", ...)`, and any tuning values already applied) so
   Mainsail's `save_config_pending` banner appears in the same gesture —
   mirroring the UX of `PROBE_CALIBRATE` and `ANGLE_CALIBRATE`.

The persistence model is deliberately split: **electrical calibration
results** (motor resistance/inductance, encoder offset) live in the
ODrive's own NVM via `ODRIVE_SAVE_CONFIG`; **declared parameters and
tuning** (everything in `[odrive_axis]`/`[stepper_x]`) live in
`printer.cfg` and are re-pushed at every connect. This means a clone
board whose NVM gets erased (or one being provisioned for the first
time) fully recovers after a single wizard run, without needing its NVM
save to have succeeded or persisted correctly — a reasonable hedge given
clone firmware's documented inconsistency around save/reboot behavior.

### Anti-cogging calibration (`ODRIVE_ANTICOGGING_CALIBRATE`)

Cogging torque (the periodic ripple from motor magnetic reluctance) is
compensated by a separate, optional on-device calibration: a sweep over
one full mechanical rotation that builds a per-encoder-position torque
compensation map. Unlike motor/encoder calibration, this runs *after*
`ODRIVE_ARM` — the axis must already be in closed-loop control, not
`IDLE` — since the sweep is itself a closed-loop-controlled move.

1. **Preconditions.** Axis armed (`CLOSED_LOOP_CONTROL`), motor and
   encoder already `pre_calibrated`, no active errors, toolhead not
   mid-print on a bound kinematics rail (same guard as
   `ODRIVE_AXIS_MOVE`, since the sweep's motion would otherwise collide
   with real printer motion on that axis).
2. **Trigger the sweep** — the exact on-device mechanism (a dedicated
   `requested_state`, or setting a `controller.config.anticogging.*`
   flag while already closed-loop, depending on firmware) needs to be
   confirmed against the actual firmware source per supported version
   during implementation, following the same
   [firmware-version tolerance layer](#firmware-version-tolerance-layer)
   approach used for every other version-sensitive behavior in this
   spec — do not assume a single mechanism works unmodified across the
   whole 0.5.1–0.5.6 range. Poll for completion the same way
   `ODRIVE_CALIBRATE` does, printing progress as it advances through the
   rotation.
3. **Check every error register** on completion, decoding any non-zero
   result the same way `ODRIVE_CALIBRATE` does.
4. **On success**, set `controller.config.anticogging.pre_calibrated = 1`
   and, if `anticogging_enabled: True` in `[odrive_axis]` (or `SAVE=1`
   was passed), enable `controller.config.anticogging.enabled` so the
   map is actually applied to subsequent motion.
5. The resulting map plus `anticogging.enabled` are ODrive-NVM-resident
   state, in the same bucket as motor/encoder calibration results —
   `SAVE=1` follows up with `ODRIVE_SAVE_CONFIG` exactly as
   `ODRIVE_CALIBRATE`'s wizard prompts for it, rather than inventing a
   separate save path.

### Safety interlocks (UX-visible)

- `ODRIVE_ARM` is refused, with an explanatory error, when: any error
  register is non-zero, the motor/encoder are not `pre_calibrated`
  (overridable with `FORCE=1`), vbus is out of range, or an index search
  is configured as required but has not succeeded.
- Homing an ODrive rail (`G28`) verifies the axis is armed and
  error-free before starting; if not, it raises a `command_error`
  naming the exact fix (e.g. "run ODRIVE_ARM AXIS=x_motor first").
- `ODRIVE_ANTICOGGING_CALIBRATE` is refused, with an explanatory error,
  unless the axis is already armed with motor and encoder calibration
  complete, and — like `ODRIVE_AXIS_MOVE` — refused if the axis is bound
  to a homed kinematics rail with active toolhead motion queued.
- If the streamer ever observes commanded motion (non-zero trapq
  velocity) for a bound rail whose axis is not armed, this is a
  motion-correctness violation and triggers `printer.invoke_shutdown`
  immediately, rather than silently doing nothing while Klipper believes
  the axis is moving.

## Safety design

- **Watchdog.** Enabled only while armed (`axis.config.enable_watchdog =
  1`, `watchdog_timeout` from config, minimum enforced value 0.25 s,
  default 1.0 s), fed by every successfully transmitted `p` line
  (streaming) or `u` line (idle-hold on firmware where `u` exists).
  Disabled during calibration (see above) and at disarm.
- **Error polling.** A round-robin poll cycles through
  `axisN.error`/`motor.error`/`encoder.error`/`controller.error`, plus
  `vbus_voltage` and available temperature sensors, completing a full
  cycle within `error_poll_period` (default 0.5 s ⇒ a full cycle in a few
  seconds). During homing and while printing, the bound axis's `axis.error`
  is additionally checked at the same fast rate as the `f`/Iq polling
  used for sensorless homing.
- **Error → response mapping.** If the errored axis is currently a
  **bound, homed kinematics axis with active toolhead motion**, any
  non-zero error, a watchdog trip, an unexpected disarm, or loss of the
  USB link is treated exactly like an unresponsive MCU: `printer.invoke_shutdown`.
  If the axis is idle, unbound, or not yet homed, the same conditions
  instead mark it errored/unhomed and emit a `respond_info` warning —
  consistent with treating a disconnected, not-yet-relied-upon ODrive as
  an optional peripheral (the same philosophy Kalico's `is_non_critical`
  MCU support and `palette2.py`'s graceful-disconnect behavior embody),
  while treating a live motion axis with the same seriousness as core
  motion hardware.
- **Following error.** Each `f` poll compares the reported
  `pos_estimate` against the sent-sample ring buffer's expected position
  at the corresponding time (adjusted for `latency_compensation`); a
  deviation beyond `following_error` (mm; `0` disables the check)
  follows the same shutdown-or-warn rule above. The check is
  automatically suspended for the first 0.5 s after arming and throughout
  calibration, when transient deviation is expected.
- **Klipper lifecycle events.** `klippy:shutdown` and `klippy:disconnect`
  handlers best-effort command every axis to `requested_state = 1`
  (`IDLE`) directly (bypassing the normal command queue, since Klippy may
  be mid-shutdown) and stop streaming. The `stepper_enable:motor_off`
  event disarms bound axes; kinematics already clears homed state on
  motor-off through its existing mechanism, requiring no ODrive-specific
  change there.
- **Thermal / bus voltage.** FET temperature above 80°C is a warning;
  above 95°C is treated as an error. Vbus outside
  `[vbus_min, vbus_max]` while armed is an error — this catches both PSU
  brownout and (on boards with a brake resistor) regenerative-braking
  bus overvoltage faults.

## Status and webhooks surface

This section specifies exactly what the future Mainsail integration
(see `ODrive_Mainsail_Integration_Plan.md`) will consume; it is written
now because it must be considered part of the module's public contract.

### `[odrive drive0].get_status(eventtime)`

```python
{
    "connected": True,
    "state": "ready",  # probing | configuring | ready | reboot_pending | lost | disconnected
    "fw_version": "0.5.4",
    "hw_version": "3.6-56V",
    "serial_number": "3559316F3237",
    "vbus_voltage": 24.1,
    "errors": [],  # board-level aggregate of all axis error names
    "streaming": {"rate": 200.0, "jitter_ms": 0.8, "underruns": 0, "tx_bytes": 184320},
    "capabilities": {"watchdog_feed_cmd": True, "device_homing": True, "endstop_gpio": True},
}
```

(Pending-`SAVE_CONFIG` state is deliberately not duplicated here — the
web UI already sources its save-config banner from the `configfile`
object's own status, which `ODRIVE_TUNE SAVE=1` and the calibration
wizard stage into via `configfile.set`.)

### `[odrive_axis x_motor].get_status(eventtime)`

```python
{
    "axis_state": "closed_loop_control",
    "armed": True,
    "calibrated": True,
    "pre_calibrated_motor": True,
    "pre_calibrated_encoder": True,
    "index_found": True,
    "errors": {"axis": [], "motor": [], "encoder": [], "controller": []},
    "pos_estimate": 12.34,   # mm
    "vel_estimate": 0.0,     # mm/s
    "pos_error": 0.012,      # mm, commanded vs. estimate
    "iq_measured": 1.2,
    "iq_setpoint": 1.1,
    "fet_temp": 34.5,
    "motor_temp": None,
    "pos_gain": 20.0, "vel_gain": 0.16, "vel_integrator_gain": 0.32,
    "filter_bandwidth": 100.0, "current_lim": 20.0, "vel_limit": 30.0,
}
```

Both objects are ordinary Klipper printer objects (the module returns
them from `load_config`/`load_config_prefix`, which registers them
automatically), so Moonraker's `printer.objects.subscribe` sees them with
no extra work — the same mechanism that already surfaces TMC driver
status and `exclude_object` state to Mainsail.

### Webhooks endpoints

- `odrive/telemetry` — a `bulk_sensor.BatchBulkHelper`-based mux endpoint
  (`add_mux_endpoint`, keyed on `axis`, following `klippy/extras/angle.py`'s
  pattern) streaming `(time, input_pos, pos_estimate, vel_estimate,
  iq_measured)` tuples at the fast poll rate. The fast rate is only
  requested from the ODrive **while at least one client is subscribed**
  (the `BatchBulkHelper` start/stop callbacks gate this), so a browser
  tuning scope does not impose continuous extra USB load when nobody is
  looking at it.
- `odrive/property_read` — a plain `register_mux_endpoint` for one-shot
  property reads (`{"property": "axis0.motor.config.current_lim"}` →
  `{"value": ...}`), letting a future panel populate form fields without
  a gcode round-trip for every field.
- Everything that **mutates** ODrive or Klipper state deliberately stays
  gcode-only (`ODRIVE_WRITE`, `ODRIVE_TUNE`, calibration, arm/disarm,
  save/erase/reboot) rather than being exposed as a webhook action —
  keeping every state change visible and auditable in the console
  history, consistent with how Kalico treats other consequential
  hardware actions (e.g. `PROBE_CALIBRATE`, `SET_TMC_FIELD`).

## Firmware-version tolerance layer

Implemented entirely in `properties.py`, so no other file ever contains a
raw ODrive property path or a hardcoded enum value.

- **Version probe.** At connect time, `r fw_version_major/minor/revision`
  is read. A recognized version selects a known overlay. An unreadable,
  zero, or otherwise implausible response (observed on some clones with
  hacked version strings) falls back to treating the device as the
  earliest supported version (0.5.1) and **runtime-probing** every
  version-sensitive property individually, rather than trusting any
  claimed version string.
- **Property path map.** Canonical internal names are mapped to their
  per-version dotted path, capturing known renames — e.g.
  `axis.config.enable_step_dir` → `axis.config.step_dir_active` on
  firmware 0.5.6 (confirmed in the ODrive CHANGELOG) — plus per-version
  differences in the error bitfield encodings between the 0.5.1 and
  ≥ 0.5.4 lines (to be pinned down precisely against each firmware tag
  during implementation of Phase 1, and encoded as literal per-version
  tables here rather than assumed).
- **Feature/behavior probing beats trusted version strings.** For
  instance, watchdog-feed-only (`u`) support is probed by attempting it
  and checking for an error response rather than assumed from the
  reported version, and the save-triggers-reboot behavior is handled
  reactively (the reconnect FSM starts polling for re-enumeration
  immediately after `ss` regardless of reported version, since a clone
  may or may not actually implement the version-gated reboot behavior it
  claims to support).
- **Feature gates.** `has_watchdog_feed_cmd`, `has_device_homing`,
  `has_endstop_gpio_config`, each resolved as a version-based default and
  then confirmed (or corrected) by a runtime probe, are exposed in
  `get_status()["capabilities"]` for both diagnostic purposes and so a
  future Mainsail panel can hide controls for unsupported features
  rather than let a user hit an opaque error.

## Testing and phasing

### Device-less test coverage

Kalico's regression test harness (`test/klippy/*.test` + `*.cfg`, run via
`scripts/test_klippy.py` in fileinput/debug mode) never has a real serial
device attached. `test/klippy/odrive.cfg` + `odrive.test` exercise:
config parsing (both required and optional options, and error paths for
missing required options like `pole_pairs`), the `LookupMultiRail`
dispatch producing an `ODriveRail`/`ODriveStepper` pair, pin-chip
registration for `virtual_endstop`, full command registration (every
`ODRIVE_*` command runs and returns a clean "not connected" response
rather than hanging or crashing), and `get_status()` shape. `G28` on an
ODrive rail is **not** exercised in this test — homing genuinely requires
device interaction. Pure logic — checksum framing, the version/property
map resolution, the trapq-sampling math (against synthetic
`pull_move`-shaped tuples), and ring-buffer interpolation — is unit
tested independent of the reactor/serial machinery entirely.

### Mock-ODrive harness

`scripts/odrive_mock.py`: opens a pty pair and speaks ASCII 0.5.x on one
end, with configurable firmware-version and "clone quirk" flags (no
reported serial number, `u` command unsupported, renamed properties). It
simulates a simple second-order axis model (so `f` returns physically
plausible position/velocity estimates that track streamed setpoints),
scripted calibration-state timelines, on-demand error injection, and
`ss`/`se`-triggered reboot behavior (closing and reopening the pty after
a delay, to exercise the reconnect FSM). Running Klippy against
`serial: /dev/pts/N` pointed at this harness is the vehicle for full
integration testing of connect → calibrate → stream → home, in CI,
without real hardware — including deliberately misbehaving "clone"
configurations.

### Implementation roadmap

Each phase is independently landable and independently testable using
the harnesses above.

| Phase | Scope | Result |
|---|---|---|
| **1 — Comms core** | `[odrive]` board object; transport (`register_fd` RX + timer TX, checksums, FIFO matching); version probe/shim skeleton; reconnect FSM including managed `ss` reboot; `ODRIVE_CONNECT/DISCONNECT/STATUS/READ/WRITE/ERRORS/CLEAR_ERRORS/DUMP_CONFIG/SAVE_CONFIG/ERASE_CONFIG/REBOOT`; board `get_status`; mock harness; `odrive.cfg`/`.test`. | Full property read/write/browse access to a live ODrive from the Mainsail console — the "no odrivetool needed" baseline. |
| **2 — Calibration & tuning UX** | `[odrive_axis]` object; config push/read-back; `ODRIVE_CALIBRATE` wizard; `ODRIVE_TUNE` (+ `configfile.set` staging); `ODRIVE_INDEX_SEARCH`; error-bitfield decode tables; axis `get_status`. | A clone board can be fully calibrated and tuned from a browser with no prior setup. |
| **3 — Manual motion & safety** | Arm/disarm with interlocks; watchdog policy; `ODRIVE_AXIS_MOVE`; idle-hold feeding; error-poll → shutdown/warn mapping; shutdown/disconnect lifecycle handlers. | An armed motor can be jogged safely under Klipper's supervision, independent of kinematics. |
| **4 — Kinematics integration** | `trapq_extract_pending` chelper addition; `stepper.py` `LookupMultiRail` hook; `ODriveStepper`/`ODriveRail`; `SetpointStreamer` (sampling, feed-forward, latency compensation, ring buffer, underrun policy); `set_position` resync; `motor_off` handling. | An ODrive axis participates in ordinary Klipper moves/jogging as a kinematics rail (homing not yet wired). |
| **5 — Homing** | `ODriveVirtualEndstop` (sensorless), `PhysicalEndstopWrapper`, homing-current events, trigger-time attribution, Kalico rehoming-logic compatibility, following-error monitor. | `G28` works on ODrive axes via either homing method. |
| **6 — Telemetry & docs** | `odrive/telemetry` bulk endpoint, `odrive/property_read`, streaming statistics, `Config_Reference.md`/`G-Codes.md`/`Kalico_Additions.md` entries, latency-measurement helper (`ODRIVE_TUNE ... MEASURE_LATENCY=1`). | Panel-ready data surface (see the Mainsail integration plan); documentation complete. |

Dependency order is strictly 1 → 2 → 3 → 4 → 5; phase 6 can begin
alongside phase 2 and continues through the rest.

## Key design decisions (summary)

1. **Rail-level duck-type via a small `stepper.LookupMultiRail` hook**,
   not virtual step/dir pins or new per-kinematics ODrive classes —
   every kinematics module works unmodified, and no full-MCU emulation
   is required (rejected: virtual step pins require faking an entire MCU
   transport interface for no benefit; rejected: bespoke kinematics
   classes don't scale across cartesian/corexy/delta/etc.).
2. **A real itersolve `stepper_kinematics` with no `stepcompress`
   attached** gives correct, kinematics-agnostic coordinate math for
   free (rejected: reimplementing each kinematics' coordinate transform
   by hand in the ODrive module).
3. **A new `trapq_extract_pending` chelper function** to sample the
   live, in-flight move queue — required because `trapq_extract_old`
   (used by `motion_report.py`) only sees *completed* moves, which fails
   for any move still in progress, including every homing move
   (rejected: monkey-patching `trapq_append`; rejected: sampling only
   completed history, which fails for the common case of in-flight
   moves).
4. **ASCII-over-CDC as the sole transport**, with checksums and FIFO
   request matching, `register_fd` for RX and a `register_timer` for TX
   at a 200 Hz default (rejected: the native Fibre protocol, whose
   endpoint-ID/CRC16 addressing is tied to the exact firmware build —
   the opposite of what clone tolerance requires; rejected as a
   near-term option: a dedicated writer thread, deferred as a documented
   fallback).
5. **Two homing paths** — a `PhysicalEndstopWrapper` for a real switch on
   a Klipper MCU pin, and a sensorless position-error/current virtual
   endstop for setups with none — cover the practical cases without
   trying to wire ODrive's own on-device homing state into `G28`
   (rejected: using ODrive state 11 as the `G28` backend, since it
   operates entirely outside Klipper's trapq/attribution model).
6. **Tiered failure response**: an ODrive is treated as an optional,
   gracefully-degrading peripheral right up until the moment it becomes
   a bound, homed, actively-moving kinematics axis — at which point any
   fault is treated with the same severity as an unresponsive MCU
   (rejected: uniform "always shut down" — unnecessarily fragile for
   idle/unbound boards; rejected: uniform "always just warn" — unsafe
   once the board is actually driving a homed axis mid-print).
