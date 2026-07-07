# Chamber LED (Particle Boron) — Implementation Spec

This document describes `[chamber_led]`: host-side support for a
Particle Boron 404X running custom firmware, connected over USB, that
drives a chamber LED strip (WS2812/NeoPixel) on the Boron's own GPIOs.
Klipper talks to the Boron over USB-CDC using a small ASCII
request/response protocol; the Boron does the actual LED bit-banging.

This is a much smaller feature than
[ODrive support](ODrive_Implementation_Spec.md), and reuses that
module's host-side serial I/O idiom (see "Transport design" below), but
has none of its calibration/arming/error-polling complexity — there is
nothing to calibrate, and a lighting peripheral going offline is not
safety-critical the way a motion axis is.

## Goals

- Plug a Boron running the chamber LED firmware into the host over USB.
- Make it behave like any other Kalico-native LED strip:
  `SET_LED LED=<name> ...` and `SET_LED_TEMPLATE` work out of the box
  via `klippy/extras/led.py`'s `LEDHelper`, the same way
  `klippy/extras/neopixel.py` wraps it.
- Optionally drive the chamber light automatically off print state
  (printing / paused / complete / error / idle), configured per-color
  in `printer.cfg`, with no scripting required.
- Expose a couple of direct gcode commands (`CHAMBER_LED_OFF`,
  `CHAMBER_LED_STATUS`) for users who want to bypass the LED
  abstraction and talk to the controller directly.

Non-goals: RGBW/individually-addressable-pixel control (the wire
protocol sets one color for the whole strip), a FIFO command queue or
checksums (this is a short, trusted point-to-point USB link, not a
noisy multi-drop bus), auto-reconnect polling (if the Boron drops off
mid-print the chamber light simply stays at its last commanded color
until Klipper is restarted — not worth the complexity for a
non-critical peripheral).

## Wire protocol

Plain ASCII, one command per line terminated by `\n`, strictly
request/response — at most one command is ever in flight, and every
command produces exactly one reply line:

| Command | Reply |
|---|---|
| `PING` | `PONG` |
| `COLOR <r> <g> <b> <brightness>` (each 0-255) | `OK` |
| `OFF` | `OK` |
| `STATUS` | `STATUS mode=<off\|solid> r=<n> g=<n> b=<n> brightness=<n> uptime_ms=<n> free_mem=<n>` |
| anything unrecognized/invalid | `ERR <reason>` |

Measured round-trip latency on real hardware: median 0.99ms, max
1.66ms over 50 samples. There is no checksum framing (unlike the
ODrive's XOR-checksummed ASCII protocol) — a corrupt line on this short
USB-CDC link is vanishingly unlikely compared to a real RS-485 bus, and
the cost of getting it wrong is a wrong LED color, not a runaway motor.

## Transport design

`klippy/extras/chamber_led/transport.py` (`ChamberLedTransport`)
follows the same non-blocking-pyserial-driven-by-the-reactor idiom as
`klippy/extras/odrive/transport.py`: `reactor.register_fd` for RX,
`reactor.register_timer` for TX. Two things it deliberately does
*not* copy from the ODrive transport, because this protocol doesn't
need them:

- **No checksum framing.** See "Wire protocol" above.
- **No FIFO pending-request queue.** The protocol is strictly one
  request/response at a time, so a single `_PendingRequest` slot
  replaces the ODrive transport's `collections.deque` of in-flight
  requests. A caller that issues a second `query()` before the first
  resolves gets an immediate `None` (a caller bug, not a transport
  condition worth queueing for).

The TX timer is event-driven rather than free-running on a fixed
period: `send_line()` wakes it immediately
(`reactor.update_timer(timer, reactor.NOW)`) instead of waiting for the
next tick of a periodic poll, since a fixed-period TX loop would add
scheduling jitter on the order of the whole measured round-trip budget
for no benefit (there is no continuous stream to pace, unlike the
ODrive's setpoint streaming).

## Module layout

- `klippy/extras/chamber_led/transport.py` — `ChamberLedTransport`,
  as above.
- `klippy/extras/chamber_led/__init__.py` — `ChamberLED`, the
  `[chamber_led <name>]` config object. Responsibilities:
  - Owns the transport and the connection lifecycle
    (`klippy:ready` connects, `klippy:shutdown`/`klippy:disconnect`
    close the port — see `klippy/extras/odrive/__init__.py`'s
    `ODriveBoard` for the pattern this mirrors, minus the
    reconnect-polling and error-polling machinery that module needs
    for a motion-critical device).
  - Wraps a `led.LEDHelper` exactly like `neopixel.py` does, so
    `SET_LED`/`SET_LED_TEMPLATE` work unmodified.
  - Registers `CHAMBER_LED_OFF` and `CHAMBER_LED_STATUS` as
    `CHAMBER_LED=<name>`-keyed mux commands (mirroring how
    `ODRIVE_STATUS ODRIVE=<name>` is registered).
  - Optionally hooks `print_stats:start_printing`,
    `print_stats:paused_printing`, `print_stats:complete_printing`,
    `print_stats:error_printing`, and `print_stats:cancelled_printing`
    (see "Automatic color changes" below) to drive color changes.

## RGBW → wire protocol color mapping

`LEDHelper` hands back a 4-tuple `(red, green, blue, white)`, each
0.0-1.0. The `COLOR` wire command takes `<r> <g> <b> <brightness>`,
each 0-255, with no white channel — the physical LEDs are WS2812 RGB,
not RGBW. `_update_leds()` handles the mismatch two ways:

- **White channel**: folded additively into each of R/G/B (clamped to
  1.0) rather than silently dropped, so `SET_LED ... WHITE=<n>` (or a
  template that emits a white component) still does something
  reasonable instead of being a silent no-op.
- **Brightness**: the wire protocol's separate `brightness` field is
  always sent as `255`; overall intensity is carried entirely in the
  R/G/B values (matching how `neopixel.py` scales
  `led_state[i][c] * 255.0` directly). The `color_printing`/etc. config
  options (see below) accept an optional 4th "R,G,B,BRIGHTNESS" element
  purely as a config-time convenience — it's folded into the
  `LEDHelper` RGB floats before being handed to `_update_leds()`, so
  manual `SET_LED` calls and automatic print-state colors go through
  the exact same RGB→wire mapping.

## Automatic color changes

Five optional config options — `color_printing`, `color_paused`,
`color_complete`, `color_error`, `color_idle` — each an `"R,G,B"` or
`"R,G,B,BRIGHTNESS"` string (0-255 per element), in the same spirit as
`led.py`'s `initial_RED`/`initial_GREEN`/etc. options. None are
required; a user who only wants manual `SET_LED` control gets no
automatic behavior and `[print_stats]` isn't even loaded.

Each configured option is wired to one of `print_stats`'s events
(`klippy/extras/print_stats.py`):

| Config option | `print_stats` event |
|---|---|
| `color_printing` | `print_stats:start_printing` |
| `color_paused` | `print_stats:paused_printing` |
| `color_complete` | `print_stats:complete_printing` |
| `color_error` | `print_stats:error_printing` |
| `color_idle` | `print_stats:cancelled_printing` |

`print_stats` has no dedicated "idle" event, but a cancelled print
(`note_cancel()` → `print_stats:cancelled_printing`) is the transition
back to an idle chamber with nothing queued, so `color_idle` rides on
that event.

## Testing

No physical hardware is available in CI, so coverage is split across
three layers:

- `test/test_chamber_led_config.py` — unit tests for the module's pure
  config/reply-parsing helpers against a real `ConfigWrapper`, no
  printer object graph required.
- `scripts/chamber_led_mock.py` + `test/test_chamber_led_mock.py` — a
  small mock firmware that speaks the real wire protocol over a pty
  (mirroring `scripts/odrive_mock.py`'s approach, minus the
  calibration state machine that protocol needs and this one doesn't),
  driven by a real pyserial client to check the mock is a faithful
  protocol peer.
- `test/klippy/chamber_led.cfg` / `chamber_led.test` — the standard
  device-less Kalico klippy test (see `odrive.cfg`/`odrive.test`,
  `led.cfg`/`led.test`): the configured serial path doesn't exist, so
  the module never connects, exercising config parsing (including the
  `color_*` auto-color options), `SET_LED`/`SET_LED_TEMPLATE` wiring,
  and graceful not-connected handling of `CHAMBER_LED_STATUS`/
  `CHAMBER_LED_OFF`.

`scripts/chamber_led_mock.py` can also be run standalone against a
real `[chamber_led]` config for interactive testing:

```
python3 scripts/chamber_led_mock.py --port /tmp/chamber_led_mock_pty
```

```
[chamber_led chamber]
serial: /tmp/chamber_led_mock_pty
baud: 115200
```
