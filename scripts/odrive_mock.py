#!/usr/bin/env python3
"""Mock ODrive v3.6 ASCII-protocol device for device-less integration testing.

Speaks the exact line-oriented ASCII wire protocol described in
docs/ODrive_Implementation_Spec.md ("Protocol choice: ASCII over
USB-CDC" and "Request/response matching") over a pty, so
klippy/extras/odrive/*.py can be driven through its full
connection-lifecycle / calibration / arm / streaming state machine with
no real ODrive attached. This is the "Mock-ODrive harness" called for by
that spec's "Testing and phasing" -> "Device-less test coverage" section:
Kalico's `test/klippy/*.test` regression harness never has a real serial
device attached, so `G28` on an ODrive rail (and calibration, arming,
save/reboot) is never exercised there. This script fills that gap.

Usage:

    python3 scripts/odrive_mock.py --port /tmp/odrive_mock_pty

The script creates a pty pair and maintains a symlink at --port that
always points at the pty's current secondary (client-facing) device.
Point a real `[odrive]` section at that stable path:

    [odrive drive0]
    serial: /tmp/odrive_mock_pty
    baud: 115200

    [odrive_axis x_motor]
    odrive: drive0
    axis: 0
    pole_pairs: 7
    torque_constant: 0.041
    encoder_cpr: 4000

    [stepper_x]
    odrive_axis: x_motor
    rotation_distance: 40
    position_min: 0
    position_max: 200
    endstop_pin: odrive_axis_x_motor:virtual_endstop
    homing_speed: 5

Then run Klippy (e.g. `python3 klippy/klippy.py your-printer.cfg`)
against that config and drive it from the gcode console --
`ODRIVE_CONNECT`, `ODRIVE_CALIBRATE AXIS=x_motor TYPE=full`,
`ODRIVE_ARM AXIS=x_motor`, `ODRIVE_SAVE_CONFIG ODRIVE=drive0`, `G28 X`,
etc. -- exactly as against real hardware, including the
save/erase/reboot-triggered USB re-enumeration (simulated here by
briefly removing and recreating the --port symlink against a fresh pty,
so the transport's `os.path.exists(serial_path)` reconnect-poll loop in
klippy/extras/odrive/__init__.py sees the same "gone, then back" signal
a real USB re-enumeration produces).

If --port is omitted, only the raw /dev/pts/N path is printed (usable
for a one-shot manual test), but a simulated `ss`/`se`/`sr` reboot will
open a *new* pty with a different path -- there is no stable path for
the client to reconnect to, so the reconnect FSM will not converge.
--port is required to exercise that behavior faithfully.

Run with --fast during interactive development to shrink the simulated
calibration/reboot delays to a few tens of milliseconds; the default
delays (a couple of real seconds each) are closer to what real ODrive
firmware takes, which matters if you're specifically testing timeout/
polling behavior.

Copyright (C) 2026  Kalico contributors

This file may be distributed under the terms of the GNU GPLv3 license.
"""

from __future__ import annotations

import argparse
import math
import os
import pty
import re
import select
import signal
import sys
import termios
import time
import tty

# Axis state values and the calibration-state -> "what firmware sets on
# success" mapping, mirrored from klippy/extras/odrive/properties.py
# (AXIS_STATE_*, CALIBRATE_TYPE_STATES) rather than imported directly.
# Importing that module means importing the "klippy" package, which
# pulls in cffi/chelper/etc. at import time (klippy/__init__.py ->
# klippy/printer.py -> klippy/mcu.py -> klippy/chelper) -- this script
# is meant to run with zero dependencies beyond the standard library, so
# the handful of integer constants it actually needs are copied here
# instead. Keep these in sync with properties.py if it ever changes.
AXIS_STATE_IDLE = 1
AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3
AXIS_STATE_MOTOR_CALIBRATION = 4
AXIS_STATE_ENCODER_INDEX_SEARCH = 6
AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

# On successful completion of one of these calibration states, real
# firmware transitions back to IDLE and sets the listed property(ies).
CALIBRATION_COMPLETION_FLAGS = {
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE: (
        "motor.config.pre_calibrated",
        "encoder.config.pre_calibrated",
    ),
    AXIS_STATE_MOTOR_CALIBRATION: ("motor.config.pre_calibrated",),
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION: ("encoder.config.pre_calibrated",),
    AXIS_STATE_ENCODER_INDEX_SEARCH: ("encoder.index_found",),
}

AXIS_PROPERTY_RE = re.compile(r"^axis(\d+)\.(.+)$")

# Main loop tick: caps how long select() blocks, so the position
# simulation advances smoothly and scheduled events (calibration
# completion, reboot) fire promptly without busy-looping.
TICK = 0.02


def _format_value(value):
    if isinstance(value, bool):
        return str(int(value))
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return "%.6f" % (value,)
    return str(value)


def _coerce_value(existing, raw):
    # "w" always carries a value as plain text; cast it to match the
    # type already in the property tree so later "r"/logic (eg.
    # int(float(readback)) in axis.py) keeps working the same way it
    # would against a real device's typed property store.
    if isinstance(existing, bool):
        try:
            return bool(int(float(raw)))
        except ValueError:
            return existing
    if isinstance(existing, int):
        try:
            return int(float(raw))
        except ValueError:
            return existing
    if isinstance(existing, float):
        try:
            return float(raw)
        except ValueError:
            return existing
    return raw


def build_default_properties(
    num_axes, serial_number, fw_version, hw_version, vbus
):
    """Seed a fresh property tree for a `num_axes`-axis ODrive v3.6.

    Paths and defaults are cross-referenced against
    klippy/extras/odrive/axis.py's `push_config`/`poll_errors_and_telemetry`
    (the `self.prop(suffix)` -> "axisN.<suffix>" pattern) and
    klippy/extras/odrive/__init__.py's board-level reads, so every path
    the real module ever sends an "r"/"w" for exists here.
    """
    props = {
        "vbus_voltage": vbus,
        "fw_version_major": fw_version[0],
        "fw_version_minor": fw_version[1],
        "fw_version_revision": fw_version[2],
        "hw_version_major": hw_version[0],
        "hw_version_minor": hw_version[1],
        "serial_number": serial_number,
    }
    for i in range(num_axes):
        p = "axis%d." % (i,)
        props.update(
            {
                p + "current_state": AXIS_STATE_IDLE,
                p + "requested_state": AXIS_STATE_IDLE,
                p + "error": 0,
                p + "config.watchdog_timeout": 0.0,
                p + "config.enable_watchdog": 0,
                p + "motor.error": 0,
                p + "motor.config.pole_pairs": 7,
                p + "motor.config.calibration_current": 10.0,
                p + "motor.config.current_lim": 20.0,
                p + "motor.config.torque_constant": 0.041,
                p + "motor.config.motor_type": 0,
                p + "motor.config.pre_calibrated": 0,
                p + "motor.current_control.Iq_measured": 0.0,
                p + "motor.fet_thermistor.temperature": 32.0,
                p + "encoder.error": 0,
                p + "encoder.config.cpr": 4000,
                p + "encoder.config.use_index": 0,
                p + "encoder.config.bandwidth": 1000.0,
                p + "encoder.config.pre_calibrated": 0,
                p + "encoder.index_found": 0,
                p + "controller.error": 0,
                p + "controller.config.pos_gain": 20.0,
                p + "controller.config.vel_gain": 0.16,
                p + "controller.config.vel_integrator_gain": 0.32,
                p + "controller.config.vel_limit": 30.0,
                p + "controller.config.control_mode": 3,
                p + "controller.config.input_mode": 3,
                p + "controller.config.input_filter_bandwidth": 100.0,
                p + "controller.input_pos": 0.0,
            }
        )
    return props


class AxisMotion:
    """First-order-lag position simulation for one motor.

    Simplification (documented, per the spec's "a simple model is
    fine"): position exponentially approaches whatever the last "p"/"t"
    setpoint was, with time constant `tau`; velocity is derived from the
    resulting position delta. This is not a physically accurate
    second-order motor/load model, but it is enough to make a
    subsequent "f" reply reflect real elapsed motion instead of a static
    number, which is what the calibration/arm/streaming state machine
    actually reads.
    """

    def __init__(self):
        self.pos_turns = 0.0
        self.vel_turns = 0.0
        self.target_turns = 0.0
        self.last_update = time.monotonic()

    def advance(self, now, tau):
        dt = now - self.last_update
        self.last_update = now
        if dt <= 0.0:
            return
        if tau > 0.0:
            alpha = 1.0 - math.exp(-dt / tau)
        else:
            alpha = 1.0
        new_pos = self.pos_turns + (self.target_turns - self.pos_turns) * alpha
        self.vel_turns = (new_pos - self.pos_turns) / dt
        self.pos_turns = new_pos

    def reset(self):
        self.pos_turns = 0.0
        self.vel_turns = 0.0
        self.target_turns = 0.0
        self.last_update = time.monotonic()


class MockOdrive:
    def __init__(self, args):
        self.args = args
        self.num_axes = args.axes
        self.calib_delay = 0.05 if args.fast else args.calib_delay
        self.reboot_delay = 0.05 if args.fast else args.reboot_delay
        self.motion_tau = args.motion_tau
        self.verbose = args.verbose

        self.props = build_default_properties(
            self.num_axes,
            args.serial_number,
            args.fw_version,
            args.hw_version,
            args.vbus,
        )
        self.motion = {i: AxisMotion() for i in range(self.num_axes)}
        self._events = []  # list of [deadline, callback]
        self.rx_buf = b""
        self.master_fd = None
        self.slave_path = None
        self.running = True

    # -- pty lifecycle --------------------------------------------------
    def _open_new_pty(self):
        master_fd, slave_fd = pty.openpty()
        try:
            tty.setraw(master_fd)
        except termios.error:
            pass
        self.master_fd = master_fd
        self.slave_path = os.ttyname(slave_fd)
        os.close(slave_fd)
        if self.args.port:
            self._update_symlink()

    def _update_symlink(self):
        link = self.args.port
        tmp = "%s.tmp%d" % (link, os.getpid())
        if os.path.lexists(tmp):
            os.unlink(tmp)
        os.symlink(self.slave_path, tmp)
        os.replace(tmp, link)

    def _close_pty(self, remove_symlink):
        if self.master_fd is not None:
            try:
                os.close(self.master_fd)
            except OSError:
                pass
            self.master_fd = None
        if remove_symlink and self.args.port and os.path.islink(self.args.port):
            try:
                os.unlink(self.args.port)
            except OSError:
                pass

    def start(self):
        self._open_new_pty()
        self._log("odrive_mock: listening on %s" % (self.slave_path,))
        if self.args.port:
            self._log(
                "odrive_mock: stable symlink %s -> %s (point 'serial:' here)"
                % (self.args.port, self.slave_path)
            )
        else:
            self._log(
                "odrive_mock: no --port given; a simulated reboot will open"
                " a new, differently-numbered pty (see --help)"
            )

    def stop(self):
        self._close_pty(remove_symlink=True)

    def _log(self, msg):
        print(msg, file=sys.stderr)
        sys.stderr.flush()

    # -- scheduler --------------------------------------------------------
    def _schedule(self, delay, callback):
        self._events.append([time.monotonic() + delay, callback])

    def _cancel_all_events(self):
        self._events = []

    def _process_events(self, now):
        due = [e for e in self._events if e[0] <= now]
        for e in due:
            self._events.remove(e)
        for e in due:
            e[1]()
        if not self._events:
            return TICK
        nxt = min(e[0] for e in self._events)
        return max(0.0, min(TICK, nxt - now))

    # -- motion simulation --------------------------------------------------
    def _advance_motion(self, now):
        for m in self.motion.values():
            m.advance(now, self.motion_tau)

    # -- protocol: checksums ------------------------------------------------
    @staticmethod
    def _checksum(body):
        cs = 0
        for ch in body.encode("ascii", errors="replace"):
            cs ^= ch
        return cs

    def _handle_raw_line(self, raw):
        try:
            text = raw.decode("ascii", errors="replace").strip("\r\n \t")
        except Exception:
            return
        if not text:
            return
        had_checksum = False
        if "*" in text:
            body, _, cs_str = text.rpartition("*")
            try:
                expected = int(cs_str)
            except ValueError:
                body, expected = text, None
            if expected is not None:
                if self._checksum(body) != expected:
                    if self.verbose:
                        self._log("<< %r (bad checksum, dropped)" % (text,))
                    return
                text = body
                had_checksum = True
        if self.verbose:
            self._log("<< %s" % (text,))
        reply = self._dispatch(text)
        if reply is not None:
            self._send_reply(reply, had_checksum)

    def _send_reply(self, body, had_checksum):
        line = body
        if had_checksum:
            line = "%s*%d" % (body, self._checksum(body))
        if self.verbose:
            self._log(">> %s" % (line,))
        if self.master_fd is None:
            return
        try:
            os.write(self.master_fd, (line + "\n").encode("ascii"))
        except OSError:
            pass

    # -- protocol: command dispatch ------------------------------------------
    def _dispatch(self, text):
        parts = text.split()
        if not parts:
            return None
        cmd = parts[0]
        if cmd == "r":
            return self._cmd_read(parts)
        if cmd == "f":
            return self._cmd_feedback(parts)
        if cmd == "w":
            self._cmd_write(parts)
            return None
        if cmd in ("p", "v", "c", "t", "u"):
            self._cmd_motion(cmd, parts)
            return None
        if cmd in ("ss", "se", "sr"):
            self._cmd_reboot(cmd)
            return None
        if cmd == "sc":
            self._cmd_clear_errors()
            return None
        if self.verbose:
            self._log("   (unrecognized command %r, ignored)" % (cmd,))
        return None

    def _cmd_read(self, parts):
        if len(parts) != 2 or parts[1] not in self.props:
            return "invalid property"
        return _format_value(self.props[parts[1]])

    def _cmd_write(self, parts):
        if len(parts) < 3:
            return
        path = parts[1]
        if path not in self.props:
            # Real firmware silently ignores writes to unknown paths too
            # (there is no reply to a "w" either way); it just never
            # takes effect, which a follow-up "r" would reveal.
            return
        self.props[path] = _coerce_value(self.props[path], parts[2])
        m = AXIS_PROPERTY_RE.match(path)
        if m and m.group(2) == "requested_state":
            axis_idx = int(m.group(1))
            if axis_idx < self.num_axes:
                self._handle_requested_state_write(axis_idx, self.props[path])

    def _cmd_motion(self, cmd, parts):
        if len(parts) < 2:
            return
        try:
            motor = int(float(parts[1]))
        except ValueError:
            return
        m = self.motion.get(motor)
        if m is None:
            return
        # "p"/"t" carry a new setpoint; "v"/"c"/"u" are velocity/torque/
        # watchdog-only commands the streamer never uses while bound to
        # kinematics (see the spec's protocol framing section) -- accepted
        # here (so they don't error) but not modeled positionally.
        if cmd in ("p", "t") and len(parts) >= 3:
            try:
                m.target_turns = float(parts[2])
            except ValueError:
                pass

    def _cmd_feedback(self, parts):
        if len(parts) != 2:
            return "invalid property"
        try:
            motor = int(float(parts[1]))
        except ValueError:
            return "invalid property"
        m = self.motion.get(motor)
        if m is None:
            return "invalid property"
        return "%.6f %.6f" % (m.pos_turns, m.vel_turns)

    def _cmd_clear_errors(self):
        for i in range(self.num_axes):
            p = "axis%d." % (i,)
            for suffix in (
                "error",
                "motor.error",
                "encoder.error",
                "controller.error",
            ):
                self.props[p + suffix] = 0

    def _cmd_reboot(self, cmd):
        self._cancel_all_events()
        self._close_pty(remove_symlink=True)
        self._log(
            "odrive_mock: received '%s' -- simulating USB re-enumeration"
            " (offline for %.2fs)" % (cmd, self.reboot_delay)
        )
        self._schedule(self.reboot_delay, lambda: self._finish_reboot(cmd))

    def _finish_reboot(self, cmd):
        for i in range(self.num_axes):
            p = "axis%d." % (i,)
            self.props[p + "current_state"] = AXIS_STATE_IDLE
            self.props[p + "requested_state"] = AXIS_STATE_IDLE
            self.motion[i].reset()
        if cmd == "se":
            # Erase: NVM wiped, so calibration/tuning revert to factory
            # defaults too, not just the transient state above.
            self.props = build_default_properties(
                self.num_axes,
                self.args.serial_number,
                self.args.fw_version,
                self.args.hw_version,
                self.args.vbus,
            )
        self.rx_buf = b""
        self._open_new_pty()
        self._log("odrive_mock: back online at %s" % (self.slave_path,))

    # -- calibration state machine -------------------------------------------
    def _handle_requested_state_write(self, axis_idx, new_state):
        p = "axis%d." % (axis_idx,)
        self.props[p + "current_state"] = new_state
        if new_state in CALIBRATION_COMPLETION_FLAGS:
            self._schedule(
                self.calib_delay,
                lambda: self._finish_calibration(axis_idx, new_state),
            )

    def _finish_calibration(self, axis_idx, state):
        p = "axis%d." % (axis_idx,)
        # A newer request (eg. disarm, or a second calibration) may have
        # superseded this one already; only land if we're still where we
        # expect to be.
        if self.props.get(p + "current_state") != state:
            return
        for flag in CALIBRATION_COMPLETION_FLAGS[state]:
            self.props[p + flag] = 1
        self.props[p + "current_state"] = AXIS_STATE_IDLE
        self.props[p + "requested_state"] = AXIS_STATE_IDLE
        if self.verbose:
            self._log(
                "odrive_mock: axis%d calibration (state %d) complete"
                % (axis_idx, state)
            )

    # -- main loop ------------------------------------------------------------
    def run(self):
        # Deliberately no signal.signal() calls here: run() is meant to be
        # usable from a background thread too (eg. tests that want a live
        # mock without a subprocess), and Python only allows registering
        # signal handlers from the main thread. main() below wires up
        # Ctrl-C/SIGTERM for the standalone-script case; a caller driving
        # MockOdrive directly can just set `.running = False` instead.
        try:
            while self.running:
                now = time.monotonic()
                self._advance_motion(now)
                timeout = self._process_events(now)
                fds = [self.master_fd] if self.master_fd is not None else []
                r, _, _ = select.select(fds, [], [], timeout)
                if self.master_fd is not None and self.master_fd in r:
                    try:
                        data = os.read(self.master_fd, 4096)
                    except OSError:
                        data = b""
                    if data:
                        self.rx_buf += data
                        while b"\n" in self.rx_buf:
                            line, self.rx_buf = self.rx_buf.split(b"\n", 1)
                            self._handle_raw_line(line)
        finally:
            self.stop()


def _version_tuple(text, expected_parts):
    parts = [int(x) for x in text.split(".")]
    while len(parts) < expected_parts:
        parts.append(0)
    return tuple(parts[:expected_parts])


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Mock ODrive v3.6 ASCII-protocol device for exercising"
            " klippy/extras/odrive without real hardware. See the module"
            " docstring (or --help) for a full usage example."
        )
    )
    parser.add_argument(
        "--port",
        default=None,
        help=(
            "Stable path to create/maintain as a symlink to the mock's"
            " current pty (recommended: point [odrive]'s 'serial:' at"
            " this path). Without it, only the raw /dev/pts/N path is"
            " printed and simulated reboots will not be reconnectable."
        ),
    )
    parser.add_argument(
        "--axes",
        type=int,
        choices=(1, 2),
        default=2,
        help="Number of simulated motor axes (default: 2)",
    )
    parser.add_argument(
        "--fast",
        action="store_true",
        help="Use short (~50ms) simulated calibration/reboot delays for"
        " quick iteration, overriding --calib-delay/--reboot-delay",
    )
    parser.add_argument(
        "--calib-delay",
        type=float,
        default=2.0,
        metavar="SECONDS",
        help="Simulated calibration duration (default: 2.0s; ignored if"
        " --fast)",
    )
    parser.add_argument(
        "--reboot-delay",
        type=float,
        default=2.0,
        metavar="SECONDS",
        help="Simulated USB re-enumeration delay after ss/se/sr (default:"
        " 2.0s; ignored if --fast)",
    )
    parser.add_argument(
        "--motion-tau",
        type=float,
        default=0.05,
        metavar="SECONDS",
        help="Time constant of the first-order-lag position simulation"
        " used to answer 'f' (default: 0.05s)",
    )
    parser.add_argument(
        "--vbus",
        type=float,
        default=24.0,
        metavar="VOLTS",
        help="Simulated bus voltage (default: 24.0)",
    )
    parser.add_argument(
        "--serial-number",
        default="3559316F3237",
        metavar="HEX",
        help="Simulated serial number, hex (default: 3559316F3237, the"
        " example from the implementation spec)",
    )
    parser.add_argument(
        "--fw-version",
        default="0.5.4",
        metavar="MAJOR.MINOR.REV",
        help="Simulated firmware version (default: 0.5.4)",
    )
    parser.add_argument(
        "--hw-version",
        default="3.6",
        metavar="MAJOR.MINOR",
        help="Simulated hardware version (default: 3.6)",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Log every protocol line sent/received to stderr",
    )
    args = parser.parse_args(argv)
    try:
        args.serial_number = int(args.serial_number, 16)
    except ValueError:
        parser.error("--serial-number must be a hex string")
    try:
        args.fw_version = _version_tuple(args.fw_version, 3)
        args.hw_version = _version_tuple(args.hw_version, 2)
    except ValueError:
        parser.error(
            "--fw-version/--hw-version must be dotted-integer versions"
        )
    return args


def main(argv=None):
    args = parse_args(argv)
    mock = MockOdrive(args)

    def _stop(signum, frame):
        mock.running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    mock.start()
    mock.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
