"""Config-time and command-time behavior tests for [odrive_axis]
(klippy/extras/odrive/axis.py).

Exercises the *real* ODriveAxis class (not a fake of it) against a real
klippy.configfile.ConfigWrapper backed by a real configparser, so option
parsing (required-vs-optional, minval/above checks, error messages) is
exactly what a live Klippy config load would do. Only the surrounding
Printer/board/gcode plumbing is faked, mirroring this module's existing
"no full Klippy printer object graph" philosophy (see test_odrive_repl.py).
"""

from __future__ import annotations

import configparser

import pytest

from klippy import configfile
from klippy import reactor as reactor_mod
from klippy.extras.odrive import axis as axis_mod
from klippy.extras.odrive import properties


class FakePins:
    def __init__(self):
        self.chips = {}

    def register_chip(self, name, chip):
        self.chips[name] = chip


class FakeGCode:
    def __init__(self):
        self.mux_commands = {}

    def register_mux_command(self, cmd, key, value, func, desc=None):
        self.mux_commands[(cmd, key, value)] = func


class FakeWebhooks:
    def __init__(self):
        self.mux_endpoints = {}

    def register_mux_endpoint(self, path, key, value, callback):
        self.mux_endpoints[(path, key, value)] = callback


class FakeTransport:
    """Records writes and echoes them back for the readback checks
    push_config() does (pole_pairs verify, calibrated_motor/_encoder
    probes), without needing a real serial device or reactor."""

    def __init__(self):
        self.writes = []
        self.props = {}
        self.read_sequences = {}
        self.fail_write_paths = set()

    def write_property(self, path, value):
        if path in self.fail_write_paths:
            raise IOError("simulated serial write failure for %s" % (path,))
        self.writes.append((path, value))
        self.props[path] = value

    def read_property_sync(self, path, timeout=2.0):
        seq = self.read_sequences.get(path)
        if seq:
            # Simulates an external, changing signal (e.g. shadow_count/
            # hall_state) -- one item consumed per read, repeating the
            # last item once exhausted so callers that poll longer than
            # the scripted sequence don't crash.
            value = seq.pop(0) if len(seq) > 1 else seq[0]
            return str(value)
        return str(self.props.get(path, 0))

    def query_sync(self, line, timeout=2.0):
        return "0.0 0.0"


class FakeBoard:
    def __init__(self, name="drive0", fw_version=None):
        self.name = name
        self.full_name = "odrive %s" % (name,)
        self.axes = {}
        self.transport = FakeTransport()
        self.sample_period = 0.005
        self.watchdog_timeout = 1.0
        self.idle_feed_period = 0.1
        self.connected = True
        self.vbus_voltage = None
        self.vbus_min = 10.0
        self.vbus_max = 26.0
        self.props = properties.PropertyMap(fw_version)

    def register_axis(self, axis):
        if axis.axis_index in self.axes:
            raise RuntimeError(
                "ODrive %s axis %d already used by '%s'"
                % (self.full_name, axis.axis_index, self.axes[axis.axis_index])
            )
        self.axes[axis.axis_index] = axis


class FakePrinter:
    def __init__(self, reactor=None):
        self._reactor = reactor if reactor is not None else object()
        self.event_handlers = []
        self.boards = {}
        self._objects = {
            "pins": FakePins(),
            "gcode": FakeGCode(),
            "webhooks": FakeWebhooks(),
        }

    def get_reactor(self):
        return self._reactor

    def lookup_object(self, name, default=configfile.sentinel):
        if name in self._objects:
            return self._objects[name]
        if default is not configfile.sentinel:
            return default
        raise KeyError(name)

    def load_object(self, config, name, default=configfile.sentinel):
        if name in self.boards:
            return self.boards[name]
        if default is not configfile.sentinel:
            return default
        raise KeyError(name)

    def register_event_handler(self, event, handler):
        self.event_handlers.append((event, handler))


class FakeGCmd:
    """Minimal stand-in for klippy.gcode.GCodeCommand."""

    def __init__(self, params=None):
        self.params = params or {}
        self.responses = []

    def get(self, name, default=None):
        return self.params.get(name, default)

    def get_int(self, name, default=None, **kwargs):
        return int(self.params.get(name, default))

    def get_float(self, name, default=None, **kwargs):
        return float(self.params.get(name, default))

    def error(self, msg):
        return Exception(msg)

    def respond_info(self, msg):
        self.responses.append(msg)


def make_config(section, options, printer):
    parser = configparser.ConfigParser()
    parser.add_section(section)
    for key, value in options.items():
        parser.set(section, key, str(value))
    return configfile.ConfigWrapper(printer, parser, {}, section)


def _build_axis(printer, extra_options, section="odrive_axis motor_a"):
    options = {
        "odrive": "drive0",
        "axis": 0,
        "pole_pairs": 7,
        "torque_constant": 0.0827,
    }
    options.update(extra_options)
    config = make_config(section, options, printer)
    return axis_mod.ODriveAxis(config)


def _printer_with_board(fw_version=None, reactor=None):
    printer = FakePrinter(reactor=reactor)
    board = FakeBoard(fw_version=fw_version)
    printer.boards["odrive drive0"] = board
    return printer, board


def test_axis_requires_encoder_cpr():
    printer, _ = _printer_with_board()

    with pytest.raises(configparser.Error, match="encoder_cpr"):
        _build_axis(printer, {}, section="odrive_axis no_cpr")


def test_axis_can_bind_a_rail():
    printer, _ = _printer_with_board()
    axis = _build_axis(printer, {"encoder_cpr": 8192})
    rail_config = make_config("stepper_x", {"odrive_axis": "motor_a"}, printer)

    with pytest.raises(configparser.Error, match="rotation_distance"):
        axis.lookup_rail(rail_config, True, None)


def test_push_config_writes_encoder_and_position_mode():
    printer, board = _printer_with_board()
    axis = _build_axis(printer, {"encoder_cpr": 8192})

    axis.push_config()

    written = dict(board.transport.writes)
    assert written[axis.prop("encoder.config.cpr")] == 8192
    assert (
        written[axis.prop("controller.config.control_mode")]
        == properties.CONTROL_MODE_POSITION_CONTROL
    )


def test_arm_requests_closed_loop_control():
    printer, board = _printer_with_board(reactor=reactor_mod.Reactor())
    axis = _build_axis(printer, {"encoder_cpr": 8192})
    axis.push_config()
    axis.calibrated_motor = True
    axis.calibrated_encoder = True

    axis.cmd_ODRIVE_ARM(FakeGCmd())

    written = dict(board.transport.writes)
    assert (
        written[axis.prop("requested_state")]
        == properties.AXIS_STATE_CLOSED_LOOP_CONTROL
    )


def test_arm_rejects_when_not_calibrated():
    printer, _ = _printer_with_board(reactor=reactor_mod.Reactor())
    axis = _build_axis(printer, {"encoder_cpr": 8192})
    axis.push_config()

    with pytest.raises(Exception, match="not calibrated"):
        axis.cmd_ODRIVE_ARM(FakeGCmd())


def test_encoder_mode_defaults_to_incremental():
    printer, _ = _printer_with_board()
    axis = _build_axis(printer, {"encoder_cpr": 8192})
    assert axis.encoder_mode == properties.ENCODER_MODE_INCREMENTAL


def test_encoder_mode_hall_choice():
    printer, _ = _printer_with_board()
    axis = _build_axis(printer, {"encoder_cpr": 42, "encoder_mode": "hall"})
    assert axis.encoder_mode == properties.ENCODER_MODE_HALL


def test_push_config_writes_encoder_mode():
    printer, board = _printer_with_board()
    axis = _build_axis(printer, {"encoder_cpr": 42, "encoder_mode": "hall"})

    axis.push_config()

    written = dict(board.transport.writes)
    assert (
        written[axis.prop("encoder.config.mode")]
        == properties.ENCODER_MODE_HALL
    )


def test_encoder_diagnose_flags_flat_signal():
    printer, board = _printer_with_board(reactor=reactor_mod.Reactor())
    axis = _build_axis(printer, {"encoder_cpr": 8192})
    board.transport.read_sequences[axis.prop("encoder.shadow_count")] = [0] * 20

    gcmd = FakeGCmd({"DURATION": 0.15})
    axis.cmd_ODRIVE_ENCODER_DIAGNOSE(gcmd)
    assert any("never changed" in r for r in gcmd.responses)


def test_encoder_diagnose_flags_clean_hall_signal():
    printer, board = _printer_with_board(reactor=reactor_mod.Reactor())
    axis = _build_axis(printer, {"encoder_cpr": 42, "encoder_mode": "hall"})
    # A real 3-hall commutation cycle: each step changes exactly one bit.
    clean_cycle = [1, 3, 2, 6, 4, 5] * 10
    board.transport.read_sequences[axis.prop("encoder.hall_state")] = (
        clean_cycle
    )

    gcmd = FakeGCmd({"DURATION": 0.15})
    axis.cmd_ODRIVE_ENCODER_DIAGNOSE(gcmd)
    assert any("looks clean" in r for r in gcmd.responses)


def test_encoder_diagnose_flags_noisy_hall_signal():
    printer, board = _printer_with_board(reactor=reactor_mod.Reactor())
    axis = _build_axis(printer, {"encoder_cpr": 42, "encoder_mode": "hall"})
    # Erratic, out-of-sequence jumps (including invalid 0/7 states) --
    # matches what a loose/noisy hall connection actually looked like on
    # real hardware.
    noisy = [
        5,
        1,
        3,
        5,
        6,
        2,
        0,
        4,
        7,
        6,
        4,
        5,
        3,
        6,
        5,
        1,
        3,
        1,
        5,
        4,
    ] * 3
    board.transport.read_sequences[axis.prop("encoder.hall_state")] = noisy

    gcmd = FakeGCmd({"DURATION": 0.15})
    axis.cmd_ODRIVE_ENCODER_DIAGNOSE(gcmd)
    assert any("noisy" in r for r in gcmd.responses)


def test_emergency_idle_survives_and_logs_a_failed_write(caplog):
    # on_emergency_idle() runs from ODriveBoard._handle_shutdown's loop
    # over every axis -- a failed write for one axis must not raise (or
    # it would abort disarming the *other* axes on the same board) and
    # must not be silently swallowed either, since a lost emergency-idle
    # write is exactly the kind of thing an operator needs visibility
    # into (see overnight/OVERNIGHT_LOG.md's e-stop review).
    printer, board = _printer_with_board(reactor=reactor_mod.Reactor())
    axis = _build_axis(printer, {"encoder_cpr": 8192})
    axis.push_config()
    axis.calibrated_motor = True
    axis.calibrated_encoder = True
    axis.cmd_ODRIVE_ARM(FakeGCmd())
    board.transport.fail_write_paths.add(axis.prop("requested_state"))

    axis.on_emergency_idle()

    assert axis.armed is False
    assert any(
        "failed to send emergency-idle" in r.message for r in caplog.records
    )
