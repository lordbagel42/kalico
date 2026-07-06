"""Config-time behavior tests for the "sensorless: True" debug path on
[odrive_axis] (klippy/extras/odrive/axis.py).

Sensorless mode drives ODrive's native back-EMF velocity estimation (no
encoder at all) just enough to spin an unencoded motor for wiring/
debugging -- see the module docstring in axis.py. These tests exercise
the *real* ODriveAxis class (not a fake of it) against a real
klippy.configfile.ConfigWrapper backed by a real configparser, so
option parsing (required-vs-optional, minval/above checks, error
messages) is exactly what a live Klippy config load would do. Only the
surrounding Printer/board/gcode plumbing is faked, mirroring this
module's existing "no full Klippy printer object graph" philosophy
(see test_odrive_repl.py).
"""

from __future__ import annotations

import configparser

import pytest

from klippy import configfile
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

    def write_property(self, path, value):
        self.writes.append((path, value))
        self.props[path] = value

    def read_property_sync(self, path, timeout=2.0):
        return str(self.props.get(path, 0))


class FakeBoard:
    def __init__(self, name="drive0"):
        self.name = name
        self.full_name = "odrive %s" % (name,)
        self.axes = {}
        self.transport = FakeTransport()
        self.sample_period = 0.005
        self.watchdog_timeout = 1.0
        self.connected = True

    def register_axis(self, axis):
        if axis.axis_index in self.axes:
            raise RuntimeError(
                "ODrive %s axis %d already used by '%s'"
                % (self.full_name, axis.axis_index, self.axes[axis.axis_index])
            )
        self.axes[axis.axis_index] = axis


class FakePrinter:
    def __init__(self):
        self._reactor = object()
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
    """Minimal stand-in for klippy.gcode.GCodeCommand -- just enough
    surface for the ODRIVE_CALIBRATE guard under test."""

    def __init__(self, params=None):
        self.params = params or {}
        self.responses = []

    def get(self, name, default=None):
        return self.params.get(name, default)

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


def _printer_with_board():
    printer = FakePrinter()
    board = FakeBoard()
    printer.boards["odrive drive0"] = board
    return printer, board


def test_sensorless_axis_does_not_require_encoder_cpr():
    printer, _ = _printer_with_board()

    axis = _build_axis(
        printer,
        {"sensorless": "True", "sensorless_pm_flux_linkage": "0.02"},
    )

    assert axis.sensorless is True
    assert axis.encoder_cpr is None


def test_sensorless_axis_without_pm_flux_linkage_errors_clearly():
    printer, _ = _printer_with_board()

    with pytest.raises(configparser.Error, match="sensorless_pm_flux_linkage"):
        _build_axis(printer, {"sensorless": "True"})


def test_non_sensorless_axis_still_requires_encoder_cpr():
    # Regression check: existing (non-sensorless) configs must keep
    # erroring exactly as before when encoder_cpr is missing.
    printer, _ = _printer_with_board()

    with pytest.raises(configparser.Error, match="encoder_cpr"):
        _build_axis(printer, {})


def test_sensorless_axis_rejects_rail_binding():
    printer, _ = _printer_with_board()
    axis = _build_axis(
        printer,
        {"sensorless": "True", "sensorless_pm_flux_linkage": "0.02"},
    )
    rail_config = make_config("stepper_x", {"odrive_axis": "motor_a"}, printer)

    with pytest.raises(configparser.Error, match="sensorless"):
        axis.lookup_rail(rail_config, True, None)


def test_non_sensorless_axis_can_still_bind_a_rail():
    # Regression check: the new guard must not affect ordinary
    # encoder-backed axes trying to bind (this only asserts we get past
    # the sensorless guard -- it will fail further in for other reasons
    # since rail_config lacks endstop_pin/rotation_distance, which is
    # unrelated to what's under test here).
    printer, _ = _printer_with_board()
    axis = _build_axis(printer, {"encoder_cpr": "8192"})
    rail_config = make_config("stepper_x", {"odrive_axis": "motor_a"}, printer)

    with pytest.raises(configparser.Error, match="rotation_distance"):
        axis.lookup_rail(rail_config, True, None)


def test_sensorless_push_config_skips_encoder_and_uses_velocity_mode():
    printer, board = _printer_with_board()
    axis = _build_axis(
        printer,
        {"sensorless": "True", "sensorless_pm_flux_linkage": "0.019"},
    )

    axis.push_config()

    written = dict(board.transport.writes)
    assert written[axis.prop("config.enable_sensorless_mode")] == 1
    assert (
        written[axis.prop("sensorless_estimator.config.pm_flux_linkage")]
        == 0.019
    )
    assert (
        written[axis.prop("controller.config.control_mode")]
        == properties.CONTROL_MODE_VELOCITY_CONTROL
    )
    assert (
        written[axis.prop("controller.config.input_mode")]
        == properties.INPUT_MODE_PASSTHROUGH
    )
    assert axis.prop("encoder.config.cpr") not in written
    assert axis.prop("encoder.config.use_index") not in written
    # No encoder to calibrate -- ODRIVE_ARM's gate must not block on it.
    assert axis.calibrated_encoder is True
    assert axis.index_found is True


def test_non_sensorless_push_config_still_writes_encoder_and_position_mode():
    printer, board = _printer_with_board()
    axis = _build_axis(printer, {"encoder_cpr": "8192"})

    axis.push_config()

    written = dict(board.transport.writes)
    assert written[axis.prop("encoder.config.cpr")] == 8192
    assert (
        written[axis.prop("controller.config.control_mode")]
        == properties.CONTROL_MODE_POSITION_CONTROL
    )
    assert axis.prop("config.enable_sensorless_mode") not in written


def test_sensorless_calibrate_rejects_non_motor_type():
    printer, _ = _printer_with_board()
    axis = _build_axis(
        printer,
        {"sensorless": "True", "sensorless_pm_flux_linkage": "0.02"},
    )
    gcmd = FakeGCmd({"TYPE": "full"})

    with pytest.raises(Exception, match="TYPE=motor"):
        axis.cmd_ODRIVE_CALIBRATE(gcmd)
