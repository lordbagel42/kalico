"""Tests for klippy/extras/odrive/__init__.py's board-level shutdown
handling.

Specifically: that printer.invoke_shutdown()'s "klippy:shutdown" event
(and "klippy:disconnect") actually reaches ODriveBoard and disarms
every registered axis, independent of each axis's own local fault path
(handle_fault/stepper_enable:motor_off). This was previously untested --
see the "e-stop system" review in overnight/OVERNIGHT_LOG.md (in the
claude-kalico repo) for how this was scoped.

Exercises the *real* ODriveBoard class (not a fake of it), constructed
against a real klippy.configfile.ConfigWrapper, per this module's
existing "no full Klippy printer object graph" test philosophy (see
test_odrive_repl.py, test_odrive_axis.py). Axes are simple fakes, since
this is testing the board's dispatch to every axis, not axis behavior
itself (that's test_odrive_axis.py's job).
"""

from __future__ import annotations

import configparser

from klippy import configfile
from klippy.extras import odrive as odrive_mod


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


class FakePrinter:
    """Tracks registered event handlers and can actually fire them, like
    the real Printer.send_event -- so a test can verify both that
    ODriveBoard registers under the right event name *and* that firing
    it does the right thing, not just that the method works in
    isolation."""

    def __init__(self):
        self._reactor = object()
        self.event_handlers = {}
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

    def register_event_handler(self, event, handler):
        self.event_handlers.setdefault(event, []).append(handler)

    def send_event(self, event):
        return [cb() for cb in self.event_handlers.get(event, [])]


class FakeAxis:
    """Records whether/how many times on_emergency_idle() ran, without
    needing a real ODriveAxis (this is testing the board's dispatch to
    every registered axis, not per-axis behavior)."""

    def __init__(self, axis_index):
        self.axis_index = axis_index
        self.emergency_idle_calls = 0

    def on_emergency_idle(self):
        self.emergency_idle_calls += 1


def make_config(section, options, printer):
    parser = configparser.ConfigParser()
    parser.add_section(section)
    for key, value in options.items():
        parser.set(section, key, str(value))
    return configfile.ConfigWrapper(printer, parser, {}, section)


def _build_board(printer, extra_options=None):
    options = {"serial": "/dev/null"}
    options.update(extra_options or {})
    config = make_config("odrive drive0", options, printer)
    return odrive_mod.ODriveBoard(config)


def test_shutdown_handler_registered_under_klippy_shutdown():
    printer = FakePrinter()
    board = _build_board(printer)

    assert board._handle_shutdown in printer.event_handlers["klippy:shutdown"]
    assert board._handle_shutdown in printer.event_handlers["klippy:disconnect"]


def test_klippy_shutdown_event_idles_every_axis():
    printer = FakePrinter()
    board = _build_board(printer)
    board.connected = True
    axis0 = FakeAxis(0)
    axis1 = FakeAxis(1)
    board.axes = {0: axis0, 1: axis1}

    printer.send_event("klippy:shutdown")

    assert axis0.emergency_idle_calls == 1
    assert axis1.emergency_idle_calls == 1
    assert board.connected is False


def test_klippy_disconnect_event_also_idles_every_axis():
    printer = FakePrinter()
    board = _build_board(printer)
    board.connected = True
    axis0 = FakeAxis(0)
    board.axes = {0: axis0}

    printer.send_event("klippy:disconnect")

    assert axis0.emergency_idle_calls == 1


def test_shutdown_when_already_disconnected_does_not_touch_axes():
    # _handle_shutdown only walks self.axes when self.connected is
    # already True -- a second shutdown/disconnect event (e.g. both
    # firing during the same shutdown) must not re-run every axis's
    # emergency-idle path a second time for no reason.
    printer = FakePrinter()
    board = _build_board(printer)
    board.connected = False
    axis0 = FakeAxis(0)
    board.axes = {0: axis0}

    printer.send_event("klippy:shutdown")

    assert axis0.emergency_idle_calls == 0
