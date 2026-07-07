"""Protocol-level tests for scripts/chamber_led_mock.py.

Starts the mock chamber_led device against a real pty (in a background
thread, no subprocess needed) and drives it with a real pyserial
client, to check that the mock is a faithful wire-protocol peer: PING,
COLOR (including range/arity validation), OFF, STATUS field parsing,
and the ERR path for unrecognized commands -- see
docs/Chamber_LED_Implementation_Spec.md for the protocol this mirrors
scripts/odrive_mock.py's client-facing test in test_odrive_mock.py, but
without any of the calibration/checksum machinery this protocol
doesn't have.

This intentionally does not import scripts/chamber_led_mock.py as a
package (scripts/ isn't one, by convention in this repo); it loads the
file directly by path.
"""

from __future__ import annotations

import importlib.util
import pathlib
import threading
import time

import pytest

serial = pytest.importorskip("serial")

_MOCK_PATH = (
    pathlib.Path(__file__).parent.parent / "scripts" / "chamber_led_mock.py"
)
_spec = importlib.util.spec_from_file_location("chamber_led_mock", _MOCK_PATH)
chamber_led_mock = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(chamber_led_mock)


class _Args:
    port = None
    verbose = False


@pytest.fixture
def mock_chamber_led(tmp_path):
    args = _Args()
    args.port = str(tmp_path / "chamber_led_mock_pty")
    mock = chamber_led_mock.MockChamberLed(args)
    mock.start()
    thread = threading.Thread(target=mock.run, daemon=True)
    thread.start()
    deadline = time.monotonic() + 2.0
    while not pathlib.Path(args.port).exists():
        if time.monotonic() > deadline:
            raise TimeoutError("mock chamber_led did not open its pty in time")
        time.sleep(0.01)
    try:
        yield mock, args.port
    finally:
        mock.running = False
        thread.join(timeout=2.0)


def _send(ser, line):
    ser.write((line + "\n").encode("ascii"))


def _recv(ser):
    return ser.readline().decode("ascii", errors="replace").strip()


def test_ping_pong(mock_chamber_led):
    _, port = mock_chamber_led
    with serial.Serial(port, 115200, timeout=2) as ser:
        _send(ser, "PING")
        assert _recv(ser) == "PONG"


def test_color_sets_solid_mode_and_returns_ok(mock_chamber_led):
    _, port = mock_chamber_led
    with serial.Serial(port, 115200, timeout=2) as ser:
        _send(ser, "COLOR 10 20 30 255")
        assert _recv(ser) == "OK"
        _send(ser, "STATUS")
        status = dict(kv.split("=", 1) for kv in _recv(ser).split()[1:])
        assert status["mode"] == "solid"
        assert status["r"] == "10"
        assert status["g"] == "20"
        assert status["b"] == "30"
        assert status["brightness"] == "255"


def test_off_resets_color_and_mode(mock_chamber_led):
    _, port = mock_chamber_led
    with serial.Serial(port, 115200, timeout=2) as ser:
        _send(ser, "COLOR 10 20 30 255")
        _recv(ser)
        _send(ser, "OFF")
        assert _recv(ser) == "OK"
        _send(ser, "STATUS")
        status = dict(kv.split("=", 1) for kv in _recv(ser).split()[1:])
        assert status["mode"] == "off"
        assert status["r"] == "0"


def test_status_reports_all_fields(mock_chamber_led):
    _, port = mock_chamber_led
    with serial.Serial(port, 115200, timeout=2) as ser:
        _send(ser, "STATUS")
        reply = _recv(ser)
        assert reply.startswith("STATUS ")
        fields = dict(kv.split("=", 1) for kv in reply.split()[1:])
        for key in (
            "mode",
            "r",
            "g",
            "b",
            "brightness",
            "uptime_ms",
            "free_mem",
        ):
            assert key in fields


def test_color_out_of_range_is_rejected(mock_chamber_led):
    _, port = mock_chamber_led
    with serial.Serial(port, 115200, timeout=2) as ser:
        _send(ser, "COLOR 256 0 0 255")
        assert _recv(ser).startswith("ERR")
        _send(ser, "STATUS")
        # The rejected COLOR must not have taken effect.
        status = dict(kv.split("=", 1) for kv in _recv(ser).split()[1:])
        assert status["mode"] == "off"


def test_color_wrong_arity_is_rejected(mock_chamber_led):
    _, port = mock_chamber_led
    with serial.Serial(port, 115200, timeout=2) as ser:
        _send(ser, "COLOR 1 2 3")
        assert _recv(ser).startswith("ERR")


def test_unknown_command_errors(mock_chamber_led):
    _, port = mock_chamber_led
    with serial.Serial(port, 115200, timeout=2) as ser:
        _send(ser, "BOGUS")
        assert _recv(ser).startswith("ERR")


def test_request_response_round_trip_is_fast(mock_chamber_led):
    # Sanity check that the mock itself doesn't add meaningful latency
    # on top of what real hardware measured (median 0.99ms, max 1.66ms
    # over 50 samples) -- this is a loose bound (a busy CI runner is not
    # a real-time environment), not a strict performance assertion.
    _, port = mock_chamber_led
    with serial.Serial(port, 115200, timeout=2) as ser:
        samples = []
        for _ in range(20):
            start = time.monotonic()
            _send(ser, "PING")
            assert _recv(ser) == "PONG"
            samples.append(time.monotonic() - start)
        assert max(samples) < 0.5
