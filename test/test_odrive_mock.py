"""Protocol-level tests for scripts/odrive_mock.py.

Starts the mock ODrive against a real pty (in a background thread, no
subprocess needed) and drives it with a real pyserial client, to check
that the mock is actually a faithful wire-protocol peer and not just a
rough approximation: XOR-checksum round tripping (with and without a
checksum on the request), "r"/"w" on a known property, the
"invalid property" error for an unknown one, a "p"/"f" position round
trip, and the calibration state-machine transition
(requested_state -> current_state -> back to IDLE with
motor/encoder.config.pre_calibrated set) -- the actual value this
harness adds over the existing device-less klippy tests (see
docs/ODrive_Implementation_Spec.md, "Device-less test coverage").

This intentionally does not import scripts/odrive_mock.py as a package
(scripts/ isn't one, by convention in this repo -- see how the other
scripts/*.py tools import *from* klippy instead); it loads the file
directly by path.
"""

from __future__ import annotations

import importlib.util
import pathlib
import threading
import time

import pytest

serial = pytest.importorskip("serial")

_MOCK_PATH = pathlib.Path(__file__).parent.parent / "scripts" / "odrive_mock.py"
_spec = importlib.util.spec_from_file_location("odrive_mock", _MOCK_PATH)
odrive_mock = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(odrive_mock)


class _Args:
    port = None
    axes = 2
    fast = False
    calib_delay = 0.05
    reboot_delay = 0.05
    motion_tau = 0.02
    vbus = 24.0
    serial_number = 0x3559316F3237
    fw_version = (0, 5, 4)
    hw_version = (3, 6)
    verbose = False


@pytest.fixture
def mock_odrive(tmp_path):
    args = _Args()
    args.port = str(tmp_path / "odrive_mock_pty")
    mock = odrive_mock.MockOdrive(args)
    mock.start()
    thread = threading.Thread(target=mock.run, daemon=True)
    thread.start()
    deadline = time.monotonic() + 2.0
    while not pathlib.Path(args.port).exists():
        if time.monotonic() > deadline:
            raise TimeoutError("mock odrive did not open its pty in time")
        time.sleep(0.01)
    try:
        yield mock, args.port
    finally:
        mock.running = False
        thread.join(timeout=2.0)


def _xor_checksum(body):
    cs = 0
    for ch in body.encode("ascii"):
        cs ^= ch
    return cs


def _send(ser, line, checksum=True):
    if checksum:
        line = "%s*%d" % (line, _xor_checksum(line))
    ser.write((line + "\n").encode("ascii"))


def _recv(ser):
    return ser.readline().decode("ascii", errors="replace").strip()


def _body(reply):
    return reply.split("*")[0]


def test_checksum_round_trip(mock_odrive):
    _, port = mock_odrive
    with serial.Serial(port, 115200, timeout=2) as ser:
        _send(ser, "r vbus_voltage", checksum=True)
        reply = _recv(ser)
        body, _, cs = reply.rpartition("*")
        assert int(cs) == _xor_checksum(body)
        assert float(body) == pytest.approx(24.0)


def test_reply_omits_checksum_when_request_had_none(mock_odrive):
    _, port = mock_odrive
    with serial.Serial(port, 115200, timeout=2) as ser:
        _send(ser, "r vbus_voltage", checksum=False)
        reply = _recv(ser)
        assert "*" not in reply
        assert float(reply) == pytest.approx(24.0)


def test_bad_checksum_is_dropped(mock_odrive):
    _, port = mock_odrive
    with serial.Serial(port, 115200, timeout=1) as ser:
        ser.write(b"r vbus_voltage*99\n")
        ser.write(b"r vbus_voltage\n")  # unambiguous follow-up, no checksum
        # The corrupted line above must not have produced a reply; the
        # very next line read back must be the second (valid) request's
        # reply, not a stray reply to the first.
        reply = _recv(ser)
        assert float(reply) == pytest.approx(24.0)


def test_read_write_known_property(mock_odrive):
    _, port = mock_odrive
    with serial.Serial(port, 115200, timeout=2) as ser:
        _send(ser, "w axis0.motor.config.current_lim 12.5")
        time.sleep(0.05)
        _send(ser, "r axis0.motor.config.current_lim")
        assert float(_body(_recv(ser))) == pytest.approx(12.5)


def test_read_unknown_property_errors(mock_odrive):
    _, port = mock_odrive
    with serial.Serial(port, 115200, timeout=2) as ser:
        _send(ser, "r axis0.totally.bogus")
        assert _body(_recv(ser)) == "invalid property"


def test_write_unknown_property_is_ignored_not_created(mock_odrive):
    _, port = mock_odrive
    with serial.Serial(port, 115200, timeout=2) as ser:
        _send(ser, "w axis0.totally.bogus 1")
        time.sleep(0.05)
        _send(ser, "r axis0.totally.bogus")
        assert _body(_recv(ser)) == "invalid property"


def test_serial_number_is_plain_decimal(mock_odrive):
    # klippy/extras/odrive/__init__.py._check_serial_number does a bare
    # int(raw) (no float()), so the reply must not contain a decimal
    # point the way most other numeric properties' replies do.
    _, port = mock_odrive
    with serial.Serial(port, 115200, timeout=2) as ser:
        _send(ser, "r serial_number")
        body = _body(_recv(ser))
        assert int(body) == 0x3559316F3237
        assert "." not in body


def test_position_feedback_tracks_setpoint(mock_odrive):
    _, port = mock_odrive
    with serial.Serial(port, 115200, timeout=2) as ser:
        _send(ser, "p 0 5.0 0.0 0.0")
        time.sleep(0.3)
        _send(ser, "f 0")
        pos_str, vel_str = _body(_recv(ser)).split()
        assert float(pos_str) == pytest.approx(5.0, abs=0.05)


def test_calibration_state_transition(mock_odrive):
    _, port = mock_odrive
    with serial.Serial(port, 115200, timeout=2) as ser:
        _send(ser, "w axis0.requested_state 3")  # full calibration
        time.sleep(0.02)
        _send(ser, "r axis0.current_state")
        assert _body(_recv(ser)) == "3"

        deadline = time.monotonic() + 2.0
        state = None
        while time.monotonic() < deadline:
            _send(ser, "r axis0.current_state")
            state = _body(_recv(ser))
            if state == "1":
                break
            time.sleep(0.02)
        assert state == "1", "calibration never returned to IDLE"

        _send(ser, "r axis0.motor.config.pre_calibrated")
        assert _body(_recv(ser)) == "1"
        _send(ser, "r axis0.encoder.config.pre_calibrated")
        assert _body(_recv(ser)) == "1"


def test_sc_clears_errors(mock_odrive):
    _, port = mock_odrive
    with serial.Serial(port, 115200, timeout=2) as ser:
        _send(ser, "w axis0.error 8")
        time.sleep(0.05)
        _send(ser, "r axis0.error")
        assert _body(_recv(ser)) == "8"
        _send(ser, "sc")
        time.sleep(0.05)
        _send(ser, "r axis0.error")
        assert _body(_recv(ser)) == "0"


def test_save_config_triggers_reconnectable_offline_window(mock_odrive):
    mock, port = mock_odrive
    with serial.Serial(port, 115200, timeout=2) as ser:
        _send(ser, "ss")
    # Mirrors klippy/extras/odrive/__init__.py's cmd_ODRIVE_SAVE_CONFIG:
    # the client closes its own handle right after sending "ss" and then
    # polls os.path.exists(serial_path) waiting for it to reappear.
    deadline = time.monotonic() + 2.0
    saw_offline = False
    while time.monotonic() < deadline:
        if not pathlib.Path(port).exists():
            saw_offline = True
            break
        time.sleep(0.005)
    assert saw_offline, "the stable port symlink never disappeared after ss"

    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        if pathlib.Path(port).exists():
            break
        time.sleep(0.01)
    else:
        pytest.fail("the stable port symlink never reappeared after ss")

    with serial.Serial(port, 115200, timeout=2) as ser:
        _send(ser, "r vbus_voltage")
        assert float(_body(_recv(ser))) == pytest.approx(24.0)
