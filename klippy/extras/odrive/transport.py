# ODrive v3.6 USB-CDC ASCII protocol transport
#
# Implements the host-side serial transport described in
# docs/ODrive_Implementation_Spec.md: non-blocking pyserial I/O driven
# by the reactor (register_fd for RX, register_timer for TX), optional
# GCode-style XOR checksums, and FIFO request/response matching (only
# "r"/"f" ASCII commands ever produce a reply, and the ODrive processes
# lines strictly in order, so a plain queue is sufficient -- no sequence
# numbers are needed, unlike the native/Fibre protocol).
#
# Copyright (C) 2026  Kalico contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections
import logging
import re

try:
    import serial
except ImportError:
    serial = None

DEFAULT_TX_PERIOD = 0.05
RESYNC_FAILURE_LIMIT = 3

# Dotted ODrive property paths (e.g. "axis0.motor.config.current_lim").
# Anything outside this conservative set -- in particular whitespace and
# control characters -- could break the line-oriented ASCII framing or
# smuggle extra commands onto the serial link, so callers that accept
# externally-supplied paths (webhooks, gcode) must validate first.
PROPERTY_PATH_RE = re.compile(r"^[A-Za-z0-9_.]{1,128}$")


def valid_property_path(path):
    return isinstance(path, str) and PROPERTY_PATH_RE.match(path) is not None


class TransportState:
    DISCONNECTED = "disconnected"
    PROBING = "probing"
    CONFIGURING = "configuring"
    READY = "ready"
    REBOOT_PENDING = "reboot_pending"
    LOST = "lost"


class _PendingRequest:
    def __init__(self, callback, deadline):
        self.callback = callback
        self.deadline = deadline
        self.done = False

    def complete(self, value):
        if self.done:
            return
        self.done = True
        if self.callback is not None:
            self.callback(value)


class OdriveTransport:
    def __init__(self, printer, name, on_io_error=None):
        self.printer = printer
        self.reactor = printer.get_reactor()
        self.name = name
        self.on_io_error = on_io_error
        self.use_checksums = True
        self.serial = None
        self.state = TransportState.DISCONNECTED
        self.rx_buf = b""
        self.pending = collections.deque()
        self.tx_queue = collections.deque()
        self.stream_cb = None
        self.tx_timer = None
        self.tx_period = DEFAULT_TX_PERIOD
        self.fd_handle = None
        self.resync_failures = 0
        self.stats = {
            "tx_bytes": 0,
            "rx_bytes": 0,
            "underruns": 0,
            "last_tick_time": None,
            "jitter_ms": 0.0,
        }

    def is_open(self):
        return self.serial is not None

    def open(self, serial_path, baud=115200):
        if serial is None:
            raise self.printer.command_error(
                "The 'pyserial' Python module is required for ODrive"
                " support (pip install pyserial)"
            )
        self.serial = serial.Serial(
            serial_path, baud, timeout=0, write_timeout=0
        )
        self.rx_buf = b""
        self.pending.clear()
        self.tx_queue.clear()
        self.resync_failures = 0
        self.fd_handle = self.reactor.register_fd(
            self.serial.fileno(), self._on_readable
        )
        if self.tx_timer is None:
            self.tx_timer = self.reactor.register_timer(
                self._tx_tick, self.reactor.NOW
            )
        else:
            self.reactor.update_timer(self.tx_timer, self.reactor.NOW)
        self.state = TransportState.PROBING

    def close(self):
        if self.fd_handle is not None:
            self.reactor.unregister_fd(self.fd_handle)
            self.fd_handle = None
        if self.tx_timer is not None:
            self.reactor.unregister_timer(self.tx_timer)
            self.tx_timer = None
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                logging.exception("odrive: error closing serial port")
            self.serial = None
        while self.pending:
            self.pending.popleft().complete(None)
        self.tx_queue.clear()
        self.stream_cb = None
        self.state = TransportState.DISCONNECTED

    def set_tx_period(self, period):
        self.tx_period = period

    def set_stream_callback(self, cb):
        self.stream_cb = cb

    def send_line(self, line):
        if self.serial is None:
            return
        self.tx_queue.append(line)

    def query(self, line, callback, timeout=2.0):
        if self.serial is None:
            callback(None)
            return
        req = _PendingRequest(callback, self.reactor.monotonic() + timeout)
        self.pending.append(req)
        self.send_line(line)

    def query_sync(self, line, timeout=2.0):
        result = []

        def cb(value):
            result.append(value)

        self.query(line, cb, timeout)
        end_time = self.reactor.monotonic() + timeout + 0.5
        while not result:
            now = self.reactor.monotonic()
            if now > end_time or self.serial is None:
                return None
            self.reactor.pause(min(now + 0.05, end_time))
        return result[0]

    def read_property_sync(self, path, timeout=2.0):
        return self.query_sync("r %s" % (path,), timeout)

    def write_property(self, path, value):
        self.send_line("w %s %s" % (path, value))

    def write_property_sync(self, path, value, verify=False, timeout=2.0):
        self.write_property(path, value)
        if not verify:
            return True
        readback = self.read_property_sync(path, timeout)
        if readback is None:
            return False
        try:
            return abs(float(readback) - float(value)) < 1e-6 or (
                readback.strip() == str(value)
            )
        except (TypeError, ValueError):
            return readback.strip() == str(value)

    def feed_watchdog(self, motor_index):
        self.send_line("u %d" % (motor_index,))

    def send_position_setpoint(
        self, motor_index, pos_turns, vel_ff=0.0, torque_ff=0.0
    ):
        self.send_line(
            "p %d %.6f %.6f %.6f" % (motor_index, pos_turns, vel_ff, torque_ff)
        )

    # Internal I/O
    def _checksum(self, body):
        cs = 0
        for ch in body.encode("ascii", errors="replace"):
            cs ^= ch
        return cs

    def _write_line(self, line):
        if self.use_checksums:
            line = "%s*%d" % (line, self._checksum(line))
        data = (line + "\n").encode("ascii", errors="replace")
        try:
            self.serial.write(data)
        except Exception:
            self._handle_io_error("write failed")
            return
        self.stats["tx_bytes"] += len(data)

    def _tx_tick(self, eventtime):
        if self.serial is None:
            return self.reactor.NEVER
        last_tick = self.stats["last_tick_time"]
        if last_tick is not None:
            jitter = abs((eventtime - last_tick) - self.tx_period) * 1000.0
            self.stats["jitter_ms"] = (
                0.9 * self.stats["jitter_ms"] + 0.1 * jitter
            )
        self.stats["last_tick_time"] = eventtime
        # Expire timed-out requests; enough consecutive timeouts means
        # the device has gone silent while the TTY stayed open, which
        # takes the same lost-link path as a checksum resync failure
        # (any successfully received line resets the counter)
        while self.pending and eventtime > self.pending[0].deadline:
            self.pending.popleft().complete(None)
            self.resync_failures += 1
            self.stats["underruns"] += 1
        if self.resync_failures >= RESYNC_FAILURE_LIMIT:
            self.resync_failures = 0
            self._handle_io_error("device not responding (request timeouts)")
            return self.reactor.NEVER
        wrote = False
        if self.stream_cb is not None:
            line = self.stream_cb(eventtime)
            if line is not None:
                self._write_line(line)
                wrote = True
        if self.tx_queue:
            self._write_line(self.tx_queue.popleft())
            wrote = True
        if self.serial is None:
            return self.reactor.NEVER
        if not wrote and self.stream_cb is None:
            self.stats["underruns"] += 0  # idle tick, not an underrun
        return eventtime + self.tx_period

    def _on_readable(self, eventtime):
        if self.serial is None:
            return
        try:
            data = self.serial.read(4096)
        except Exception:
            self._handle_io_error("read failed")
            return
        if not data:
            return
        self.stats["rx_bytes"] += len(data)
        self.rx_buf += data
        while b"\n" in self.rx_buf:
            line, self.rx_buf = self.rx_buf.split(b"\n", 1)
            self._handle_line(line)

    def _handle_line(self, raw):
        try:
            text = raw.decode("ascii", errors="replace").strip("\r\n \t")
        except Exception:
            return
        if not text:
            return
        if self.use_checksums and "*" in text:
            body, _, cs_str = text.rpartition("*")
            try:
                expected = int(cs_str)
            except ValueError:
                body, expected = text, None
            if expected is not None:
                if self._checksum(body) != expected:
                    self._resync_failure()
                    return
                text = body
        self.resync_failures = 0
        if self.pending:
            self.pending.popleft().complete(text)
        # Otherwise: unsolicited line (should not normally happen on the
        # ASCII protocol); ignore it.

    def _resync_failure(self):
        self.resync_failures += 1
        if self.resync_failures >= RESYNC_FAILURE_LIMIT:
            self.resync_failures = 0
            self._handle_io_error("checksum resync failed")

    def _handle_io_error(self, reason):
        if self.on_io_error is not None:
            self.on_io_error(reason)
        else:
            self.close()
