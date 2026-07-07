# Particle Boron chamber LED USB-serial ASCII protocol transport
#
# Implements the host-side serial transport described in
# docs/Chamber_LED_Implementation_Spec.md: non-blocking pyserial I/O
# driven by the reactor (register_fd for RX, register_timer for TX),
# following the same idiom as klippy/extras/odrive/transport.py. Unlike
# the ODrive link, this protocol is a short, trusted point-to-point USB
# link (not a noisy multi-drop bus), and it is strictly one
# request/response at a time -- so there is no checksum framing and no
# FIFO request queue, just a single pending request slot.
#
# Copyright (C) 2026  Kalico contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections
import logging

try:
    import serial
except ImportError:
    serial = None


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


class ChamberLedTransport:
    def __init__(self, printer, name, on_io_error=None):
        self.printer = printer
        self.reactor = printer.get_reactor()
        self.name = name
        self.on_io_error = on_io_error
        self.serial = None
        self.rx_buf = b""
        # At most one request is ever in flight (the protocol is
        # strictly request/response), so a single slot replaces the
        # pending-request deque that a multi-drop/streaming link (like
        # the ODrive's) would need.
        self.pending = None
        self.tx_queue = collections.deque()
        self.tx_timer = None
        self.fd_handle = None

    def is_open(self):
        return self.serial is not None

    def open(self, serial_path, baud=115200):
        if serial is None:
            raise self.printer.command_error(
                "The 'pyserial' Python module is required for chamber_led"
                " support (pip install pyserial)"
            )
        self.serial = serial.Serial(
            serial_path, baud, timeout=0, write_timeout=0
        )
        self.rx_buf = b""
        self.pending = None
        self.tx_queue.clear()
        self.fd_handle = self.reactor.register_fd(
            self.serial.fileno(), self._on_readable
        )
        if self.tx_timer is None:
            self.tx_timer = self.reactor.register_timer(self._tx_tick)

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
                logging.exception("chamber_led: error closing serial port")
            self.serial = None
        if self.pending is not None:
            pending, self.pending = self.pending, None
            pending.complete(None)
        self.tx_queue.clear()

    def send_line(self, line):
        if self.serial is None:
            return
        self.tx_queue.append(line)
        # Wake the TX timer immediately -- this link is only ever driven
        # by short request/response exchanges, not a continuous stream,
        # so there is nothing to gain from waiting for a periodic tick
        # (and doing so would blow the ~1ms round-trip budget).
        self.reactor.update_timer(self.tx_timer, self.reactor.NOW)

    def query(self, line, callback, timeout=1.0):
        if self.serial is None:
            callback(None)
            return
        if self.pending is not None:
            # Callers must not overlap requests; this is a caller bug,
            # not a transport-level condition worth queuing for.
            callback(None)
            return
        self.pending = _PendingRequest(
            callback, self.reactor.monotonic() + timeout
        )
        self.send_line(line)

    def query_sync(self, line, timeout=1.0):
        result = []

        def cb(value):
            result.append(value)

        self.query(line, cb, timeout)
        end_time = self.reactor.monotonic() + timeout
        while not result:
            now = self.reactor.monotonic()
            if now > end_time or self.serial is None:
                # Free the pending slot; if a reply does show up later
                # it will be handled as an unsolicited line.
                self.pending = None
                return None
            self.reactor.pause(min(now + 0.01, end_time))
        return result[0]

    # Internal I/O
    def _write_line(self, line):
        data = (line + "\n").encode("ascii", errors="replace")
        try:
            self.serial.write(data)
        except Exception:
            self._handle_io_error("write failed")

    def _tx_tick(self, eventtime):
        if self.serial is None:
            return self.reactor.NEVER
        if self.tx_queue:
            self._write_line(self.tx_queue.popleft())
        if self.serial is not None and self.tx_queue:
            return eventtime
        return self.reactor.NEVER

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
        if self.pending is not None:
            pending, self.pending = self.pending, None
            pending.complete(text)
        # Otherwise: unsolicited line (e.g. a stale reply after a
        # timeout already gave up on it); ignore it.

    def _handle_io_error(self, reason):
        if self.on_io_error is not None:
            self.on_io_error(reason)
        else:
            self.close()
