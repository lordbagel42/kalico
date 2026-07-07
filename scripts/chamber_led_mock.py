#!/usr/bin/env python3
"""Mock chamber_led (Particle Boron) device for device-less testing.

Speaks the exact line-oriented ASCII wire protocol described in
docs/Chamber_LED_Implementation_Spec.md over a pty, so
klippy/extras/chamber_led/*.py can be exercised with no real Boron
attached. This mirrors scripts/odrive_mock.py's approach for the
ODrive integration, but the protocol itself is far simpler (plain
request/response, no checksums, no calibration state machine), so this
script is a fraction of the size.

Usage:

    python3 scripts/chamber_led_mock.py --port /tmp/chamber_led_mock_pty

Point a [chamber_led] section at that stable path:

    [chamber_led chamber]
    serial: /tmp/chamber_led_mock_pty
    baud: 115200

Copyright (C) 2026  Kalico contributors

This file may be distributed under the terms of the GNU GPLv3 license.
"""

from __future__ import annotations

import argparse
import os
import pty
import select
import signal
import sys
import termios
import time
import tty

TICK = 0.1


class MockChamberLed:
    def __init__(self, args):
        self.args = args
        self.verbose = args.verbose
        self.start_time = None
        self.mode = "off"
        self.r = 0
        self.g = 0
        self.b = 0
        self.brightness = 0
        self.rx_buf = b""
        self.master_fd = None
        self.slave_path = None
        self.running = True

    def start(self):
        master_fd, slave_fd = pty.openpty()
        try:
            tty.setraw(master_fd)
        except termios.error:
            pass
        self.master_fd = master_fd
        self.slave_path = os.ttyname(slave_fd)
        os.close(slave_fd)
        self.start_time = time.monotonic()
        self._log("chamber_led_mock: listening on %s" % (self.slave_path,))
        if self.args.port:
            self._update_symlink()
            self._log(
                "chamber_led_mock: stable symlink %s -> %s"
                " (point 'serial:' here)" % (self.args.port, self.slave_path)
            )

    def stop(self):
        if self.master_fd is not None:
            try:
                os.close(self.master_fd)
            except OSError:
                pass
            self.master_fd = None
        if self.args.port and os.path.islink(self.args.port):
            try:
                os.unlink(self.args.port)
            except OSError:
                pass

    def _update_symlink(self):
        link = self.args.port
        tmp = "%s.tmp%d" % (link, os.getpid())
        if os.path.lexists(tmp):
            os.unlink(tmp)
        os.symlink(self.slave_path, tmp)
        os.replace(tmp, link)

    def _log(self, msg):
        print(msg, file=sys.stderr)
        sys.stderr.flush()

    def _send_reply(self, line):
        if self.verbose:
            self._log(">> %s" % (line,))
        if self.master_fd is None:
            return
        try:
            os.write(self.master_fd, (line + "\n").encode("ascii"))
        except OSError:
            pass

    def _handle_raw_line(self, raw):
        try:
            text = raw.decode("ascii", errors="replace").strip("\r\n \t")
        except Exception:
            return
        if not text:
            return
        if self.verbose:
            self._log("<< %s" % (text,))
        self._send_reply(self._dispatch(text))

    def _dispatch(self, text):
        parts = text.split()
        cmd = parts[0] if parts else ""
        if cmd == "PING":
            return self._cmd_ping(parts)
        if cmd == "COLOR":
            return self._cmd_color(parts)
        if cmd == "OFF":
            return self._cmd_off(parts)
        if cmd == "STATUS":
            return self._cmd_status(parts)
        return "ERR unknown command"

    def _cmd_ping(self, parts):
        if len(parts) != 1:
            return "ERR usage: PING"
        return "PONG"

    def _cmd_color(self, parts):
        if len(parts) != 5:
            return "ERR usage: COLOR <r> <g> <b> <brightness>"
        try:
            r, g, b, brightness = (int(p) for p in parts[1:])
        except ValueError:
            return "ERR invalid integer"
        for v in (r, g, b, brightness):
            if v < 0 or v > 255:
                return "ERR value out of range"
        self.mode = "solid"
        self.r, self.g, self.b, self.brightness = r, g, b, brightness
        return "OK"

    def _cmd_off(self, parts):
        if len(parts) != 1:
            return "ERR usage: OFF"
        self.mode = "off"
        self.r = self.g = self.b = self.brightness = 0
        return "OK"

    def _cmd_status(self, parts):
        if len(parts) != 1:
            return "ERR usage: STATUS"
        uptime_ms = int((time.monotonic() - self.start_time) * 1000.0)
        # A real Boron reports shrinking heap; a fixed value is enough to
        # exercise the field's presence/parsing on the host side.
        free_mem = 65536
        return (
            "STATUS mode=%s r=%d g=%d b=%d brightness=%d uptime_ms=%d"
            " free_mem=%d"
            % (
                self.mode,
                self.r,
                self.g,
                self.b,
                self.brightness,
                uptime_ms,
                free_mem,
            )
        )

    def run(self):
        # No signal.signal() here: run() must also work from a background
        # thread (tests drive it that way); main() wires up Ctrl-C/SIGTERM
        # for the standalone-script case.
        try:
            while self.running:
                fds = [self.master_fd] if self.master_fd is not None else []
                r, _, _ = select.select(fds, [], [], TICK)
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


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port",
        default=None,
        help="Stable path to (re)create as a symlink to the pty",
    )
    parser.add_argument(
        "--verbose", action="store_true", help="Log every line read/written"
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    mock = MockChamberLed(args)
    mock.start()

    def _stop(signum, frame):
        mock.running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    mock.run()


if __name__ == "__main__":
    main()
