# ODrive v3.6 USB servo board support
#
# See docs/ODrive_Implementation_Spec.md for the full design. This
# module implements the board-level [odrive <name>] object: the USB
# transport, connection lifecycle (including the fw>=0.5.2
# save-triggers-reboot re-enumeration quirk), firmware-version
# detection, error/vbus polling, and the board-level gcode command set.
# Per-motor state lives in the companion odrive_axis module
# ([odrive_axis <name>], klippy/extras/odrive_axis.py) so that Kalico's
# section-name-to-module resolution (which keys off the first word of
# the config section) can find it.
#
# Copyright (C) 2026  Kalico contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import os

from . import properties
from .transport import OdriveTransport, TransportState


class ODriveBoard:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.full_name = config.get_name()
        self.name = self.full_name.split()[-1]
        self.serial_path = config.get("serial")
        self.serial_number_hex = config.get("serial_number", None)
        if self.serial_number_hex:
            self.serial_number_hex = self.serial_number_hex.strip().upper()
        self.baud = config.getint("baud", 115200)
        self.use_checksums = config.getboolean("use_checksums", True)
        self.auto_reconnect = config.getboolean("auto_reconnect", True)
        self.reconnect_timeout = config.getfloat(
            "reconnect_timeout", 15.0, above=0.0
        )
        self.sample_period = config.getfloat(
            "sample_period", 0.005, minval=0.002, maxval=0.02
        )
        self.latency_compensation = config.getfloat(
            "latency_compensation", 0.010, minval=0.0
        )
        self.error_poll_period = config.getfloat(
            "error_poll_period", 0.5, above=0.0
        )
        self.watchdog_timeout = config.getfloat(
            "watchdog_timeout", 1.0, minval=0.0
        )
        if 0.0 < self.watchdog_timeout < 0.25:
            raise config.error(
                "watchdog_timeout in section '%s' must be 0 (disabled) or"
                " >= 0.25" % (self.full_name,)
            )
        self.vbus_min = config.getfloat("vbus_min", 10.0, minval=0.0)
        self.vbus_max = config.getfloat("vbus_max", 26.0, above=self.vbus_min)
        self.idle_feed_period = config.getfloat(
            "idle_feed_period", 0.1, above=0.0
        )
        self.stall_shutdown_ticks = config.getint(
            "stall_shutdown_ticks", 3, minval=1
        )

        self.transport = OdriveTransport(
            self.printer, self.full_name, on_io_error=self._handle_io_error
        )
        self.transport.use_checksums = self.use_checksums

        self.axes = {}
        self.props = properties.PropertyMap(None)
        self.fw_version = None
        self.hw_version = None
        self.serial_number = None
        self.vbus_voltage = None
        self.connected = False
        self.state = TransportState.DISCONNECTED
        self._reboot_pending = False
        self._reconnect_timer = None
        self._reconnect_deadline = 0.0
        self._error_poll_timer = None
        self._poll_axis_index = 0

        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler(
            "klippy:shutdown", self._handle_shutdown
        )
        self.printer.register_event_handler(
            "klippy:disconnect", self._handle_shutdown
        )

        gcode = self.printer.lookup_object("gcode")
        commands = [
            (
                "ODRIVE_CONNECT",
                self.cmd_ODRIVE_CONNECT,
                "Connect to an ODrive board",
            ),
            (
                "ODRIVE_DISCONNECT",
                self.cmd_ODRIVE_DISCONNECT,
                "Disconnect from an ODrive board",
            ),
            (
                "ODRIVE_STATUS",
                self.cmd_ODRIVE_STATUS,
                "Report ODrive board/axis status",
            ),
            (
                "ODRIVE_CLEAR_ERRORS",
                self.cmd_ODRIVE_CLEAR_ERRORS,
                "Clear all ODrive error flags",
            ),
            (
                "ODRIVE_ERRORS",
                self.cmd_ODRIVE_ERRORS,
                "Decode and report ODrive error flags",
            ),
            ("ODRIVE_READ", self.cmd_ODRIVE_READ, "Read a raw ODrive property"),
            (
                "ODRIVE_WRITE",
                self.cmd_ODRIVE_WRITE,
                "Write a raw ODrive property",
            ),
            (
                "ODRIVE_DUMP_CONFIG",
                self.cmd_ODRIVE_DUMP_CONFIG,
                "Dump known ODrive config properties",
            ),
            (
                "ODRIVE_SAVE_CONFIG",
                self.cmd_ODRIVE_SAVE_CONFIG,
                "Persist config to ODrive NVM (reboots)",
            ),
            (
                "ODRIVE_ERASE_CONFIG",
                self.cmd_ODRIVE_ERASE_CONFIG,
                "Erase ODrive NVM config (reboots)",
            ),
            (
                "ODRIVE_REBOOT",
                self.cmd_ODRIVE_REBOOT,
                "Reboot the ODrive board",
            ),
        ]
        for cmd, func, desc in commands:
            gcode.register_mux_command(
                cmd, "ODRIVE", self.name, func, desc=desc
            )

        from . import telemetry

        telemetry.register_board_endpoint(self)

    # Minimal MCU-like surface used by ODriveStepper (get_mcu() returns
    # this board) so it shares the same print_time timebase as
    # MCU-driven steppers in the same kinematics.
    non_critical_disconnected = False

    def estimated_print_time(self, eventtime):
        mcu = self.printer.lookup_object("mcu")
        return mcu.estimated_print_time(eventtime)

    def is_fileoutput(self):
        mcu = self.printer.lookup_object("mcu")
        return mcu.is_fileoutput()

    def get_name(self):
        return self.full_name

    # Registration from odrive_axis.py
    def register_axis(self, axis):
        if axis.axis_index in self.axes:
            raise self.printer.config_error(
                "ODrive %s axis %d is already used by '%s'"
                % (
                    self.full_name,
                    axis.axis_index,
                    self.axes[axis.axis_index].full_name,
                )
            )
        self.axes[axis.axis_index] = axis

    def get_status(self, eventtime=None):
        errors = []
        for axis in self.axes.values():
            for names in axis.last_errors.values():
                errors.extend(names)
        return {
            "connected": self.connected,
            "state": self.state,
            "fw_version": _version_str(self.fw_version),
            "hw_version": _version_str(self.hw_version),
            "serial_number": self.serial_number,
            "vbus_voltage": self.vbus_voltage,
            "errors": errors,
            "streaming": {
                "rate": (1.0 / self.sample_period)
                if self.sample_period
                else 0.0,
                "jitter_ms": self.transport.stats["jitter_ms"],
                "underruns": self.transport.stats["underruns"],
                "tx_bytes": self.transport.stats["tx_bytes"],
            },
            "save_config_pending": False,
            "capabilities": dict(self.props.capabilities),
        }

    # Connection lifecycle
    def _handle_ready(self):
        try:
            self._try_connect()
        except Exception as e:
            logging.info(
                "odrive %s: initial connect failed: %s", self.full_name, e
            )

    def _handle_shutdown(self):
        if self.connected:
            for axis in self.axes.values():
                axis.on_emergency_idle()
        self.transport.close()
        self.connected = False
        self.state = TransportState.DISCONNECTED
        self._stop_error_poll()

    def _try_connect(self):
        if not os.path.exists(self.serial_path):
            raise self.printer.command_error(
                "ODrive serial device '%s' does not exist" % (self.serial_path,)
            )
        self.transport.open(self.serial_path, self.baud)
        self.state = TransportState.PROBING
        self._run_probe_and_configure()

    def _run_probe_and_configure(self):
        t = self.transport
        vbus = t.read_property_sync("vbus_voltage", timeout=2.0)
        if vbus is None:
            t.close()
            self.state = TransportState.DISCONNECTED
            raise self.printer.command_error(
                "ODrive %s did not respond on %s"
                % (self.full_name, self.serial_path)
            )
        try:
            self.vbus_voltage = float(vbus)
        except ValueError:
            self.vbus_voltage = None
        fw_version = self._probe_version(
            "fw_version_major", "fw_version_minor", "fw_version_revision"
        )
        self.fw_version = fw_version
        self.hw_version = self._probe_version(
            "hw_version_major", "hw_version_minor"
        )
        self.props = properties.PropertyMap(fw_version)
        self._check_serial_number()
        self.state = TransportState.CONFIGURING
        for axis in self.axes.values():
            axis.push_config()
        self.state = TransportState.READY
        self.connected = True
        self._start_error_poll()
        logging.info(
            "odrive %s: connected, firmware %s",
            self.full_name,
            _version_str(fw_version) or "unknown",
        )

    def _probe_version(self, *properties_):
        parts = []
        for prop in properties_:
            raw = self.transport.read_property_sync(prop, timeout=1.0)
            try:
                parts.append(int(float(raw)))
            except (TypeError, ValueError):
                return None
        version = tuple(parts)
        if not any(version):
            return None
        return version

    def _check_serial_number(self):
        raw = self.transport.read_property_sync("serial_number", timeout=1.0)
        try:
            sn_val = int(raw)
        except (TypeError, ValueError):
            sn_val = None
        if sn_val:
            self.serial_number = "%X" % (sn_val,)
        else:
            self.serial_number = None
            if self.serial_number_hex:
                logging.warning(
                    "odrive %s: device did not report a usable serial"
                    " number; skipping serial_number verification"
                    " (known clone-firmware limitation)",
                    self.full_name,
                )
            return
        if (
            self.serial_number_hex
            and self.serial_number != self.serial_number_hex
        ):
            self.transport.close()
            self.state = TransportState.DISCONNECTED
            raise self.printer.command_error(
                "ODrive at %s reports serial number %s, expected %s"
                % (self.serial_path, self.serial_number, self.serial_number_hex)
            )

    def _handle_io_error(self, reason):
        logging.warning(
            "odrive %s: transport error: %s", self.full_name, reason
        )
        was_ready = self.state == TransportState.READY
        self.transport.close()
        self.connected = False
        self._stop_error_poll()
        if self._reboot_pending:
            self.state = TransportState.REBOOT_PENDING
            self._start_reconnect_poll()
            return
        self.state = TransportState.LOST
        for axis in self.axes.values():
            axis.on_disconnected()
        critical = any(axis.is_motion_critical() for axis in self.axes.values())
        if was_ready and critical:
            self.printer.invoke_shutdown(
                "ODrive %s lost communication during motion" % (self.full_name,)
            )
            return
        if self.auto_reconnect:
            self._start_reconnect_poll()

    def _start_reconnect_poll(self):
        self._reconnect_deadline = (
            self.reactor.monotonic() + self.reconnect_timeout
        )
        if self._reconnect_timer is None:
            self._reconnect_timer = self.reactor.register_timer(
                self._reconnect_tick
            )
        self.reactor.update_timer(
            self._reconnect_timer, self.reactor.monotonic() + 0.5
        )

    def _reconnect_tick(self, eventtime):
        if eventtime > self._reconnect_deadline:
            logging.warning(
                "odrive %s: gave up reconnecting after %.1fs",
                self.full_name,
                self.reconnect_timeout,
            )
            self._reboot_pending = False
            return self.reactor.NEVER
        if not os.path.exists(self.serial_path):
            return eventtime + 0.5
        try:
            self._try_connect()
        except Exception as e:
            logging.info(
                "odrive %s: reconnect attempt failed: %s", self.full_name, e
            )
            return eventtime + 0.5
        self._reboot_pending = False
        for axis in self.axes.values():
            axis.on_reconnected()
        return self.reactor.NEVER

    # Error / telemetry polling
    def _start_error_poll(self):
        if self._error_poll_timer is None:
            self._error_poll_timer = self.reactor.register_timer(
                self._error_poll_tick
            )
        self.reactor.update_timer(self._error_poll_timer, self.reactor.NOW)

    def _stop_error_poll(self):
        if self._error_poll_timer is not None:
            self.reactor.update_timer(
                self._error_poll_timer, self.reactor.NEVER
            )

    def _error_poll_tick(self, eventtime):
        if not self.connected:
            return self.reactor.NEVER
        vbus = self.transport.read_property_sync("vbus_voltage", timeout=1.0)
        try:
            if vbus is not None:
                self.vbus_voltage = float(vbus)
        except ValueError:
            pass
        if self.vbus_voltage is not None and (
            self.vbus_voltage < self.vbus_min
            or self.vbus_voltage > self.vbus_max
        ):
            for axis in self.axes.values():
                if axis.armed:
                    axis.handle_fault(
                        "bus voltage %.2fV out of range [%.2f, %.2f]"
                        % (self.vbus_voltage, self.vbus_min, self.vbus_max)
                    )
        axes = list(self.axes.values())
        if axes:
            idx = self._poll_axis_index % len(axes)
            self._poll_axis_index += 1
            axes[idx].poll_errors_and_telemetry()
        return eventtime + self.error_poll_period

    # GCode commands
    def _require_connected(self, gcmd):
        if not self.connected:
            raise gcmd.error(
                "ODrive %s is not connected (run ODRIVE_CONNECT ODRIVE=%s)"
                % (self.full_name, self.name)
            )

    def cmd_ODRIVE_CONNECT(self, gcmd):
        if self.connected:
            gcmd.respond_info(
                "ODrive %s is already connected" % (self.full_name,)
            )
            return
        self._try_connect()
        gcmd.respond_info(
            "ODrive %s connected, firmware %s"
            % (self.full_name, _version_str(self.fw_version) or "unknown")
        )

    def cmd_ODRIVE_DISCONNECT(self, gcmd):
        for axis in self.axes.values():
            axis.on_emergency_idle()
        self.transport.close()
        self.connected = False
        self._stop_error_poll()
        self.state = TransportState.DISCONNECTED
        gcmd.respond_info("ODrive %s disconnected" % (self.full_name,))

    def cmd_ODRIVE_STATUS(self, gcmd):
        gcmd.respond_info(
            "ODrive %s: state=%s connected=%s fw=%s hw=%s serial=%s vbus=%s"
            % (
                self.full_name,
                self.state,
                self.connected,
                _version_str(self.fw_version) or "unknown",
                _version_str(self.hw_version) or "unknown",
                self.serial_number or "unknown",
                "%.2fV" % self.vbus_voltage
                if self.vbus_voltage is not None
                else "?",
            )
        )
        for axis in sorted(self.axes.values(), key=lambda a: a.axis_index):
            gcmd.respond_info(axis.format_status_line())

    def cmd_ODRIVE_CLEAR_ERRORS(self, gcmd):
        self._require_connected(gcmd)
        self.transport.send_line("sc")
        for axis in self.axes.values():
            axis.last_errors = {
                "axis": [],
                "motor": [],
                "encoder": [],
                "controller": [],
            }
            axis.last_error_codes = {
                "axis": 0,
                "motor": 0,
                "encoder": 0,
                "controller": 0,
            }
        gcmd.respond_info("ODrive %s: errors cleared" % (self.full_name,))

    def cmd_ODRIVE_ERRORS(self, gcmd):
        self._require_connected(gcmd)
        for axis in sorted(self.axes.values(), key=lambda a: a.axis_index):
            axis.poll_errors_and_telemetry()
            gcmd.respond_info(axis.format_errors())

    def cmd_ODRIVE_READ(self, gcmd):
        self._require_connected(gcmd)
        prop = gcmd.get("PROPERTY")
        value = self.transport.read_property_sync(prop, timeout=2.0)
        if value is None:
            raise gcmd.error(
                "ODrive %s: no response reading '%s'" % (self.full_name, prop)
            )
        gcmd.respond_info("%s = %s" % (prop, value))

    def cmd_ODRIVE_WRITE(self, gcmd):
        self._require_connected(gcmd)
        prop = gcmd.get("PROPERTY")
        value = gcmd.get("VALUE")
        force = gcmd.get_int("FORCE", 0)
        for axis in self.axes.values():
            if axis.armed and axis.owns_property(prop) and not force:
                raise gcmd.error(
                    "Property '%s' is managed by axis '%s' while armed;"
                    " pass FORCE=1 to override" % (prop, axis.name)
                )
        ok = self.transport.write_property_sync(
            prop, value, verify=True, timeout=2.0
        )
        if not ok:
            raise gcmd.error(
                "ODrive %s: write to '%s' could not be verified"
                % (self.full_name, prop)
            )
        gcmd.respond_info("%s = %s (written)" % (prop, value))

    def cmd_ODRIVE_DUMP_CONFIG(self, gcmd):
        self._require_connected(gcmd)
        gcmd.respond_info(
            "ODrive %s: fw=%s hw=%s serial=%s vbus=%.2fV"
            % (
                self.full_name,
                _version_str(self.fw_version) or "unknown",
                _version_str(self.hw_version) or "unknown",
                self.serial_number or "unknown",
                self.vbus_voltage or 0.0,
            )
        )
        for axis in sorted(self.axes.values(), key=lambda a: a.axis_index):
            gcmd.respond_info(axis.format_config_dump())

    def cmd_ODRIVE_SAVE_CONFIG(self, gcmd):
        self._require_connected(gcmd)
        gcmd.respond_info(
            "ODrive %s: saving configuration to NVM (device will reboot"
            " and briefly disconnect)..." % (self.full_name,)
        )
        for axis in self.axes.values():
            if axis.rail is not None:
                gcmd.respond_info(
                    "Warning: axis '%s' will become unhomed after reboot"
                    % (axis.name,)
                )
        self._reboot_pending = True
        self.transport.send_line("ss")
        self.reactor.pause(self.reactor.monotonic() + 0.3)
        for axis in self.axes.values():
            axis.on_emergency_idle()
        self.transport.close()
        self.connected = False
        self._stop_error_poll()
        self.state = TransportState.REBOOT_PENDING
        self._start_reconnect_poll()
        gcmd.respond_info(
            "ODrive %s: reconnecting after save..." % (self.full_name,)
        )

    def cmd_ODRIVE_ERASE_CONFIG(self, gcmd):
        self._require_connected(gcmd)
        if not gcmd.get_int("CONFIRM", 0):
            raise gcmd.error(
                "ODRIVE_ERASE_CONFIG requires CONFIRM=1 -- this wipes all"
                " ODrive-side calibration and configuration"
            )
        self._reboot_pending = True
        self.transport.send_line("se")
        self.reactor.pause(self.reactor.monotonic() + 0.3)
        for axis in self.axes.values():
            axis.on_emergency_idle()
        self.transport.close()
        self.connected = False
        self._stop_error_poll()
        self.state = TransportState.REBOOT_PENDING
        self._start_reconnect_poll()
        gcmd.respond_info(
            "ODrive %s: NVM erased, reconnecting..." % (self.full_name,)
        )

    def cmd_ODRIVE_REBOOT(self, gcmd):
        self._require_connected(gcmd)
        self._reboot_pending = True
        self.transport.send_line("sr")
        self.reactor.pause(self.reactor.monotonic() + 0.3)
        for axis in self.axes.values():
            axis.on_emergency_idle()
        self.transport.close()
        self.connected = False
        self._stop_error_poll()
        self.state = TransportState.REBOOT_PENDING
        self._start_reconnect_poll()
        gcmd.respond_info("ODrive %s: rebooting..." % (self.full_name,))


def _version_str(version):
    if not version:
        return None
    return ".".join(str(v) for v in version)


def load_config_prefix(config):
    return ODriveBoard(config)
