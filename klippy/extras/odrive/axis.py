# ODrive v3.6 per-motor axis support ([odrive_axis <name>])
#
# See docs/ODrive_Implementation_Spec.md. Owns motor/encoder/controller
# configuration, the calibration wizard, arm/disarm interlocks, tuning,
# error/telemetry polling, and (when bound to a [stepper_x]-style rail)
# hands off to odrv_stepper.ODriveRail / streamer.SetpointStreamer. An
# encoder is required -- there is no sensorless (encoder-less) mode.
#
# Copyright (C) 2026  Kalico contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

from . import properties

CALIBRATE_POLL_PERIOD = 0.1
CALIBRATE_TIMEOUT = 60.0

# Properties that are actively managed by the module while an axis is
# armed and bound -- ODRIVE_WRITE refuses to touch these without
# FORCE=1 so a user can't fight the setpoint streamer or calibration
# wizard by accident.
_OWNED_SUFFIXES = (
    "controller.input_pos",
    "controller.input_vel",
    "controller.input_torque",
    "requested_state",
    "controller.config.control_mode",
    "controller.config.input_mode",
)


class ODriveAxis:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.full_name = config.get_name()
        self.name = self.full_name.split()[-1]
        board_name = config.get("odrive")
        self.board = self.printer.load_object(config, "odrive " + board_name)
        self.axis_index = config.getint("axis", minval=0, maxval=1)
        self.board.register_axis(self)

        self.pole_pairs = config.getint("pole_pairs", minval=1)
        self.torque_constant = config.getfloat("torque_constant", above=0.0)
        self.current_lim = config.getfloat("current_lim", 20.0, above=0.0)
        self.calibration_current = config.getfloat(
            "calibration_current", 10.0, above=0.0
        )
        self.motor_type = config.getchoice(
            "motor_type", properties.MOTOR_TYPE_CHOICES, "high_current"
        )
        self.encoder_cpr = config.getint("encoder_cpr", minval=1)
        self.encoder_use_index = config.getboolean("encoder_use_index", False)
        self.encoder_bandwidth = config.getfloat(
            "encoder_bandwidth", 1000.0, above=0.0
        )
        self.pos_gain = config.getfloat("pos_gain", 20.0, above=0.0)
        self.vel_gain = config.getfloat("vel_gain", 0.16, above=0.0)
        self.vel_integrator_gain = config.getfloat(
            "vel_integrator_gain", 0.32, minval=0.0
        )
        self.vel_limit = config.getfloat("vel_limit", 30.0, above=0.0)
        self.input_mode_name = config.getchoice(
            "input_mode", properties.INPUT_MODE_CHOICES, "pos_filter"
        )
        self.filter_bandwidth = config.getfloat(
            "filter_bandwidth", None, above=0.0
        )
        self.enable_thermistor = config.getboolean("enable_thermistor", False)
        self.motor_temp_limit = config.getfloat(
            "motor_temp_limit", 90.0, above=0.0
        )
        self.following_error = config.getfloat(
            "following_error", 1.0, minval=0.0
        )

        self.armed = False
        self.calibrated_motor = False
        self.calibrated_encoder = False
        self.index_found = False
        self.axis_state_value = None
        self.axis_state_name = "unknown"
        self.last_errors = {
            "axis": [],
            "motor": [],
            "encoder": [],
            "controller": [],
        }
        self.last_error_codes = {
            "axis": 0,
            "motor": 0,
            "encoder": 0,
            "controller": 0,
        }
        self.pos_estimate_turns = 0.0
        self.vel_estimate_turns = 0.0
        self.iq_measured = 0.0
        self.iq_setpoint = 0.0
        self.fet_temp = None
        self.motor_temp = None
        self.last_setpoint_turns = 0.0
        self.last_setpoint_time = None
        self.rail = None
        self.streamer = None
        self._watchdog_timer = None
        self._following_error_suspend_until = 0.0

        self._virtual_endstop = None
        ppins = self.printer.lookup_object("pins")
        ppins.register_chip("odrive_axis_%s" % (self.name,), self)

        gcode = self.printer.lookup_object("gcode")
        commands = [
            (
                "ODRIVE_CALIBRATE",
                self.cmd_ODRIVE_CALIBRATE,
                "Run an ODrive motor/encoder calibration sequence",
            ),
            (
                "ODRIVE_INDEX_SEARCH",
                self.cmd_ODRIVE_INDEX_SEARCH,
                "Run ODrive encoder index search",
            ),
            (
                "ODRIVE_ARM",
                self.cmd_ODRIVE_ARM,
                "Enter ODrive closed-loop control",
            ),
            (
                "ODRIVE_DISARM",
                self.cmd_ODRIVE_DISARM,
                "Leave ODrive closed-loop control",
            ),
            (
                "ODRIVE_TUNE",
                self.cmd_ODRIVE_TUNE,
                "Adjust ODrive controller gains/limits",
            ),
            (
                "ODRIVE_AXIS_MOVE",
                self.cmd_ODRIVE_AXIS_MOVE,
                "Standalone on-device move (not for bound axes while printing)",
            ),
            (
                "ODRIVE_WATCHDOG",
                self.cmd_ODRIVE_WATCHDOG,
                "Diagnostic override of the ODrive watchdog",
            ),
        ]
        for cmd, func, desc in commands:
            gcode.register_mux_command(cmd, "AXIS", self.name, func, desc=desc)

        from . import telemetry

        telemetry.register_axis_endpoint(self)
        self.printer.register_event_handler(
            "stepper_enable:motor_off", self._handle_motor_off
        )

    def _handle_motor_off(self, print_time):
        if self.armed:
            self.on_emergency_idle()

    def prop(self, suffix):
        return "axis%d.%s" % (self.axis_index, suffix)

    # Pin chip interface ("odrive_axis_<name>:virtual_endstop"), modeled
    # on TMCVirtualPinHelper in klippy/extras/tmc.py
    def setup_pin(self, pin_type, pin_params):
        ppins = self.printer.lookup_object("pins")
        if pin_type != "endstop" or pin_params["pin"] != "virtual_endstop":
            raise ppins.error("odrive_axis virtual pin only useful as endstop")
        if pin_params["invert"] or pin_params["pullup"]:
            raise ppins.error(
                "Can not pullup/invert odrive_axis virtual endstop pin"
            )
        if self._virtual_endstop is None:
            from .endstop import ODriveVirtualEndstop

            self._virtual_endstop = ODriveVirtualEndstop(self.printer, self)
        return self._virtual_endstop

    def owns_property(self, raw_property):
        prefix = "axis%d." % (self.axis_index,)
        if not raw_property.startswith(prefix):
            return False
        suffix = raw_property[len(prefix) :]
        return suffix in _OWNED_SUFFIXES

    # Status
    def get_status(self, eventtime=None):
        return {
            "axis_state": self.axis_state_name,
            "armed": self.armed,
            "calibrated": self.calibrated_motor and self.calibrated_encoder,
            "pre_calibrated_motor": self.calibrated_motor,
            "pre_calibrated_encoder": self.calibrated_encoder,
            "index_found": self.index_found,
            "errors": dict(self.last_errors),
            "pos_estimate": self._turns_to_mm(self.pos_estimate_turns),
            "vel_estimate": self._turns_rate_to_mm(self.vel_estimate_turns),
            "pos_error": self._pos_error_mm(),
            "iq_measured": self.iq_measured,
            "iq_setpoint": self.iq_setpoint,
            "fet_temp": self.fet_temp,
            "motor_temp": self.motor_temp,
            "pos_gain": self.pos_gain,
            "vel_gain": self.vel_gain,
            "vel_integrator_gain": self.vel_integrator_gain,
            "filter_bandwidth": self.filter_bandwidth,
            "current_lim": self.current_lim,
            "vel_limit": self.vel_limit,
        }

    def _turns_to_mm(self, turns):
        if self.rail is None:
            return turns
        return self.rail.stepper.turns_to_mm(turns)

    def _turns_rate_to_mm(self, turns_rate):
        # Scale-only conversion for velocities: the coordinate-resync
        # offset used by _turns_to_mm applies to positions, not rates
        if self.rail is None:
            return turns_rate
        stepper = self.rail.stepper
        return turns_rate * stepper.rotation_distance / stepper.direction

    def _pos_error_mm(self):
        if self.rail is None:
            return None
        return self._turns_to_mm(self.pos_estimate_turns) - self._turns_to_mm(
            self.last_setpoint_turns
        )

    def format_status_line(self):
        errs = (
            ", ".join(n for names in self.last_errors.values() for n in names)
            or "none"
        )
        return (
            "  axis %s (M%d): state=%s armed=%s calibrated=%s/%s"
            " index_found=%s pos=%.4f errors=%s"
            % (
                self.name,
                self.axis_index,
                self.axis_state_name,
                self.armed,
                self.calibrated_motor,
                self.calibrated_encoder,
                self.index_found,
                self._turns_to_mm(self.pos_estimate_turns),
                errs,
            )
        )

    def format_errors(self):
        parts = []
        for category, names in self.last_errors.items():
            code = self.last_error_codes[category]
            parts.append(
                "%s=0x%X(%s)" % (category, code, ",".join(names) or "none")
            )
        return "axis %s: %s" % (self.name, " ".join(parts))

    def format_config_dump(self):
        base = (
            "axis %s (M%d): pole_pairs=%d torque_constant=%.4f"
            " current_lim=%.2f pos_gain=%.3f vel_gain=%.4f"
            " vel_integrator_gain=%.4f vel_limit=%.2f"
            % (
                self.name,
                self.axis_index,
                self.pole_pairs,
                self.torque_constant,
                self.current_lim,
                self.pos_gain,
                self.vel_gain,
                self.vel_integrator_gain,
                self.vel_limit,
            )
        )
        return base + " encoder_cpr=%d input_mode=%s" % (
            self.encoder_cpr,
            self.input_mode_name,
        )

    # Rail binding (called via stepper.LookupMultiRail hook)
    def lookup_rail(
        self, config, need_position_minmax, default_position_endstop
    ):
        from . import odrv_stepper

        if self.rail is not None:
            raise config.error(
                "ODrive axis '%s' is already bound to rail '%s'"
                % (self.name, self.rail.get_name())
            )
        rail = odrv_stepper.ODriveRail(
            config, self, need_position_minmax, default_position_endstop
        )
        self.rail = rail
        return rail

    def is_motion_critical(self):
        if self.rail is None:
            return False
        toolhead = self.printer.lookup_object("toolhead", None)
        if toolhead is None:
            return False
        return self.rail.stepper.is_homed() and self.armed

    # Config push (called by ODriveBoard during CONFIGURING)
    def push_config(self):
        t = self.board.transport
        t.write_property(self.prop("motor.config.pole_pairs"), self.pole_pairs)
        t.write_property(
            self.prop("motor.config.calibration_current"),
            self.calibration_current,
        )
        t.write_property(
            self.prop("motor.config.current_lim"), self.current_lim
        )
        t.write_property(
            self.prop("motor.config.torque_constant"), self.torque_constant
        )
        t.write_property(self.prop("motor.config.motor_type"), self.motor_type)
        t.write_property(self.prop("encoder.config.cpr"), self.encoder_cpr)
        t.write_property(
            self.prop("encoder.config.use_index"),
            int(self.encoder_use_index),
        )
        t.write_property(
            self.prop("encoder.config.bandwidth"), self.encoder_bandwidth
        )
        t.write_property(self.prop("controller.config.pos_gain"), self.pos_gain)
        t.write_property(self.prop("controller.config.vel_gain"), self.vel_gain)
        t.write_property(
            self.prop("controller.config.vel_integrator_gain"),
            self.vel_integrator_gain,
        )
        t.write_property(
            self.prop("controller.config.vel_limit"), self.vel_limit
        )
        t.write_property(
            self.prop("controller.config.control_mode"),
            properties.CONTROL_MODE_POSITION_CONTROL,
        )
        # self.input_mode_name already holds the *resolved* enum
        # value -- config.getchoice() returns choices[matched_string],
        # not the string itself -- so re-indexing INPUT_MODE_CHOICES
        # with it here raised KeyError on every push_config()/
        # ODRIVE_ARM call for any axis with at least one configured
        # axis (caught by scripts/odrive_mock.py, which is the first
        # thing to ever actually drive this code path end to end).
        input_mode = self.input_mode_name
        t.write_property(self.prop("controller.config.input_mode"), input_mode)
        fb = self.filter_bandwidth
        if fb is None:
            fb = 0.5 / self.board.sample_period
        t.write_property(
            self.prop("controller.config.input_filter_bandwidth"), fb
        )
        t.write_property(
            self.prop("config.watchdog_timeout"), self.board.watchdog_timeout
        )
        readback = t.read_property_sync(
            self.prop("motor.config.pole_pairs"), timeout=2.0
        )
        try:
            ok = int(float(readback)) == self.pole_pairs
        except (TypeError, ValueError):
            ok = False
        if not ok:
            logging.warning(
                "odrive_axis %s: config push could not be verified"
                " (pole_pairs read-back mismatch)",
                self.full_name,
            )
        self.calibrated_motor = self._read_bool(
            self.prop("motor.config.pre_calibrated")
        )
        self.calibrated_encoder = self._read_bool(
            self.prop("encoder.config.pre_calibrated")
        )
        self.index_found = self._read_bool(self.prop("encoder.index_found"))

    def _read_bool(self, path):
        raw = self.board.transport.read_property_sync(path, timeout=1.0)
        if raw is None:
            return False
        try:
            return bool(int(float(raw)))
        except (TypeError, ValueError):
            return raw.strip().lower() in ("true", "1")

    # Connection lifecycle callbacks from ODriveBoard
    def on_disconnected(self):
        self.armed = False
        self._stop_watchdog_feed()
        if self.streamer is not None:
            self.streamer.stop()
        if self.rail is not None:
            self.rail.stepper.clear_homed()

    def on_reconnected(self):
        self.push_config()
        if self.rail is not None:
            self.rail.stepper.clear_homed()

    def on_emergency_idle(self):
        if self.board.connected:
            try:
                self.board.transport.write_property(
                    self.prop("requested_state"), properties.AXIS_STATE_IDLE
                )
            except Exception:
                pass
        self.armed = False
        self._stop_watchdog_feed()
        if self.streamer is not None:
            self.streamer.stop()

    # Fault handling
    def handle_fault(self, message):
        logging.warning("odrive_axis %s: fault: %s", self.full_name, message)
        if self.is_motion_critical():
            self.printer.invoke_shutdown(
                "ODrive axis %s: %s" % (self.name, message)
            )
            return
        self.on_emergency_idle()
        gcode = self.printer.lookup_object("gcode")
        gcode.respond_info(
            "ODrive axis %s fault (motion not affected): %s"
            % (self.name, message)
        )

    # Error / telemetry polling (called from ODriveBoard's poll timer;
    # runs via query_sync, safe from a reactor timer callback because
    # reactor.pause() transparently spawns a helper dispatch greenlet)
    def poll_errors_and_telemetry(self):
        t = self.board.transport
        if not self.board.connected:
            return
        for category, suffix in (
            ("axis", "error"),
            ("motor", "motor.error"),
            ("encoder", "encoder.error"),
            ("controller", "controller.error"),
        ):
            val = t.read_property_sync(self.prop(suffix), timeout=1.0)
            try:
                code = int(float(val)) if val is not None else 0
            except ValueError:
                code = 0
            self.last_error_codes[category] = code
            self.last_errors[category] = properties.decode_bitfield(
                code, properties.ERROR_TABLES[category]
            )
        state_raw = t.read_property_sync(
            self.prop("current_state"), timeout=1.0
        )
        try:
            self.axis_state_value = int(float(state_raw))
        except (TypeError, ValueError):
            self.axis_state_value = None
        self.axis_state_name = properties.axis_state_name(self.axis_state_value)
        fb = t.query_sync("f %d" % (self.axis_index,), timeout=1.0)
        if fb:
            parts = fb.split()
            if len(parts) >= 2:
                try:
                    self.pos_estimate_turns = float(parts[0])
                    self.vel_estimate_turns = float(parts[1])
                except ValueError:
                    pass
        iq = t.read_property_sync(
            self.prop("motor.current_control.Iq_measured"), timeout=1.0
        )
        try:
            self.iq_measured = float(iq) if iq is not None else self.iq_measured
        except ValueError:
            pass
        fet_temp = t.read_property_sync(
            self.prop("motor.fet_thermistor.temperature"), timeout=1.0
        )
        try:
            self.fet_temp = float(fet_temp) if fet_temp is not None else None
        except ValueError:
            self.fet_temp = None
        if self.fet_temp is not None and self.fet_temp > 95.0 and self.armed:
            self.handle_fault(
                "FET temperature %.1fC exceeds 95C" % (self.fet_temp,)
            )
        if any(self.last_error_codes.values()) and self.armed:
            self.handle_fault(
                "error flags set: %s"
                % (
                    " ".join(
                        "%s=0x%X" % (k, v)
                        for k, v in self.last_error_codes.items()
                        if v
                    )
                )
            )
        self._check_following_error()

    def _check_following_error(self):
        if (
            not self.armed
            or self.following_error <= 0.0
            or self.rail is None
            or self.reactor.monotonic() < self._following_error_suspend_until
        ):
            return
        err = abs(self._pos_error_mm() or 0.0)
        if err > self.following_error:
            self.handle_fault(
                "following error %.3fmm exceeds limit %.3fmm"
                % (err, self.following_error)
            )

    def note_setpoint_sent(self, turns):
        self.last_setpoint_turns = turns
        self.last_setpoint_time = self.reactor.monotonic()

    # Watchdog feed (idle-hold / non-streaming safety net; the setpoint
    # streamer's own "p" traffic feeds the watchdog at a much higher
    # rate whenever it is actively streaming)
    def _start_watchdog_feed(self):
        if self.board.watchdog_timeout <= 0.0:
            return
        period = min(
            self.board.idle_feed_period, self.board.watchdog_timeout / 3.0
        )
        if self._watchdog_timer is None:
            self._watchdog_timer = self.reactor.register_timer(
                self._watchdog_feed_tick
            )
        self._watchdog_period = period
        self.reactor.update_timer(self._watchdog_timer, self.reactor.NOW)

    def _stop_watchdog_feed(self):
        if self._watchdog_timer is not None:
            self.reactor.update_timer(self._watchdog_timer, self.reactor.NEVER)

    def _watchdog_feed_tick(self, eventtime):
        if not self.armed or not self.board.connected:
            return self.reactor.NEVER
        # A bare "u" feed leaves the setpoint alone -- unlike re-sending
        # the last p/v setpoint, which (confirmed on real 0.5.1
        # hardware) faults the legacy AXIS_STATE_SENSORLESS_CONTROL
        # state almost immediately.
        self.board.transport.feed_watchdog(self.axis_index)
        return eventtime + self._watchdog_period

    # GCode commands
    def _require_connected(self, gcmd):
        if not self.board.connected:
            raise gcmd.error(
                "ODrive %s is not connected" % (self.board.full_name,)
            )

    cmd_ODRIVE_CALIBRATE_help = (
        "Run an ODrive motor/encoder calibration sequence"
    )

    def cmd_ODRIVE_CALIBRATE(self, gcmd):
        self._require_connected(gcmd)
        cal_type = gcmd.get("TYPE", "full").lower()
        if cal_type not in properties.CALIBRATE_TYPE_STATES:
            raise gcmd.error(
                "Unknown TYPE '%s' (expected one of %s)"
                % (cal_type, ", ".join(properties.CALIBRATE_TYPE_STATES))
            )
        if self.armed:
            raise gcmd.error(
                "Disarm axis '%s' before calibrating (ODRIVE_DISARM)"
                % (self.name,)
            )
        t = self.board.transport
        prev_watchdog = t.read_property_sync(
            self.prop("config.enable_watchdog"), timeout=1.0
        )
        t.write_property(self.prop("config.enable_watchdog"), 0)
        target_state = properties.CALIBRATE_TYPE_STATES[cal_type]
        gcmd.respond_info(
            "ODrive axis %s: starting %s calibration..." % (self.name, cal_type)
        )
        t.write_property(self.prop("requested_state"), target_state)
        deadline = self.reactor.monotonic() + CALIBRATE_TIMEOUT
        last_state = None
        while True:
            now = self.reactor.monotonic()
            if now > deadline:
                raise gcmd.error(
                    "ODrive axis %s: calibration timed out" % (self.name,)
                )
            state_raw = t.read_property_sync(
                self.prop("current_state"), timeout=1.0
            )
            try:
                state = int(float(state_raw))
            except (TypeError, ValueError):
                state = None
            if state != last_state and state is not None:
                gcmd.respond_info(
                    "  -> %s" % (properties.axis_state_name(state),)
                )
                last_state = state
            if state == properties.AXIS_STATE_IDLE and last_state is not None:
                break
            self.reactor.pause(now + CALIBRATE_POLL_PERIOD)
        if prev_watchdog is not None:
            t.write_property(self.prop("config.enable_watchdog"), prev_watchdog)
        self.poll_errors_and_telemetry()
        if any(self.last_error_codes.values()):
            raise gcmd.error(
                "ODrive axis %s: calibration failed: %s"
                % (self.name, self.format_errors())
            )
        if cal_type in ("full", "motor"):
            self.calibrated_motor = True
            t.write_property(self.prop("motor.config.pre_calibrated"), 1)
        if cal_type in ("full", "encoder_offset"):
            self.calibrated_encoder = True
            t.write_property(self.prop("encoder.config.pre_calibrated"), 1)
        if cal_type == "index":
            self.index_found = self._read_bool(self.prop("encoder.index_found"))
        gcmd.respond_info(
            "ODrive axis %s: %s calibration successful.\n"
            "Run ODRIVE_SAVE_CONFIG ODRIVE=%s to persist to ODrive NVM."
            % (self.name, cal_type, self.board.name)
        )
        configfile = self.printer.lookup_object("configfile")
        configfile.set(self.full_name, "pole_pairs", str(self.pole_pairs))

    cmd_ODRIVE_INDEX_SEARCH_help = "Run ODrive encoder index search"

    def cmd_ODRIVE_INDEX_SEARCH(self, gcmd):
        gcmd_params = dict(gcmd.get_command_parameters())
        gcmd_params["TYPE"] = "index"
        gcmd2 = self.printer.lookup_object("gcode").create_gcode_command(
            "ODRIVE_CALIBRATE", "ODRIVE_CALIBRATE", gcmd_params
        )
        self.cmd_ODRIVE_CALIBRATE(gcmd2)

    cmd_ODRIVE_ARM_help = "Enter ODrive closed-loop control"

    def cmd_ODRIVE_ARM(self, gcmd):
        self._require_connected(gcmd)
        force = gcmd.get_int("FORCE", 0)
        self.poll_errors_and_telemetry()
        if any(self.last_error_codes.values()) and not force:
            raise gcmd.error(
                "ODrive axis %s: cannot arm, errors present: %s"
                " (ODRIVE_CLEAR_ERRORS or FORCE=1)"
                % (self.name, self.format_errors())
            )
        if (
            not (self.calibrated_motor and self.calibrated_encoder)
            and not force
        ):
            raise gcmd.error(
                "ODrive axis %s: not calibrated (ODRIVE_CALIBRATE or FORCE=1)"
                % (self.name,)
            )
        if self.board.vbus_voltage is not None and not (
            self.board.vbus_min
            <= self.board.vbus_voltage
            <= self.board.vbus_max
        ):
            raise gcmd.error(
                "ODrive %s: bus voltage %.2fV out of range"
                % (self.board.full_name, self.board.vbus_voltage)
            )
        t = self.board.transport
        if self.board.watchdog_timeout > 0.0:
            t.write_property(
                self.prop("config.watchdog_timeout"),
                self.board.watchdog_timeout,
            )
            t.write_property(self.prop("config.enable_watchdog"), 1)
        # Defensively reassert the configured input_mode: a prior
        # ODRIVE_AXIS_MOVE leaves the ODrive in TRAP_TRAJ (which must
        # stay active for the duration of that on-device move), and the
        # streamer's high-rate "p" setpoints require pos_filter/
        # passthrough instead -- switching TRAP_TRAJ back immediately
        # after writing input_pos would abort the trapezoidal profile.
        # self.input_mode_name already holds the *resolved* enum value --
        # config.getchoice() returns choices[matched_string], not the
        # string itself -- so re-indexing INPUT_MODE_CHOICES with it here
        # raised KeyError on every push_config()/ODRIVE_ARM call for any
        # axis with at least one configured axis (caught by scripts/
        # odrive_mock.py, which is the first thing to ever actually drive
        # this code path end to end).
        input_mode = self.input_mode_name
        t.write_property(self.prop("controller.config.input_mode"), input_mode)
        self.last_setpoint_turns = self.pos_estimate_turns
        t.write_property(
            self.prop("controller.input_pos"), self.last_setpoint_turns
        )
        t.write_property(
            self.prop("requested_state"),
            properties.AXIS_STATE_CLOSED_LOOP_CONTROL,
        )
        self.armed = True
        self._following_error_suspend_until = self.reactor.monotonic() + 0.5
        self._start_watchdog_feed()
        if self.streamer is not None:
            self.streamer.start()
        gcmd.respond_info("ODrive axis %s: armed" % (self.name,))

    cmd_ODRIVE_DISARM_help = "Leave ODrive closed-loop control"

    def cmd_ODRIVE_DISARM(self, gcmd):
        self.on_emergency_idle()
        gcmd.respond_info("ODrive axis %s: disarmed" % (self.name,))

    cmd_ODRIVE_TUNE_help = "Adjust ODrive controller gains/limits"

    def cmd_ODRIVE_TUNE(self, gcmd):
        self._require_connected(gcmd)
        t = self.board.transport
        updates = []
        pos_gain = gcmd.get_float("POS_GAIN", None, above=0.0)
        if pos_gain is not None:
            self.pos_gain = pos_gain
            updates.append(
                ("pos_gain", self.prop("controller.config.pos_gain"), pos_gain)
            )
        vel_gain = gcmd.get_float("VEL_GAIN", None, above=0.0)
        if vel_gain is not None:
            self.vel_gain = vel_gain
            updates.append(
                ("vel_gain", self.prop("controller.config.vel_gain"), vel_gain)
            )
        vel_integrator_gain = gcmd.get_float(
            "VEL_INTEGRATOR_GAIN", None, minval=0.0
        )
        if vel_integrator_gain is not None:
            self.vel_integrator_gain = vel_integrator_gain
            updates.append(
                (
                    "vel_integrator_gain",
                    self.prop("controller.config.vel_integrator_gain"),
                    vel_integrator_gain,
                )
            )
        filter_bandwidth = gcmd.get_float("FILTER_BANDWIDTH", None, above=0.0)
        if filter_bandwidth is not None:
            self.filter_bandwidth = filter_bandwidth
            updates.append(
                (
                    "filter_bandwidth",
                    self.prop("controller.config.input_filter_bandwidth"),
                    filter_bandwidth,
                )
            )
        current_lim = gcmd.get_float("CURRENT_LIM", None, above=0.0)
        if current_lim is not None:
            self.current_lim = current_lim
            updates.append(
                (
                    "current_lim",
                    self.prop("motor.config.current_lim"),
                    current_lim,
                )
            )
        vel_limit = gcmd.get_float("VEL_LIMIT", None, above=0.0)
        if vel_limit is not None:
            self.vel_limit = vel_limit
            updates.append(
                (
                    "vel_limit",
                    self.prop("controller.config.vel_limit"),
                    vel_limit,
                )
            )
        for _, path, value in updates:
            t.write_property(path, value)
        if not updates:
            gcmd.respond_info(self.format_config_dump())
            return
        msg = ", ".join(
            "%s=%s" % (name, value) for name, path, value in updates
        )
        gcmd.respond_info("ODrive axis %s: tuned %s" % (self.name, msg))
        if gcmd.get_int("SAVE", 0):
            configfile = self.printer.lookup_object("configfile")
            for name, _, value in updates:
                configfile.set(self.full_name, name, str(value))
            gcmd.respond_info(
                "The SAVE_CONFIG command will update the printer config"
                " file and restart."
            )

    cmd_ODRIVE_AXIS_MOVE_help = "Standalone on-device move"

    def cmd_ODRIVE_AXIS_MOVE(self, gcmd):
        self._require_connected(gcmd)
        if not self.armed:
            raise gcmd.error("ODrive axis %s: not armed" % (self.name,))
        if self.rail is not None and self.rail.stepper.is_homed():
            toolhead = self.printer.lookup_object("toolhead")
            if (
                toolhead.get_status(self.reactor.monotonic()).get(
                    "print_time", 0
                )
                > 0
            ):
                raise gcmd.error(
                    "ODrive axis %s is bound to a homed rail; use normal"
                    " toolhead moves instead of ODRIVE_AXIS_MOVE" % (self.name,)
                )
        pos_mm = gcmd.get_float("POS", None)
        pos_turns_param = gcmd.get_float("TURNS", None)
        if (pos_mm is None) == (pos_turns_param is None):
            raise gcmd.error(
                "Specify exactly one of POS (mm) or TURNS (motor turns)"
            )
        if pos_mm is not None:
            if self.rail is None:
                # Without a bound rail there is no rotation_distance to
                # convert mm to motor turns -- silently sending mm as
                # turns would move an arbitrary (likely huge) distance
                raise gcmd.error(
                    "ODrive axis %s is not bound to a rail; use TURNS="
                    " to move in raw motor turns" % (self.name,)
                )
            pos_turns = self.rail.stepper.mm_to_turns(pos_mm)
            target_desc = "%.4fmm" % (pos_mm,)
        else:
            pos_turns = pos_turns_param
            target_desc = "%.4f turns" % (pos_turns_param,)
        vel = gcmd.get_float("VEL", None, above=0.0)
        t = self.board.transport
        if vel is not None:
            t.write_property(self.prop("controller.config.vel_limit"), vel)
        t.write_property(
            self.prop("controller.config.input_mode"),
            properties.INPUT_MODE_TRAP_TRAJ,
        )
        t.write_property(self.prop("controller.input_pos"), pos_turns)
        self.last_setpoint_turns = pos_turns
        gcmd.respond_info(
            "ODrive axis %s: moving to %s (input_mode stays TRAP_TRAJ"
            " until the next ODRIVE_ARM)" % (self.name, target_desc)
        )

    cmd_ODRIVE_WATCHDOG_help = "Diagnostic override of the ODrive watchdog"

    def cmd_ODRIVE_WATCHDOG(self, gcmd):
        self._require_connected(gcmd)
        enable = gcmd.get_int("ENABLE")
        timeout = gcmd.get_float("TIMEOUT", None, minval=0.25)
        t = self.board.transport
        if timeout is not None:
            t.write_property(self.prop("config.watchdog_timeout"), timeout)
        t.write_property(self.prop("config.enable_watchdog"), int(bool(enable)))
        gcmd.respond_info(
            "ODrive axis %s: watchdog %s%s"
            % (
                self.name,
                "enabled" if enable else "disabled",
                (" (timeout=%.2fs)" % timeout) if timeout is not None else "",
            )
        )
