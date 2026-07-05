# ODrive v3.6 homing endstops
#
# Two homing paths, both described in docs/ODrive_Implementation_Spec.md
# ("Homing design"):
#
#  - PhysicalEndstopWrapper: a real switch wired to a Klipper MCU pin.
#    Delegates to the real MCU_endstop for trigger-sync accuracy, but
#    keeps ODrive steppers (which have no firmware oid) out of the
#    trsync stepper list.
#  - ODriveVirtualEndstop: a purely host-driven "sensorless" endstop
#    that polls ODrive position error / measured current during the
#    homing move and completes a plain reactor completion when the
#    trigger condition holds for enough consecutive samples. Modeled on
#    klippy/extras/probe_eddy_current.py's EddyEndstopWrapper, but with
#    no MCU trigger-sync hardware behind it at all.
#
# Copyright (C) 2026  Kalico contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.

POLL_TIMEOUT_MARGIN = 0.5


class PhysicalEndstopWrapper:
    def __init__(self, mcu_endstop):
        self._mcu_endstop = mcu_endstop
        self._odrive_steppers = []

    def __getattr__(self, name):
        # Delegate anything we don't explicitly implement (e.g. an
        # optional get_position_endstop()) straight to the wrapped
        # MCU_endstop, so hasattr() checks against this wrapper reflect
        # the underlying object's real capabilities.
        return getattr(self._mcu_endstop, name)

    def get_mcu(self):
        return self._mcu_endstop.get_mcu()

    def add_stepper(self, stepper):
        if hasattr(stepper, "get_oid"):
            self._mcu_endstop.add_stepper(stepper)
        else:
            if stepper not in self._odrive_steppers:
                self._odrive_steppers.append(stepper)

    def get_steppers(self):
        return list(self._mcu_endstop.get_steppers()) + list(
            self._odrive_steppers
        )

    def home_start(
        self, print_time, sample_time, sample_count, rest_time, triggered=True
    ):
        return self._mcu_endstop.home_start(
            print_time,
            sample_time,
            sample_count,
            rest_time,
            triggered=triggered,
        )

    def home_wait(self, home_end_time):
        return self._mcu_endstop.home_wait(home_end_time)

    def query_endstop(self, print_time):
        return self._mcu_endstop.query_endstop(print_time)


class ODriveVirtualEndstop:
    def __init__(self, printer, axis):
        self.printer = printer
        self.reactor = printer.get_reactor()
        self.axis = axis
        self._steppers = []
        self._poll_timer = None
        self._completion = None
        self._trigger_print_time = 0.0
        self._prev_current_lim = None
        self._consecutive = 0
        self._pos_error_threshold = 0.5
        self._current_threshold = 0.0
        self._trigger_count = 2
        self._poll_period = 0.010
        self._trigger_latency = 0.005
        printer.register_event_handler(
            "homing:homing_move_begin", self._handle_homing_move_begin
        )
        printer.register_event_handler(
            "homing:homing_move_end", self._handle_homing_move_end
        )

    def get_mcu(self):
        return self.axis.board

    def add_stepper(self, stepper):
        if stepper not in self._steppers:
            self._steppers.append(stepper)

    def get_steppers(self):
        return list(self._steppers)

    def home_start(
        self, print_time, sample_time, sample_count, rest_time, triggered=True
    ):
        cfg = self.axis.rail.homing_cfg
        self._pos_error_threshold = cfg["pos_error_threshold"]
        self._current_threshold = cfg["current_threshold"]
        self._trigger_count = cfg["trigger_count"]
        self._poll_period = cfg["poll_period"]
        self._trigger_latency = cfg["trigger_latency"]
        self._consecutive = 0
        self._trigger_print_time = 0.0
        self._completion = self.reactor.completion()
        if self._poll_timer is None:
            self._poll_timer = self.reactor.register_timer(self._poll_tick)
        self.reactor.update_timer(self._poll_timer, self.reactor.NOW)
        return self._completion

    def _poll_tick(self, eventtime):
        if self._completion is None or self._completion.test():
            return self.reactor.NEVER
        axis = self.axis
        t = axis.board.transport
        timeout = self._poll_period + POLL_TIMEOUT_MARGIN
        fb = t.query_sync("f %d" % (axis.axis_index,), timeout=timeout)
        triggered = False
        if fb:
            parts = fb.split()
            if len(parts) >= 2:
                try:
                    pos = float(parts[0])
                    vel = float(parts[1])
                except ValueError:
                    pos = None
                if pos is not None:
                    axis.pos_estimate_turns = pos
                    axis.vel_estimate_turns = vel
                    pos_err_turns = abs(pos - axis.last_setpoint_turns)
                    pos_err_mm = (
                        pos_err_turns * axis.rail.stepper.rotation_distance
                    )
                    if pos_err_mm > self._pos_error_threshold:
                        triggered = True
        if not triggered and self._current_threshold > 0.0:
            iq = t.read_property_sync(
                axis.prop("motor.current_control.Iq_measured"), timeout=timeout
            )
            try:
                if iq is not None and abs(float(iq)) > self._current_threshold:
                    triggered = True
            except ValueError:
                pass
        self._consecutive = self._consecutive + 1 if triggered else 0
        if self._consecutive >= self._trigger_count:
            rx_time = self.reactor.monotonic()
            self._trigger_print_time = max(
                0.0,
                axis.board.estimated_print_time(rx_time)
                - self._trigger_latency,
            )
            self._completion.complete(1)
            return self.reactor.NEVER
        return eventtime + self._poll_period

    def home_wait(self, home_end_time):
        if self._poll_timer is not None:
            self.reactor.update_timer(self._poll_timer, self.reactor.NEVER)
        if self._completion is not None:
            self._completion.wait(home_end_time)
        result = self._trigger_print_time
        self._completion = None
        return result

    def query_endstop(self, print_time):
        return 0

    # Sensorless homing current reduction, mirroring
    # TMCVirtualPinHelper.handle_homing_move_begin/end in extras/tmc.py
    def _handle_homing_move_begin(self, hmove):
        if self not in hmove.get_mcu_endstops():
            return
        homing_current = self.axis.rail.homing_cfg.get("homing_current")
        if homing_current is None:
            return
        self._prev_current_lim = self.axis.current_lim
        self.axis.board.transport.write_property(
            self.axis.prop("motor.config.current_lim"), homing_current
        )

    def _handle_homing_move_end(self, hmove):
        if self not in hmove.get_mcu_endstops():
            return
        if self._prev_current_lim is not None:
            self.axis.board.transport.write_property(
                self.axis.prop("motor.config.current_lim"),
                self._prev_current_lim,
            )
            self._prev_current_lim = None
