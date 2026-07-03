# ODrive-backed rail/stepper duck-types
#
# See docs/ODrive_Implementation_Spec.md, section "The ODriveStepper /
# ODriveRail duck-types". ODriveStepper mirrors the public surface of
# klippy/stepper.py's MCU_stepper (and ODriveRail mirrors PrinterRail)
# closely enough that every kinematics class in klippy/kinematics/*.py
# works completely unmodified. The key trick: ODriveStepper allocates a
# real itersolve stepper_kinematics object (so coordinate math for any
# kinematics -- cartesian, corexy, delta, ... -- is exactly correct)
# but never attaches a stepcompress sink and never calls
# itersolve_generate_steps, so there is nowhere for "steps" to go --
# the trapq is instead sampled by streamer.SetpointStreamer.
#
# Copyright (C) 2026  Kalico contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections

from ... import chelper


class ODriveStepper:
    def __init__(self, config, axis):
        self.printer = config.get_printer()
        self.axis = axis
        self.full_name = config.get_name()
        self.rotation_distance = config.getfloat("rotation_distance", above=0.0)
        direction = config.getint("odrive_direction", 1)
        if direction not in (1, -1):
            raise config.error(
                "odrive_direction in section '%s' must be 1 or -1"
                % (self.full_name,)
            )
        self.direction = direction
        self.virtual_steps_per_rotation = config.getint(
            "virtual_steps_per_rotation", 4000, minval=1
        )
        self.step_dist = (
            self.rotation_distance / self.virtual_steps_per_rotation
        )
        ffi_main, ffi_lib = chelper.get_ffi()
        self._ffi_main = ffi_main
        self._ffi_lib = ffi_lib
        self._itersolve_check_active = ffi_lib.itersolve_check_active
        self._stepper_kinematics = None
        self._trapq = ffi_main.NULL
        self._mcu_position_offset = 0.0
        self._homed = False
        self._tmc_current_helper = None
        self._active_callbacks = []

    def get_tmc_current_helper(self):
        return self._tmc_current_helper

    def set_tmc_current_helper(self, tmc_current_helper):
        self._tmc_current_helper = tmc_current_helper

    def get_mcu(self):
        return self.axis.board

    def get_name(self, short=False):
        if short and self.full_name.startswith("stepper_"):
            return self.full_name[8:]
        return self.full_name

    def units_in_radians(self):
        return False

    def get_pulse_duration(self):
        return None, False

    def setup_default_pulse_duration(self, pulse_duration, step_both_edge):
        pass

    def setup_itersolve(self, alloc_func, *params):
        sk = self._ffi_main.gc(
            getattr(self._ffi_lib, alloc_func)(*params), self._ffi_lib.free
        )
        self.set_stepper_kinematics(sk)

    def get_step_dist(self):
        return self.step_dist

    def get_rotation_distance(self):
        return self.rotation_distance, self.virtual_steps_per_rotation

    def set_rotation_distance(self, rotation_dist):
        mcu_pos = self.get_mcu_position()
        self.rotation_distance = rotation_dist
        self.step_dist = rotation_dist / self.virtual_steps_per_rotation
        self._set_mcu_position(mcu_pos)

    def get_dir_inverted(self):
        inverted = 0 if self.direction == 1 else 1
        return inverted, inverted

    def calc_position_from_coord(self, coord):
        return self._ffi_lib.itersolve_calc_position_from_coord(
            self._stepper_kinematics, coord[0], coord[1], coord[2]
        )

    def set_position(self, coord):
        mcu_pos = self.get_mcu_position()
        self._ffi_lib.itersolve_set_position(
            self._stepper_kinematics, coord[0], coord[1], coord[2]
        )
        self._set_mcu_position(mcu_pos)
        if self.axis.streamer is not None:
            self.axis.streamer.note_position_set(coord)

    def get_commanded_position(self):
        return self._ffi_lib.itersolve_get_commanded_pos(
            self._stepper_kinematics
        )

    def get_mcu_position(self, cmd_pos=None):
        if cmd_pos is None:
            cmd_pos = self.get_commanded_position()
        mcu_pos_dist = cmd_pos + self._mcu_position_offset
        mcu_pos = mcu_pos_dist / self.step_dist
        if mcu_pos >= 0.0:
            return int(mcu_pos + 0.5)
        return int(mcu_pos - 0.5)

    def _set_mcu_position(self, mcu_pos):
        mcu_pos_dist = mcu_pos * self.step_dist
        self._mcu_position_offset = mcu_pos_dist - self.get_commanded_position()

    def get_past_mcu_position(self, print_time):
        pos_mm = None
        if self.axis.streamer is not None:
            pos_mm = self.axis.streamer.get_ring_buffer_position_mm(print_time)
        if pos_mm is None:
            return self.get_mcu_position()
        return self.get_mcu_position(pos_mm)

    def mcu_to_commanded_position(self, mcu_pos):
        return mcu_pos * self.step_dist - self._mcu_position_offset

    def get_stepper_kinematics(self):
        return self._stepper_kinematics

    def set_stepper_kinematics(self, sk):
        old_sk = self._stepper_kinematics
        mcu_pos = 0
        if old_sk is not None:
            mcu_pos = self.get_mcu_position()
        self._stepper_kinematics = sk
        self.set_trapq(self._trapq)
        self._set_mcu_position(mcu_pos)
        return old_sk

    def note_homing_end(self):
        self._homed = True

    def is_homed(self):
        return self._homed

    def clear_homed(self):
        self._homed = False

    def get_trapq(self):
        return self._trapq

    def set_trapq(self, tq):
        if tq is None:
            tq = self._ffi_main.NULL
        self._ffi_lib.itersolve_set_trapq(self._stepper_kinematics, tq)
        old_tq = self._trapq
        self._trapq = tq
        if self.axis.streamer is not None:
            self.axis.streamer.set_trapq(tq)
        return old_tq

    def add_active_callback(self, cb):
        self._active_callbacks.append(cb)

    def generate_steps(self, flush_time):
        if self._active_callbacks:
            ret = self._itersolve_check_active(
                self._stepper_kinematics, flush_time
            )
            if ret:
                cbs = self._active_callbacks
                self._active_callbacks = []
                for cb in cbs:
                    cb(ret)
        if self.axis.streamer is not None:
            self.axis.streamer.note_flush_time(flush_time)

    def is_active_axis(self, axis):
        a = axis.encode()
        return self._ffi_lib.itersolve_is_active_axis(
            self._stepper_kinematics, a
        )

    def turns_to_mm(self, turns):
        if self.axis.streamer is not None:
            return self.axis.streamer.turns_to_mm(turns)
        return turns * self.rotation_distance / self.direction

    def mm_to_turns(self, pos_mm):
        if self.axis.streamer is not None:
            return self.axis.streamer.mm_to_turns(pos_mm)
        return self.direction * pos_mm / self.rotation_distance


class ODriveRail:
    def __init__(
        self,
        config,
        axis,
        need_position_minmax=True,
        default_position_endstop=None,
    ):
        from . import endstop as endstop_mod
        from .streamer import SetpointStreamer

        self.axis = axis
        self.printer = config.get_printer()
        self.stepper = ODriveStepper(config, axis)
        self.steppers = [self.stepper]
        self.get_name = self.stepper.get_name
        self.get_commanded_position = self.stepper.get_commanded_position
        self.calc_position_from_coord = self.stepper.calc_position_from_coord
        self.endstops = []

        endstop_pin = config.get("endstop_pin", None)
        if endstop_pin is None:
            raise config.error(
                "ODrive-backed rail '%s' requires endstop_pin"
                % (config.get_name(),)
            )
        endstop_is_virtual = ":virtual_endstop" in endstop_pin
        ppins = self.printer.lookup_object("pins")
        mcu_endstop = ppins.setup_pin("endstop", endstop_pin)
        if endstop_is_virtual:
            real_endstop = mcu_endstop
        else:
            real_endstop = endstop_mod.PhysicalEndstopWrapper(mcu_endstop)
        if hasattr(real_endstop, "get_position_endstop"):
            self.position_endstop = real_endstop.get_position_endstop()
        elif default_position_endstop is None:
            self.position_endstop = config.getfloat("position_endstop")
        else:
            self.position_endstop = config.getfloat(
                "position_endstop", default_position_endstop
            )
        real_endstop.add_stepper(self.stepper)
        name = self.stepper.get_name(short=True)
        self.endstops.append((real_endstop, name))
        query_endstops = self.printer.load_object(config, "query_endstops")
        query_endstops.register_endstop(real_endstop, name)

        if need_position_minmax:
            self.position_min = config.getfloat("position_min", 0.0)
            self.position_max = config.getfloat(
                "position_max", above=self.position_min
            )
        else:
            self.position_min = 0.0
            self.position_max = self.position_endstop
        if (
            self.position_endstop < self.position_min
            or self.position_endstop > self.position_max
        ):
            raise config.error(
                "position_endstop in section '%s' must be between"
                " position_min and position_max" % (config.get_name(),)
            )

        self.use_sensorless_homing = config.getboolean(
            "use_sensorless_homing", endstop_is_virtual
        )
        self.homing_speed = config.getfloat("homing_speed", 5.0, above=0.0)
        default_second_homing_speed = self.homing_speed / 2.0
        if self.use_sensorless_homing:
            default_second_homing_speed = self.homing_speed
        self.second_homing_speed = config.getfloat(
            "second_homing_speed", default_second_homing_speed, above=0.0
        )
        self.homing_retract_speed = config.getfloat(
            "homing_retract_speed", self.homing_speed, above=0.0
        )
        self.homing_retract_dist = config.getfloat(
            "homing_retract_dist", 5.0, minval=0.0
        )
        self.homing_positive_dir = config.getboolean(
            "homing_positive_dir", None
        )
        self.min_home_dist = config.getfloat(
            "min_home_dist", self.homing_retract_dist, minval=0.0
        )
        self.homing_accel = config.getfloat("homing_accel", None, above=0.0)
        if self.homing_positive_dir is None:
            axis_len = self.position_max - self.position_min
            if self.position_endstop <= self.position_min + axis_len / 4.0:
                self.homing_positive_dir = False
            elif self.position_endstop >= self.position_max - axis_len / 4.0:
                self.homing_positive_dir = True
            else:
                raise config.error(
                    "Unable to infer homing_positive_dir in section '%s'"
                    % (config.get_name(),)
                )
            config.getboolean("homing_positive_dir", self.homing_positive_dir)
        elif (
            self.homing_positive_dir
            and self.position_endstop == self.position_min
        ) or (
            not self.homing_positive_dir
            and self.position_endstop == self.position_max
        ):
            raise config.error(
                "Invalid homing_positive_dir / position_endstop in '%s'"
                % (config.get_name(),)
            )

        self.homing_cfg = {
            "homing_current": config.getfloat(
                "homing_current", None, above=0.0
            ),
            "pos_error_threshold": config.getfloat(
                "homing_pos_error_threshold", 0.5, above=0.0
            ),
            "current_threshold": config.getfloat(
                "homing_current_threshold", 0.0, minval=0.0
            ),
            "trigger_count": config.getint("homing_trigger_count", 2, minval=1),
            "poll_period": config.getfloat(
                "homing_poll_period", 0.010, above=0.0
            ),
            "trigger_latency": config.getfloat(
                "homing_trigger_latency", 0.005, minval=0.0
            ),
        }

        self.streamer = SetpointStreamer(config, axis, self.stepper)
        axis.streamer = self.streamer

    def get_tmc_current_helpers(self):
        return [None]

    def get_range(self):
        return self.position_min, self.position_max

    def get_homing_info(self):
        homing_info = collections.namedtuple(
            "homing_info",
            [
                "speed",
                "position_endstop",
                "retract_speed",
                "retract_dist",
                "positive_dir",
                "second_homing_speed",
                "use_sensorless_homing",
                "min_home_dist",
                "accel",
            ],
        )(
            self.homing_speed,
            self.position_endstop,
            self.homing_retract_speed,
            self.homing_retract_dist,
            self.homing_positive_dir,
            self.second_homing_speed,
            self.use_sensorless_homing,
            self.min_home_dist,
            self.homing_accel,
        )
        return homing_info

    def get_steppers(self):
        return list(self.steppers)

    def get_endstops(self):
        return list(self.endstops)

    def add_extra_stepper(self, config):
        raise config.error(
            "Multi-stepper ODrive rails are not supported (section '%s')"
            % (config.get_name(),)
        )

    def setup_itersolve(self, alloc_func, *params):
        self.stepper.setup_itersolve(alloc_func, *params)

    def generate_steps(self, flush_time):
        self.stepper.generate_steps(flush_time)

    def set_trapq(self, trapq):
        self.stepper.set_trapq(trapq)

    def set_position(self, coord):
        self.stepper.set_position(coord)
