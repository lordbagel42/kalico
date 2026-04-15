# Code for handling the kinematics of colinear tripteron robots
#
# Copyright (C) 2025
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math

from klippy import stepper


class ColinearTripteronKinematics:
    def __init__(self, toolhead, config):
        # Read geometry parameters
        arm_angle = config.getfloat("arm_angle", 30.0, above=0.0, below=90.0)
        alpha_rot = config.getfloat("alpha_tower_rotation", 0.0)
        beta_rot = config.getfloat("beta_tower_rotation", 120.0)
        gamma_rot = config.getfloat("gamma_tower_rotation", 240.0)
        # Precompute per-tower IK coefficients
        t = math.tan(math.radians(arm_angle))
        a_x = math.sin(math.radians(alpha_rot)) * t
        a_y = math.cos(math.radians(alpha_rot)) * t
        b_x = math.sin(math.radians(beta_rot)) * t
        b_y = math.cos(math.radians(beta_rot)) * t
        g_x = math.sin(math.radians(gamma_rot)) * t
        g_y = math.cos(math.radians(gamma_rot)) * t
        self.coeffs = (a_x, a_y, b_x, b_y, g_x, g_y)
        # Check that the tower geometry is non-degenerate
        det = (a_y * b_x - g_y * b_x - a_x * b_y
               - a_y * g_x + b_y * g_x + a_x * g_y)
        if abs(det) < 1e-10:
            raise config.error(
                "Colinear tripteron tower geometry is degenerate"
                " (determinant ~ 0)")
        self.det = det
        # Setup tower rails (stepper_a, stepper_b, stepper_c)
        stepper_configs = [config.getsection("stepper_" + a) for a in "abc"]
        rail_a = stepper.LookupMultiRail(stepper_configs[0])
        a_endstop = rail_a.get_homing_info().position_endstop
        rail_b = stepper.LookupMultiRail(
            stepper_configs[1], default_position_endstop=a_endstop)
        rail_c = stepper.LookupMultiRail(
            stepper_configs[2], default_position_endstop=a_endstop)
        self.rails = [rail_a, rail_b, rail_c]
        # Setup itersolve for each rail
        # IK: stepper = cx*x - cy*y + z  =>  C sees cx*x + cy*y + z
        # so pass (cx=a_x, cy=-a_y) etc.
        ik_params = [(a_x, -a_y), (b_x, -b_y), (g_x, -g_y)]
        for rail, (cx, cy) in zip(self.rails, ik_params):
            rail.setup_itersolve("colinear_tripteron_stepper_alloc", cx, cy)
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        # No endstop cross-registration needed. home_rails() starts all
        # endstops simultaneously, and the trdispatch mechanism ensures all
        # steppers stop when any endstop triggers.
        config.get_printer().register_event_handler(
            "stepper_enable:motor_off", self._motor_off)
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            "max_z_velocity", max_velocity, above=0.0, maxval=max_velocity)
        self.max_z_accel = config.getfloat(
            "max_z_accel", max_accel, above=0.0, maxval=max_accel)
        self.need_home = True
        ranges = [r.get_range() for r in self.rails]
        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.0)
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.0)
        self.supports_dual_carriage = False

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def calc_position(self, stepper_positions):
        a_x, a_y, b_x, b_y, g_x, g_y = self.coeffs
        spos = [stepper_positions[rail.get_name()] for rail in self.rails]
        a, b, c = spos
        det = self.det
        x = (a * (g_y - b_y) + b * (a_y - g_y) + c * (b_y - a_y)) / det
        y = (a * (g_x - b_x) + b * (a_x - g_x) + c * (b_x - a_x)) / det
        z = (a * (b_y * g_x - b_x * g_y)
             + b * (a_x * g_y - a_y * g_x)
             + c * (a_y * b_x - a_x * b_y)) / det
        return [x, y, z]

    def set_position(self, newpos, homing_axes):
        for rail in self.rails:
            rail.set_position(newpos)
        if tuple(homing_axes) == (0, 1, 2):
            self.need_home = False

    def clear_homing_state(self, axes):
        # Clearing any axis requires re-homing all (coupled kinematics)
        if 0 in axes or 1 in axes or 2 in axes:
            self.need_home = True

    def home(self, homing_state):
        # All axes are homed simultaneously (like delta)
        homing_state.set_axes([0, 1, 2])
        forcepos = [0.0, 0.0, None, None]
        # Use the first rail's homing info to set force position
        hi = self.rails[0].get_homing_info()
        position_min, position_max = self.rails[0].get_range()
        if hi.positive_dir:
            forcepos[2] = -1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[2] = 1.5 * (position_max - hi.position_endstop)
        homepos = [0.0, 0.0, hi.position_endstop, None]
        homing_state.home_rails(self.rails, forcepos, homepos)

    def _motor_off(self, print_time):
        self.need_home = True

    def check_move(self, move):
        if self.need_home:
            raise move.move_error("Must home axis first")
        end_pos = move.end_pos
        if (end_pos[0] < self.axes_min.x or end_pos[0] > self.axes_max.x
                or end_pos[1] < self.axes_min.y
                or end_pos[1] > self.axes_max.y
                or end_pos[2] < self.axes_min.z
                or end_pos[2] > self.axes_max.z):
            raise move.move_error()
        if move.axes_d[2]:
            z_ratio = move.move_d / abs(move.axes_d[2])
            move.limit_speed(
                self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)

    def get_status(self, eventtime):
        return {
            "homed_axes": "" if self.need_home else "xyz",
            "axis_minimum": self.axes_min,
            "axis_maximum": self.axes_max,
        }


def load_kinematics(toolhead, config):
    return ColinearTripteronKinematics(toolhead, config)
