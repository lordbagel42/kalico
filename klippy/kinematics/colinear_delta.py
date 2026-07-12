# Code for handling the kinematics of colinear delta robots
#
# A colinear delta is two independent linear-delta mechanisms that share the
# same three physical (colinear) rails.  A "toolhead" delta (steppers a/b/c)
# positions the nozzle and a "bed" delta (steppers d/e/f) positions the build
# plate.  Each of the three vertical rails carries two carriages (one from each
# mechanism), which is why the rails are "colinear".
#
# The commanded coordinate P = (x, y, z) is the position of the nozzle relative
# to the part.  The two mechanisms share that motion (a "motion_split" fraction
# to the toolhead, the remainder to the bed) and the bed mechanism operates in
# a coordinate frame rotated about Z relative to the toolhead frame
# (secondary_rotation, 180 degrees for a typical colinear delta).  The relative
# geometry obeys
#
#     T_world - B_world = Rz(theta) * P
#
# where T_world / B_world are the toolhead / bed effector world positions.  The
# split assigns T_world = split*Rz(theta)*P and B_world = -(1-split)*Rz(theta)*P
# (up to constants fixed at homing), so each carriage runs a standard delta
# sphere solve on a fixed affine map of P.  See docs/Colinear_Delta.md and
# docs/developers/Colinear_Delta_Agent_Guide.md for the full derivation.
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2026  Kalico contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import math

from klippy import mathutil, stepper
from klippy.kinematics.delta import DeltaCalibration

# Slow moves once the ratio of tower to XY movement exceeds SLOW_RATIO
SLOW_RATIO = 3.0


class ColinearDeltaKinematics:
    def __init__(self, toolhead, config):
        # Motion split factor and secondary (bed) frame rotation
        self.split = split = config.getfloat(
            "motion_split", 0.5, above=0.0, below=1.0
        )
        self.secondary_rotation = config.getfloat("secondary_rotation", 180.0)
        theta = math.radians(self.secondary_rotation)
        self.cos_theta = ct = math.cos(theta)
        self.sin_theta = st = math.sin(theta)
        # Setup the six rails: toolhead a/b/c and bed d/e/f
        stepper_configs = [config.getsection("stepper_" + a) for a in "abcdef"]
        rail_a = stepper.LookupMultiRail(
            stepper_configs[0], need_position_minmax=False
        )
        a_endstop = rail_a.get_homing_info().position_endstop
        rail_b = stepper.LookupMultiRail(
            stepper_configs[1],
            need_position_minmax=False,
            default_position_endstop=a_endstop,
        )
        rail_c = stepper.LookupMultiRail(
            stepper_configs[2],
            need_position_minmax=False,
            default_position_endstop=a_endstop,
        )
        rail_d = stepper.LookupMultiRail(
            stepper_configs[3], need_position_minmax=False
        )
        d_endstop = rail_d.get_homing_info().position_endstop
        rail_e = stepper.LookupMultiRail(
            stepper_configs[4],
            need_position_minmax=False,
            default_position_endstop=d_endstop,
        )
        rail_f = stepper.LookupMultiRail(
            stepper_configs[5],
            need_position_minmax=False,
            default_position_endstop=d_endstop,
        )
        self.rails = [rail_a, rail_b, rail_c, rail_d, rail_e, rail_f]
        self.toolhead_rails = self.rails[:3]
        self.bed_rails = self.rails[3:]
        config.get_printer().register_event_handler(
            "stepper_enable:motor_off", self._motor_off
        )
        # Setup max velocity
        self.max_velocity, self.max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            "max_z_velocity",
            self.max_velocity,
            above=0.0,
            maxval=self.max_velocity,
        )
        self.max_z_accel = config.getfloat(
            "max_z_accel", self.max_accel, above=0.0, maxval=self.max_accel
        )
        # Read radius and per-tower arm lengths (shared rails => shared radius)
        self.radius = radius = config.getfloat("delta_radius", above=0.0)
        print_radius = config.getfloat("print_radius", radius, above=0.0)
        arm_length_a = stepper_configs[0].getfloat("arm_length", above=radius)
        self.tool_arms = tool_arms = [
            sconfig.getfloat("arm_length", arm_length_a, above=radius)
            for sconfig in stepper_configs[:3]
        ]
        # Bed arms default to the matching toolhead arm on the same rail
        bed_arms = [
            stepper_configs[3 + i].getfloat(
                "arm_length", tool_arms[i], above=radius
            )
            for i in range(3)
        ]
        arm_lengths = tool_arms + bed_arms
        self.arm_lengths = arm_lengths
        self.arm2 = [arm**2 for arm in arm_lengths]
        # Tower cartesian positions.  Bed carriages ride the same rails, so bed
        # tower angles default to the matching toolhead tower angle.
        self.tool_angles = tool_angles = [
            sconfig.getfloat("angle", angle)
            for sconfig, angle in zip(stepper_configs[:3], [210.0, 330.0, 90.0])
        ]
        bed_angles = [
            stepper_configs[3 + i].getfloat("angle", tool_angles[i])
            for i in range(3)
        ]
        self.angles = angles = tool_angles + bed_angles
        self.towers = [
            (
                math.cos(math.radians(angle)) * radius,
                math.sin(math.radians(angle)) * radius,
            )
            for angle in angles
        ]
        # Per-stepper affine coefficients (see module docstring):
        #   toolhead effector = split      * Rz(theta) * P
        #   bed effector      = -(1-split) * Rz(theta) * P
        tool_coef = (split * ct, split * st, split)
        bed_coef = (
            -(1.0 - split) * ct,
            -(1.0 - split) * st,
            -(1.0 - split),
        )
        self.coefs = [tool_coef] * 3 + [bed_coef] * 3
        for rail, arm2, tower, coef in zip(
            self.rails, self.arm2, self.towers, self.coefs
        ):
            rail.setup_itersolve(
                "colinear_delta_stepper_alloc",
                arm2,
                tower[0],
                tower[1],
                coef[0],
                coef[1],
                coef[2],
            )
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        # Absolute carriage heights at each endstop (delta convention)
        self.abs_endstops = [
            rail.get_homing_info().position_endstop
            + math.sqrt(arm2 - radius**2)
            for rail, arm2 in zip(self.rails, self.arm2)
        ]
        # Per-mechanism home positions (relative coordinate P at which each
        # mechanism sits on its endstops).  T_eff = split*Rz(theta)*home_tool
        # and B_eff = -(1-split)*Rz(theta)*home_bed, so invert accordingly.
        t_eff = mathutil.trilateration(
            [
                (self.towers[i][0], self.towers[i][1], self.abs_endstops[i])
                for i in range(3)
            ],
            self.arm2[:3],
        )
        b_eff = mathutil.trilateration(
            [
                (self.towers[i][0], self.towers[i][1], self.abs_endstops[i])
                for i in range(3, 6)
            ],
            self.arm2[3:],
        )
        self.home_tool = self._scale_unrotate(t_eff, 1.0 / split)
        self.home_bed = self._scale_unrotate(b_eff, -1.0 / (1.0 - split))
        # After homing (bed first, toolhead last) the machine rests at the
        # toolhead home (nozzle furthest from the part).
        self.home_position = tuple(self.home_tool)
        self.max_z = self.home_tool[2]
        self.min_z = config.getfloat(
            "minimum_z_position", 0.0, maxval=self.max_z
        )
        # Reachable envelope.  The sphere argument depends only on effector XY,
        # so validity is a pure XY-reach constraint per mechanism.  A move
        # coordinate P at XY radius rho places the toolhead effector at
        # split*rho and the bed effector at (1-split)*rho, so the reachable P
        # radius is bounded by the tighter of the two scaled reaches.
        tool_scale = split
        bed_scale = 1.0 - split
        min_arm_tool = min(tool_arms)
        min_arm_bed = min(bed_arms)
        tool_reach = (min_arm_tool - radius) / tool_scale
        bed_reach = (min_arm_bed - radius) / bed_scale
        max_reach = min(tool_reach, bed_reach)
        if print_radius >= max_reach:
            raise config.error(
                "print_radius (%.3f) exceeds the reachable radius (%.3f) for"
                " this colinear delta.  Increase arm_length, reduce"
                " delta_radius, or lower print_radius."
                % (print_radius, max_reach)
            )
        self.max_xy2 = print_radius**2
        half_min_step_dist = (
            min(r.get_steppers()[0].get_step_dist() for r in self.rails) * 0.5
        )

        def ratio_to_xy(ratio, min_arm, scale):
            return (
                ratio
                * math.sqrt(
                    min_arm**2 / (ratio**2 + 1.0) - half_min_step_dist**2
                )
                + half_min_step_dist
                - radius
            ) / scale

        self.slow_xy2 = (
            min(
                ratio_to_xy(SLOW_RATIO, min_arm_tool, tool_scale),
                ratio_to_xy(SLOW_RATIO, min_arm_bed, bed_scale),
            )
            ** 2
        )
        self.very_slow_xy2 = (
            min(
                ratio_to_xy(2.0 * SLOW_RATIO, min_arm_tool, tool_scale),
                ratio_to_xy(2.0 * SLOW_RATIO, min_arm_bed, bed_scale),
            )
            ** 2
        )
        max_xy = math.sqrt(self.max_xy2)
        logging.info(
            "Colinear delta max build radius %.2fmm (moves slowed past %.2fmm"
            " and %.2fmm), build Z %.2f..%.2fmm, split %.3f, rotation %.1fdeg"
            % (
                max_xy,
                math.sqrt(self.slow_xy2),
                math.sqrt(self.very_slow_xy2),
                self.min_z,
                self.max_z,
                split,
                self.secondary_rotation,
            )
        )
        self.axes_min = toolhead.Coord(-max_xy, -max_xy, self.min_z, 0.0)
        self.axes_max = toolhead.Coord(max_xy, max_xy, self.max_z, 0.0)
        self.need_home = True
        self.limit_xy2 = -1.0
        self.set_position([0.0, 0.0, 0.0], ())
        self.supports_dual_carriage = False

    def _scale_unrotate(self, eff, scale):
        # Recover a relative coordinate P from an effector position:
        #   eff = (1/scale) * Rz(theta) * P  =>  P = scale * Rz(-theta) * eff
        ct, st = self.cos_theta, self.sin_theta
        return [
            scale * (ct * eff[0] + st * eff[1]),
            scale * (-st * eff[0] + ct * eff[1]),
            scale * eff[2],
        ]

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def _actuator_to_cartesian(self, spos):
        # Forward kinematics: P = Rz(-theta) * (T_world - B_world)
        t_eff = mathutil.trilateration(
            [(self.towers[i][0], self.towers[i][1], spos[i]) for i in range(3)],
            self.arm2[:3],
        )
        b_eff = mathutil.trilateration(
            [
                (self.towers[i][0], self.towers[i][1], spos[i])
                for i in range(3, 6)
            ],
            self.arm2[3:],
        )
        ct, st = self.cos_theta, self.sin_theta
        dx = t_eff[0] - b_eff[0]
        dy = t_eff[1] - b_eff[1]
        dz = t_eff[2] - b_eff[2]
        return [ct * dx + st * dy, -st * dx + ct * dy, dz]

    def calc_position(self, stepper_positions):
        spos = [stepper_positions[rail.get_name()] for rail in self.rails]
        return self._actuator_to_cartesian(spos)

    def set_position(self, newpos, homing_axes):
        for rail in self.rails:
            rail.set_position(newpos)
        self.limit_xy2 = -1.0
        if tuple(homing_axes) == (0, 1, 2):
            self.need_home = False

    def clear_homing_state(self, axes):
        # Clearing homing state for each axis individually is not implemented
        if 0 in axes or 1 in axes or 2 in axes:
            self.limit_xy2 = -1
            self.need_home = True

    def home(self, homing_state):
        # All axes are homed simultaneously.  The two mechanisms move in
        # opposite Z directions for a relative-Z move, so they are homed
        # sequentially: the bed mechanism first, then the toolhead, which
        # leaves the nozzle parked furthest from the part.
        homing_state.set_axes([0, 1, 2])
        span = 1.5 * math.sqrt(max(self.arm2))
        # Home the bed mechanism (its carriages rise as the relative Z drops)
        forcepos_bed = list(self.home_bed)
        forcepos_bed[2] = self.home_tool[2] + span
        homing_state.home_rails(
            self.bed_rails, forcepos_bed, list(self.home_bed)
        )
        # Home the toolhead mechanism from the current (bed home) position
        homing_state.home_rails(
            self.toolhead_rails, list(self.home_bed), list(self.home_tool)
        )

    def _motor_off(self, print_time):
        self.clear_homing_state((0, 1, 2))

    def check_move(self, move):
        end_pos = move.end_pos
        end_xy2 = end_pos[0] ** 2 + end_pos[1] ** 2
        if end_xy2 <= self.limit_xy2 and not move.axes_d[2]:
            # Normal XY move
            return
        if self.need_home:
            raise move.move_error("Must home first")
        end_z = end_pos[2]
        limit_xy2 = self.max_xy2
        if end_xy2 > limit_xy2 or end_z > self.max_z or end_z < self.min_z:
            # Move out of range - verify not a homing move.  Homing drives the
            # two mechanisms to opposite Z extremes, so a centered move
            # anywhere between the bed and toolhead home heights is permitted.
            if (
                end_pos[:2] != self.home_position[:2]
                or end_z < self.home_bed[2]
                or end_z > self.home_tool[2]
            ):
                raise move.move_error()
            limit_xy2 = -1.0
        if move.axes_d[2]:
            z_ratio = move.move_d / abs(move.axes_d[2])
            move.limit_speed(
                self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio
            )
            limit_xy2 = -1.0
        # Limit the speed/accel of moves at the extreme edge of the envelope
        extreme_xy2 = max(
            end_xy2, move.start_pos[0] ** 2 + move.start_pos[1] ** 2
        )
        if extreme_xy2 > self.slow_xy2:
            r = 0.5
            if extreme_xy2 > self.very_slow_xy2:
                r = 0.25
            move.limit_speed(self.max_velocity * r, self.max_accel * r)
            limit_xy2 = -1.0
        self.limit_xy2 = min(limit_xy2, self.slow_xy2)

    def get_status(self, eventtime):
        return {
            "homed_axes": "" if self.need_home else "xyz",
            "axis_minimum": self.axes_min,
            "axis_maximum": self.axes_max,
            "cone_start_z": self.max_z,
        }

    def get_calibration(self):
        # DELTA_CALIBRATE operates on the toolhead mechanism; the bed mechanism
        # is assumed to be symmetric (see docs/Colinear_Delta.md).
        endstops = [
            rail.get_homing_info().position_endstop
            for rail in self.toolhead_rails
        ]
        stepdists = [
            rail.get_steppers()[0].get_step_dist()
            for rail in self.toolhead_rails
        ]
        return DeltaCalibration(
            self.radius, self.tool_angles, self.tool_arms, endstops, stepdists
        )


def load_kinematics(toolhead, config):
    return ColinearDeltaKinematics(toolhead, config)
