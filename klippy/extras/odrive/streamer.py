# ODrive v3.6 trapq-to-setpoint streaming
#
# See docs/ODrive_Implementation_Spec.md, "Motion streaming design".
# Samples the toolhead's live (not-yet-finalized) trapq -- via the
# trapq_extract_pending chelper addition -- to get an exact analytic
# commanded position/velocity for this axis at a time slightly ahead of
# "now" (to compensate for USB + ODrive filter latency), converts it to
# turns, and streams it as an ASCII "p" setpoint line.
#
# Optional input shaping (shaper_type/shaper_freq/damping_ratio): Kalico's
# [input_shaper] mechanism (klippy/chelper/kin_shaper.c) shapes motion by
# swapping a stepper's stepper_kinematics for a wrapper whose
# calc_position_cb convolves the *original* kinematics' position samples
# at several time offsets -- but that convolution only ever runs from
# itersolve_gen_steps_range (klippy/chelper/itersolve.c), the MCU step-
# generation loop. This module never calls into itersolve at all (see
# odrv_stepper.py's module docstring) -- it samples the trapq directly --
# so a normal [input_shaper] section has zero effect on an ODrive-driven
# rail. This reimplements the same convolution (mirroring kin_shaper.c's
# init_shaper/shift_pulses/calc_position, and reusing shaper_defs.py's
# per-shaper-type impulse coefficients) as a host-side pre-filter over the
# setpoint stream instead.
#
# Copyright (C) 2026  Kalico contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections

from ... import chelper
from .. import shaper_defs

RING_BUFFER_SECONDS = 4.0
VEL_EPSILON = 0.0005

SHAPER_INIT_FUNCS = {
    cfg.name: cfg.init_func for cfg in shaper_defs.INPUT_SHAPERS
}


class SetpointStreamer:
    def __init__(self, config, axis, stepper):
        self.printer = config.get_printer()
        self.axis = axis
        self.board = axis.board
        self.stepper = stepper
        self.direction = stepper.direction
        self.rotation_distance = stepper.rotation_distance
        self.offset_turns = 0.0
        self._trapq = None
        self._sample_horizon = 0.0
        self._active = False
        self._last_sent_turns = 0.0
        self._ring = collections.deque()
        ffi_main, ffi_lib = chelper.get_ffi()
        self._ffi_main = ffi_main
        self._ffi_lib = ffi_lib
        self._shaper_pulses = self._build_shaper(config)

    def _build_shaper(self, config):
        shaper_type = config.get("shaper_type", "none").lower()
        if shaper_type == "none":
            return None
        if shaper_type not in SHAPER_INIT_FUNCS:
            raise config.error(
                "Unknown shaper_type '%s' in section '%s' (expected one of"
                " none, %s)"
                % (
                    shaper_type,
                    config.get_name(),
                    ", ".join(sorted(SHAPER_INIT_FUNCS)),
                )
            )
        shaper_freq = config.getfloat("shaper_freq", above=0.0)
        damping_ratio = config.getfloat(
            "damping_ratio",
            shaper_defs.DEFAULT_DAMPING_RATIO,
            above=0.0,
            below=1.0,
        )
        amplitudes, times = SHAPER_INIT_FUNCS[shaper_type](
            shaper_freq, damping_ratio
        )
        total_a = sum(amplitudes)
        amplitudes = [a / total_a for a in amplitudes]
        # Mirrors kin_shaper.c's shift_pulses: centering the convolution on
        # its amplitude-weighted mean time makes it an identity transform
        # for constant-velocity motion (shaped(v*t) == v*t), not just a
        # pure delay -- see calc_position/shift_pulses there for the
        # original derivation. `offset` here is exactly kin_shaper.c's
        # per-pulse final pulse time (mean_t - t_i), the amount to add to
        # the target sample time for that pulse.
        mean_t = sum(a * t for a, t in zip(amplitudes, times))
        offsets = [mean_t - t for t in times]
        return list(zip(amplitudes, offsets))

    def set_trapq(self, tq):
        self._trapq = tq

    def note_flush_time(self, flush_time):
        if flush_time > self._sample_horizon:
            self._sample_horizon = flush_time

    def note_position_set(self, coord):
        pos_mm = self.stepper.get_commanded_position()
        target_turns = self.direction * pos_mm / self.rotation_distance
        self.offset_turns = self._last_sent_turns - target_turns
        self._ring.clear()

    def turns_to_mm(self, turns):
        return (
            (turns - self.offset_turns)
            * self.rotation_distance
            / self.direction
        )

    def mm_to_turns(self, pos_mm):
        return (
            self.direction * pos_mm / self.rotation_distance + self.offset_turns
        )

    def get_ring_buffer_position_mm(self, print_time):
        if not self._ring:
            return None
        prev = None
        for t, pos in self._ring:
            if t >= print_time:
                if prev is None:
                    return pos
                pt, pp = prev
                if t == pt:
                    return pos
                frac = (print_time - pt) / (t - pt)
                return pp + frac * (pos - pp)
            prev = (t, pos)
        return self._ring[-1][1]

    def start(self):
        if self._active:
            return
        self._active = True
        self._last_sent_turns = self.mm_to_turns(
            self.stepper.get_commanded_position()
        )
        self.board.transport.set_tx_period(self.board.sample_period)
        self.board.transport.set_stream_callback(self._tx_tick)

    def stop(self):
        if not self._active:
            return
        self._active = False
        if self.board.transport.stream_cb is self._tx_tick:
            self.board.transport.set_stream_callback(None)

    def _sample_trapq(self, print_time):
        if self._trapq is None:
            return None, 0.0
        data = self._ffi_main.new("struct pull_move[1]")
        count = self._ffi_lib.trapq_extract_pending(
            self._trapq, data, 1, print_time, print_time + 1e-6
        )
        if not count:
            count = self._ffi_lib.trapq_extract_old(
                self._trapq, data, 1, 0.0, print_time
            )
        if not count:
            return None, 0.0
        move = data[0]
        move_time = max(0.0, min(move.move_t, print_time - move.print_time))
        dist = (move.start_v + 0.5 * move.accel * move_time) * move_time
        coord = (
            move.start_x + move.x_r * dist,
            move.start_y + move.y_r * dist,
            move.start_z + move.z_r * dist,
        )
        pos_mm = self.stepper.calc_position_from_coord(coord)
        move_time2 = min(move.move_t, move_time + VEL_EPSILON)
        dist2 = (move.start_v + 0.5 * move.accel * move_time2) * move_time2
        coord2 = (
            move.start_x + move.x_r * dist2,
            move.start_y + move.y_r * dist2,
            move.start_z + move.z_r * dist2,
        )
        pos_mm2 = self.stepper.calc_position_from_coord(coord2)
        dt = move_time2 - move_time
        vel_mm = (pos_mm2 - pos_mm) / dt if dt > 0.0 else 0.0
        return pos_mm, vel_mm

    def _sample_trapq_shaped(self, target_pt):
        pos_sum = 0.0
        vel_sum = 0.0
        for amplitude, offset in self._shaper_pulses:
            sample_pt = min(target_pt + offset, self._sample_horizon)
            pos_mm, vel_mm = self._sample_trapq(sample_pt)
            if pos_mm is None:
                # Not enough trapq history/lookahead for this pulse yet
                # (e.g. right at the start of a move sequence) -- fall
                # back to the same "hold last position" behavior as the
                # unshaped path rather than a partial, meaningless sum.
                return None, 0.0
            pos_sum += amplitude * pos_mm
            vel_sum += amplitude * vel_mm
        return pos_sum, vel_sum

    def _tx_tick(self, eventtime):
        board = self.board
        target_pt = (
            board.estimated_print_time(eventtime) + board.latency_compensation
        )
        if target_pt > self._sample_horizon:
            target_pt = self._sample_horizon
        if self._shaper_pulses is None:
            pos_mm, vel_mm = self._sample_trapq(target_pt)
        else:
            pos_mm, vel_mm = self._sample_trapq_shaped(target_pt)
        if pos_mm is None:
            pos_turns = self._last_sent_turns
            vel_turns = 0.0
            pos_mm = self.turns_to_mm(pos_turns)
        else:
            pos_turns = self.mm_to_turns(pos_mm)
            vel_turns = self.direction * vel_mm / self.rotation_distance
        self._last_sent_turns = pos_turns
        self._ring.append((target_pt, pos_mm))
        while (
            len(self._ring) > 2
            and (target_pt - self._ring[0][0]) > RING_BUFFER_SECONDS
        ):
            self._ring.popleft()
        self.axis.note_setpoint_sent(pos_turns)
        return "p %d %.6f %.6f 0.0" % (
            self.axis.axis_index,
            pos_turns,
            vel_turns,
        )
