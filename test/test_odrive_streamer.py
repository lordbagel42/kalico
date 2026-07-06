"""Tests for klippy/extras/odrive/streamer.py's input-shaping support.

Uses the *real* chelper trapq + a real cartesian itersolve stepper
kinematics (mirroring the minimal standalone setup in
klippy/extras/manual_stepper.py) rather than faking trapq sampling --
the whole point of this module is exact analytic sampling of real
trapq/itersolve math, so a fake trapq would test nothing useful. No
toolhead/printer object graph is needed for either, matching this
package's existing "no full Klippy printer object graph" test
philosophy (see test_odrive_repl.py, test_odrive_axis.py).
"""

from __future__ import annotations

import pytest

from klippy import chelper
from klippy.extras import shaper_defs
from klippy.extras.odrive import streamer as streamer_mod


class FakeConfig:
    def __init__(self, options=None):
        self.options = options or {}

    def get_printer(self):
        return None

    def get_name(self):
        return "test_stepper"

    def get(self, name, default=None):
        return self.options.get(name, default)

    def getfloat(
        self,
        name,
        default=None,
        above=None,
        below=None,
        minval=None,
        maxval=None,
    ):
        val = self.options.get(name, default)
        if val is None:
            raise self.error("%s must be specified" % (name,))
        val = float(val)
        if above is not None and not (val > above):
            raise self.error("%s must be above %s" % (name, above))
        if below is not None and not (val < below):
            raise self.error("%s must be below %s" % (name, below))
        return val

    def error(self, msg):
        return Exception(msg)


class RealLinearStepper:
    """A real chelper trapq + cartesian itersolve kinematics, standalone."""

    def __init__(self, rotation_distance=40.0, direction=1):
        self.rotation_distance = rotation_distance
        self.direction = direction
        ffi_main, ffi_lib = chelper.get_ffi()
        self._ffi_main = ffi_main
        self._ffi_lib = ffi_lib
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self._sk = ffi_main.gc(
            ffi_lib.cartesian_stepper_alloc(b"x"), ffi_lib.free
        )
        ffi_lib.itersolve_set_trapq(self._sk, self.trapq)

    def append_move(
        self,
        print_time,
        accel_t,
        cruise_t,
        decel_t,
        start_pos,
        start_v,
        cruise_v,
        accel,
    ):
        self._ffi_lib.trapq_append(
            self.trapq,
            print_time,
            accel_t,
            cruise_t,
            decel_t,
            start_pos,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            start_v,
            cruise_v,
            accel,
        )

    def calc_position_from_coord(self, coord):
        return self._ffi_lib.itersolve_calc_position_from_coord(
            self._sk, coord[0], coord[1], coord[2]
        )

    def get_commanded_position(self):
        return self._ffi_lib.itersolve_get_commanded_pos(self._sk)


class FakeBoard:
    latency_compensation = 0.0

    def estimated_print_time(self, eventtime):
        return eventtime


class FakeAxis:
    axis_index = 0

    def __init__(self, board):
        self.board = board
        self.setpoints = []

    def note_setpoint_sent(self, turns):
        self.setpoints.append(turns)


def _make_streamer(shaper_options=None):
    config = FakeConfig(shaper_options)
    board = FakeBoard()
    axis = FakeAxis(board)
    stepper = RealLinearStepper()
    s = streamer_mod.SetpointStreamer(config, axis, stepper)
    s.set_trapq(stepper.trapq)
    s._sample_horizon = 1e9
    return s, stepper, axis


def test_no_shaper_configured_by_default():
    s, _, _ = _make_streamer()
    assert s._shaper_pulses is None


def test_unknown_shaper_type_errors():
    with pytest.raises(Exception, match="Unknown shaper_type"):
        _make_streamer({"shaper_type": "bogus", "shaper_freq": 30.0})


def test_shaper_type_none_is_explicitly_allowed():
    s, _, _ = _make_streamer({"shaper_type": "none"})
    assert s._shaper_pulses is None


def test_shaper_pulses_amplitudes_normalized_and_offsets_match_formula():
    s, _, _ = _make_streamer(
        {"shaper_type": "zv", "shaper_freq": 30.0, "damping_ratio": 0.1}
    )
    amplitudes, times = shaper_defs.get_zv_shaper(30.0, 0.1)
    total_a = sum(amplitudes)
    expected_a = [a / total_a for a in amplitudes]
    mean_t = sum(a * t for a, t in zip(expected_a, times))
    expected_offsets = [mean_t - t for t in times]

    assert s._shaper_pulses is not None
    got_a = [a for a, _ in s._shaper_pulses]
    got_offsets = [off for _, off in s._shaper_pulses]
    assert got_a == pytest.approx(expected_a)
    assert got_offsets == pytest.approx(expected_offsets)
    assert sum(got_a) == pytest.approx(1.0)


@pytest.mark.parametrize(
    "shaper_type", ["zv", "mzv", "zvd", "ei", "2hump_ei", "3hump_ei"]
)
def test_shaped_sampling_is_identity_for_constant_velocity(shaper_type):
    # The whole point of centering the convolution on its amplitude-
    # weighted mean time (shift_pulses in kin_shaper.c) is that shaping a
    # pure constant-velocity trajectory reproduces it exactly -- this is
    # the strongest single correctness check for the offset formula.
    s, stepper, _ = _make_streamer(
        {"shaper_type": shaper_type, "shaper_freq": 30.0, "damping_ratio": 0.1}
    )
    stepper.append_move(0.0, 0.1, 2.0, 0.1, 0.0, 0.0, 10.0, 100.0)

    target_pt = 1.0
    unshaped_pos, unshaped_vel = s._sample_trapq(target_pt)
    shaped_pos, shaped_vel = s._sample_trapq_shaped(target_pt)

    assert shaped_pos == pytest.approx(unshaped_pos, abs=1e-6)
    assert shaped_vel == pytest.approx(unshaped_vel, abs=1e-6)
    assert shaped_vel == pytest.approx(10.0, abs=1e-3)


def test_shaped_sampling_differs_near_a_sharp_acceleration_change():
    # Near a sharp transition (end of an accel phase into cruise), the
    # shaper should actually pull in samples from both sides and produce
    # something different from the naive single-sample read -- proving
    # the shaped path is really being exercised, not silently falling
    # through to unshaped.
    s, stepper, _ = _make_streamer(
        {"shaper_type": "mzv", "shaper_freq": 20.0, "damping_ratio": 0.1}
    )
    # Sharp, short accel phase (0.02s) up to a fast cruise, right at the
    # sample point, so the shaper's lookback/lookahead window straddles
    # the transition.
    stepper.append_move(0.0, 0.02, 2.0, 0.02, 0.0, 0.0, 50.0, 2500.0)

    target_pt = 0.02
    unshaped_pos, _ = s._sample_trapq(target_pt)
    shaped_pos, _ = s._sample_trapq_shaped(target_pt)

    assert shaped_pos != pytest.approx(unshaped_pos, abs=1e-9)


def test_tx_tick_uses_shaped_sampling_when_configured():
    s, stepper, axis = _make_streamer(
        {"shaper_type": "zv", "shaper_freq": 30.0, "damping_ratio": 0.1}
    )
    stepper.append_move(0.0, 0.1, 2.0, 0.1, 0.0, 0.0, 10.0, 100.0)

    line = s._tx_tick(1.0)

    assert line.startswith("p 0 ")
    assert axis.setpoints, "note_setpoint_sent should have been called"
    # Constant velocity -- shaped result should match the direct formula
    # (turns = mm / rotation_distance, direction=1).
    assert axis.setpoints[-1] == pytest.approx(9.5 / 40.0, abs=1e-4)
