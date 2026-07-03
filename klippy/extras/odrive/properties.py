# ODrive v3.6 firmware property paths, enum values, and error decode
# tables for the 0.5.x firmware line (see docs/ODrive_Implementation_Spec.md)
#
# Copyright (C) 2026  Kalico contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# Axis states (AxisState)
AXIS_STATE_UNDEFINED = 0
AXIS_STATE_IDLE = 1
AXIS_STATE_STARTUP_SEQUENCE = 2
AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3
AXIS_STATE_MOTOR_CALIBRATION = 4
AXIS_STATE_ENCODER_INDEX_SEARCH = 6
AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7
AXIS_STATE_CLOSED_LOOP_CONTROL = 8
AXIS_STATE_LOCKIN_SPIN = 9
AXIS_STATE_ENCODER_DIR_FIND = 10
AXIS_STATE_HOMING = 11

AXIS_STATE_NAMES = {
    AXIS_STATE_UNDEFINED: "undefined",
    AXIS_STATE_IDLE: "idle",
    AXIS_STATE_STARTUP_SEQUENCE: "startup_sequence",
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE: "full_calibration_sequence",
    AXIS_STATE_MOTOR_CALIBRATION: "motor_calibration",
    AXIS_STATE_ENCODER_INDEX_SEARCH: "encoder_index_search",
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION: "encoder_offset_calibration",
    AXIS_STATE_CLOSED_LOOP_CONTROL: "closed_loop_control",
    AXIS_STATE_LOCKIN_SPIN: "lockin_spin",
    AXIS_STATE_ENCODER_DIR_FIND: "encoder_dir_find",
    AXIS_STATE_HOMING: "homing",
}

CALIBRATE_TYPE_STATES = {
    "full": AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
    "motor": AXIS_STATE_MOTOR_CALIBRATION,
    "encoder_offset": AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
    "index": AXIS_STATE_ENCODER_INDEX_SEARCH,
}

# Control modes (ControlMode)
CONTROL_MODE_VOLTAGE_CONTROL = 0
CONTROL_MODE_TORQUE_CONTROL = 1
CONTROL_MODE_VELOCITY_CONTROL = 2
CONTROL_MODE_POSITION_CONTROL = 3

# Input modes (InputMode)
INPUT_MODE_INACTIVE = 0
INPUT_MODE_PASSTHROUGH = 1
INPUT_MODE_VEL_RAMP = 2
INPUT_MODE_POS_FILTER = 3
INPUT_MODE_MIX_CHANNELS = 4
INPUT_MODE_TRAP_TRAJ = 5
INPUT_MODE_TORQUE_RAMP = 6
INPUT_MODE_MIRROR = 7

INPUT_MODE_CHOICES = {
    "passthrough": INPUT_MODE_PASSTHROUGH,
    "pos_filter": INPUT_MODE_POS_FILTER,
}

MOTOR_TYPE_CHOICES = {
    "high_current": 0,
    "gimbal": 2,
}

# Best-effort error bitfield decode tables for the 0.5.x firmware line.
# These are approximate and may not match every clone's exact firmware
# revision -- see the "Firmware-version tolerance layer" section of
# docs/ODrive_Implementation_Spec.md. Unknown bits still decode (as
# "bit_N") rather than being silently dropped.
AXIS_ERRORS = {
    0x01: "invalid_state",
    0x100: "watchdog_timer_expired",
    0x200: "min_endstop_pressed",
    0x400: "max_endstop_pressed",
    0x800: "estop_requested",
    0x1000: "homing_without_endstop",
    0x2000: "over_temp",
    0x4000: "unknown_position",
}

MOTOR_ERRORS = {
    0x1: "phase_resistance_out_of_range",
    0x2: "phase_inductance_out_of_range",
    0x8: "drv_fault",
    0x10: "control_deadline_missed",
    0x80: "modulation_magnitude",
    0x400: "current_sense_saturation",
    0x1000: "current_limit_violation",
    0x10000: "modulation_is_nan",
    0x20000: "motor_thermistor_over_temp",
    0x40000: "fet_thermistor_over_temp",
    0x80000: "timer_update_missed",
    0x100000: "current_measurement_unavailable",
    0x200000: "controller_failed",
    0x400000: "i_bus_out_of_range",
    0x800000: "brake_resistor_disarmed",
    0x1000000: "system_level",
    0x2000000: "bad_timing",
    0x4000000: "unknown_phase_estimate",
    0x8000000: "unknown_phase_vel",
    0x10000000: "unknown_torque",
    0x20000000: "unknown_current_command",
    0x40000000: "unknown_current_measurement",
    0x80000000: "unknown_vbus_voltage",
}

ENCODER_ERRORS = {
    0x1: "unstable_gain",
    0x2: "cpr_polepairs_mismatch",
    0x4: "no_response",
    0x8: "unsupported_encoder_mode",
    0x10: "illegal_hall_state",
    0x20: "index_not_found_yet",
    0x40: "abs_spi_timeout",
    0x80: "abs_spi_com_fail",
    0x100: "abs_spi_not_ready",
}

CONTROLLER_ERRORS = {
    0x1: "overspeed",
    0x2: "invalid_input_mode",
    0x4: "unstable_gain",
    0x8: "invalid_mirror_axis",
    0x10: "invalid_load_encoder",
    0x20: "invalid_estimate",
    0x40: "invalid_circular_range",
    0x80: "spinout_detected",
}

ERROR_TABLES = {
    "axis": AXIS_ERRORS,
    "motor": MOTOR_ERRORS,
    "encoder": ENCODER_ERRORS,
    "controller": CONTROLLER_ERRORS,
}


def decode_bitfield(value, table):
    if not value:
        return []
    names = []
    remaining = value
    for bit, name in sorted(table.items()):
        if value & bit:
            names.append(name)
            remaining &= ~bit
    bitpos = 0
    while remaining:
        if remaining & 1:
            names.append("bit_%d" % (bitpos,))
        remaining >>= 1
        bitpos += 1
    return names


def axis_state_name(value):
    try:
        value = int(value)
    except (TypeError, ValueError):
        return "unknown"
    return AXIS_STATE_NAMES.get(value, "unknown(%d)" % (value,))


# Known per-version property renames. Each entry maps a canonical
# internal key to the dotted property suffix (relative to "axisN.") used
# on that firmware line. Firmware >= 0.5.6 renamed enable_step_dir to
# step_dir_active; this table is intentionally small -- everything the
# module writes/reads for calibration, tuning, and streaming uses paths
# that are stable across the whole 0.5.x line.
VERSION_OVERRIDES = {
    (0, 5, 6): {
        "step_dir_active": "config.step_dir_active",
    },
}
DEFAULT_PATHS = {
    "step_dir_active": "config.enable_step_dir",
}


class PropertyMap:
    def __init__(self, fw_version=None):
        # fw_version is a (major, minor, revision) tuple, or None if unknown
        self.fw_version = fw_version
        overrides = {}
        if fw_version is not None:
            for min_version, paths in sorted(VERSION_OVERRIDES.items()):
                if fw_version >= min_version:
                    overrides.update(paths)
        self._paths = dict(DEFAULT_PATHS)
        self._paths.update(overrides)
        # Feature gates: default from version, confirmed/corrected later
        # by runtime probing (see ODriveBoard._probe_capabilities).
        self.capabilities = {
            "watchdog_feed_cmd": fw_version is None or fw_version >= (0, 5, 2),
            "device_homing": fw_version is None or fw_version >= (0, 5, 2),
            "endstop_gpio": fw_version is None or fw_version >= (0, 5, 2),
        }

    def path(self, canonical):
        return self._paths.get(canonical, canonical)
