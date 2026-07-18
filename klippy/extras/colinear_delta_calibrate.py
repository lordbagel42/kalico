# Colinear delta calibration support (both mechanisms)
#
# A colinear delta is two linear-delta mechanisms sharing three physical
# rails: a "toolhead" delta (steppers a/b/c) and a "bed" delta (steppers
# d/e/f).  The stock DELTA_CALIBRATE tool can only calibrate the toolhead
# side, because a bed probe measures the nozzle *relative* to the plate and,
# at a single motion_split, the two mechanisms' contributions to that relative
# surface are not separable.
#
# This tool calibrates the bed side as well.  Two methods are provided:
#
#   * "bed" (default): treat the already-calibrated toolhead delta as a known
#     coordinate-measuring arm and solve the bed delta's three arm lengths and
#     three endstops from a single set of probe points.  Well conditioned; run
#     an ordinary DELTA_CALIBRATE first.
#
#   * "joint": solve BOTH deltas at once.  This is only observable when probe
#     samples span at least two motion_split values, because changing the
#     split exercises each mechanism at a different effector scale for the same
#     commanded point.  Collect samples at several splits with PROBE_ONLY=1
#     (changing motion_split in the config and restarting between passes - the
#     stored samples are split-independent step counts and survive the
#     restart), then run SOLVE=joint.
#
# Copyright (C) 2026  Kalico contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import math

from klippy import mathutil

from . import probe

SECTION = "colinear_delta_calibrate"


def load_config_stable(config, option):
    return config.getfloatlist(option, count=6)


class ColinearDeltaCalibrate:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler(
            "klippy:connect", self.handle_connect
        )
        # Default probing points (same scatter pattern as delta_calibrate)
        radius = config.getfloat("radius", above=0.0)
        points = [(0.0, 0.0)]
        scatter = [0.95, 0.90, 0.85, 0.70, 0.75, 0.80]
        for i in range(6):
            r = math.radians(90.0 + 60.0 * i)
            dist = radius * scatter[i]
            points.append((math.cos(r) * dist, math.sin(r) * dist))
        self.probe_helper = probe.ProbePointsHelper(
            config, self.probe_finalize, default_points=points
        )
        self.probe_helper.minimum_points(3)
        # Restore accumulated probe samples: (z_offset, motion_split, stable6)
        self.samples = []
        for i in range(9999):
            z_offset = config.getfloat("sample%d_z" % (i,), None)
            if z_offset is None:
                break
            split = config.getfloat("sample%d_split" % (i,))
            stable = list(load_config_stable(config, "sample%d_pos" % (i,)))
            self.samples.append((z_offset, split, stable))
        # Per-run options set from the gcode command before probing
        self._method = "bed"
        self._probe_only = False
        # Register gcode commands
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_command(
            "COLINEAR_DELTA_CALIBRATE",
            self.cmd_COLINEAR_DELTA_CALIBRATE,
            desc=self.cmd_COLINEAR_DELTA_CALIBRATE_help,
        )

    def handle_connect(self):
        kin = self.printer.lookup_object("toolhead").get_kinematics()
        if not hasattr(kin, "get_full_calibration"):
            raise self.printer.config_error(
                "colinear_delta_calibrate is only for colinear delta printers"
            )

    def _persist_samples(self, configfile):
        configfile.remove_section(SECTION)
        for i, (z_offset, split, stable) in enumerate(self.samples):
            configfile.set(SECTION, "sample%d_z" % (i,), "%.6f" % (z_offset,))
            configfile.set(SECTION, "sample%d_split" % (i,), "%.6f" % (split,))
            configfile.set(
                SECTION,
                "sample%d_pos" % (i,),
                ",".join("%.3f" % (v,) for v in stable),
            )

    def probe_finalize(self, offsets, positions):
        z_offset = offsets[2]
        kin = self.printer.lookup_object("toolhead").get_kinematics()
        cal = kin.get_full_calibration()
        for p in positions:
            stable = cal.calc_stable_position(p)
            self.samples.append((z_offset, cal.split, stable))
        self.gcode.respond_info(
            "Collected %d probe samples at motion_split %.3f"
            " (%d stored total)"
            % (len(positions), cal.split, len(self.samples))
        )
        if self._probe_only:
            configfile = self.printer.lookup_object("configfile")
            self._persist_samples(configfile)
            self.gcode.respond_info(
                "PROBE_ONLY: samples stored.  Run SAVE_CONFIG to keep them"
                " across a restart, then probe again at a different"
                " motion_split before SOLVE=joint."
            )
            return
        self.calculate_params(cal, self._method)

    def calculate_params(self, cal, method):
        if not self.samples:
            raise self.gcode.error("No probe samples collected")
        adj_params, params = cal.coordinate_descent_params(method)
        if method == "joint":
            splits = set("%.4f" % (s,) for (_, s, _) in self.samples)
            if len(splits) < 2:
                self.gcode.respond_info(
                    "WARNING: joint calibration is under-determined with"
                    " samples from a single motion_split (%s).  The toolhead"
                    " and bed geometry cannot be separated.  Collect samples"
                    " at >= 2 motion_split values (PROBE_ONLY=1) first, or use"
                    " the default SOLVE=bed." % (", ".join(sorted(splits)),)
                )
        logging.info(
            "Colinear delta calibrate (%s) with %d samples\nInitial: %s",
            method,
            len(self.samples),
            params,
        )

        def errorfunc(params):
            try:
                new_cal = cal.new_calibration(params)
                total_error = 0.0
                for z_offset, _split, stable in self.samples:
                    z = new_cal.get_position_from_stable(stable)[2]
                    total_error += (z - z_offset) ** 2
                return total_error
            except ValueError:
                return 9999999999999.9

        new_params = mathutil.background_coordinate_descent(
            self.printer, adj_params, params, errorfunc
        )
        logging.info("Calculated colinear delta parameters: %s", new_params)
        new_cal = cal.new_calibration(new_params)
        n = len(self.samples)
        for tag, c in (("orig", cal), ("new", new_cal)):
            sq = 0.0
            for z_offset, _split, stable in self.samples:
                sq += (c.get_position_from_stable(stable)[2] - z_offset) ** 2
            logging.info(
                "Colinear delta calibrate %s rms z error: %.6f mm",
                tag,
                math.sqrt(sq / n),
            )
        # Store results for SAVE_CONFIG
        configfile = self.printer.lookup_object("configfile")
        new_cal.save_state(configfile)
        self._persist_samples(configfile)
        self.gcode.respond_info(
            "The SAVE_CONFIG command will update the printer config file\n"
            "with these parameters and restart the printer."
        )

    cmd_COLINEAR_DELTA_CALIBRATE_help = (
        "Calibrate the bed mechanism of a colinear delta printer"
    )

    def cmd_COLINEAR_DELTA_CALIBRATE(self, gcmd):
        if gcmd.get_int("CLEAR", 0):
            self.samples = []
            configfile = self.printer.lookup_object("configfile")
            configfile.remove_section(SECTION)
            gcmd.respond_info(
                "Cleared stored colinear delta calibration samples."
                "  Run SAVE_CONFIG to persist."
            )
            return
        # NB: METHOD is consumed by the probe helper (manual/automatic); the
        # solver selection uses SOLVE to avoid colliding with it.
        method = gcmd.get("SOLVE", "bed").lower()
        if method not in ("bed", "joint"):
            raise gcmd.error("SOLVE must be 'bed' or 'joint'")
        self._method = method
        self._probe_only = bool(gcmd.get_int("PROBE_ONLY", 0))
        self.probe_helper.start_probe(gcmd)


def load_config(config):
    return ColinearDeltaCalibrate(config)
