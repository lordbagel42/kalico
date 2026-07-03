# ODrive v3.6 per-motor axis support -- thin top-level shim
#
# Kalico resolves a config section's module by the first word of its
# name (see Printer.load_object / PrinterModule.get_init_function in
# klippy/printer.py), so [odrive_axis <name>] needs a top-level module
# literally named "odrive_axis". The implementation itself lives in the
# odrive package alongside the board-level [odrive <name>] code (see
# klippy/extras/odrive/axis.py and docs/ODrive_Implementation_Spec.md).
#
# Copyright (C) 2026  Kalico contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from .odrive.axis import ODriveAxis


def load_config_prefix(config):
    return ODriveAxis(config)
