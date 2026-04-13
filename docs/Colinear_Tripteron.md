# Colinear Tripteron

This document describes the configuration and calibration of colinear
tripteron style printers in Kalico.

## Overview

A colinear tripteron is a 3-DOF purely translational parallel
manipulator. Three linear actuators (typically lead screws or belt
drives) slide along the Z axis, each connected to a shared
end-effector platform via a rigid arm held at a fixed angle from
vertical. The three legs are rotated around the Z axis — in a standard
symmetric arrangement at 0°, 120°, and 240°.

Key advantages of colinear tripteron kinematics:

- **Fully linear kinematics**: The forward and inverse kinematic
  equations are simple linear functions — no square roots or
  trigonometry at runtime. This makes motion planning extremely fast
  to compute.
- **High stiffness**: As a parallel manipulator, the structure is
  inherently rigid compared to serial (cantilever) designs.
- **All-translational motion**: The end-effector has three degrees of
  pure translational freedom (X, Y, Z) with no rotational coupling,
  so the nozzle remains perpendicular to the bed at all positions.
- **Identical actuators**: All three actuators are the same type
  (vertical linear), simplifying construction and sourcing.

## Hardware Requirements

A colinear tripteron printer requires:

- **Three vertical linear rails or lead screws**: These are the
  actuators for the three towers (A, B, C). Each tower has a carriage
  that moves up and down along the Z axis.
- **Three rigid arms**: Each arm connects one tower's carriage to the
  end-effector platform. The arms are held at a fixed angle from
  vertical (the `arm_angle` parameter). All arms must be the same
  length.
- **End-effector platform**: A common platform (with hotend mount)
  where all three arms converge. The joints at the platform allow the
  arms to pivot as the carriages move.
- **Three endstops**: One per tower, typically placed at the maximum Z
  position (top of travel). All three towers home simultaneously to
  max Z, similar to delta printers.
- **Frame**: A structure that holds the three vertical rails at the
  correct angular spacing around the Z axis (typically 120° apart).

## Configuration Reference

The colinear tripteron kinematics are configured in the `[printer]`
section of the printer config file.

### [printer] section

```
[printer]
kinematics: colinear_tripteron
max_velocity: 300
max_accel: 3000
arm_angle: 30
#alpha_tower_rotation: 0
#beta_tower_rotation: 120
#gamma_tower_rotation: 240
```

- `kinematics: colinear_tripteron` — Selects colinear tripteron
  kinematics. Required.
- `max_velocity` — Maximum velocity (in mm/s) of the toolhead.
  Required.
- `max_accel` — Maximum acceleration (in mm/s²) of the toolhead.
  Required.
- `arm_angle` — The angle of each rigid arm from vertical, in
  degrees. Default is `30.0`. Typical range is 15–60 degrees. A
  smaller angle gives finer XY resolution (each unit of carriage
  travel produces less XY displacement) but reduces the reachable XY
  range. A larger angle gives more XY range but less precision per
  step. This value must be measured accurately — a 1° error
  significantly affects XY positioning.
- `alpha_tower_rotation` — Azimuth angle of tower A around the Z
  axis, in degrees. Default is `0.0`.
- `beta_tower_rotation` — Azimuth angle of tower B around the Z axis,
  in degrees. Default is `120.0`.
- `gamma_tower_rotation` — Azimuth angle of tower C around the Z
  axis, in degrees. Default is `240.0`.

### Stepper sections

Colinear tripteron printers use `[stepper_a]`, `[stepper_b]`, and
`[stepper_c]` for the three tower actuators. Do **not** use
`[stepper_x]`, `[stepper_y]`, or `[stepper_z]`.

Each stepper section accepts the standard stepper parameters:

```
[stepper_a]
step_pin:
dir_pin:
enable_pin:
microsteps: 16
rotation_distance: 40
endstop_pin:
position_endstop: 300
position_max: 300
homing_speed: 50
```

- `rotation_distance` — Distance the carriage moves per full stepper
  revolution (in mm). For a lead screw, this is pitch × number of
  starts. For example, a T8×8 lead screw (2mm pitch, 4 starts) has a
  rotation distance of 8.
- `position_endstop` — Distance (in mm) between the nozzle and the
  bed when the endstop triggers. This value is per-tower and must be
  calibrated for each tower individually to achieve a level bed.
- `position_max` — Maximum carriage position (in mm). Set this to the
  full travel distance of the linear rail.
- `endstop_pin` — Endstop switch pin. Each tower should have its own
  endstop at the maximum Z position (top of travel).
- `homing_speed` — Speed (in mm/s) at which the carriage moves toward
  the endstop during homing.

## Homing

All three towers home simultaneously to their maximum Z positions,
similar to delta printers. After homing completes, all three Cartesian
axes (X, Y, Z) are considered homed. There is no separate per-axis
homing — a `G28` command always homes all towers.

The toolhead position after homing is at `X=0 Y=0` and a Z position
determined by the `position_endstop` values of the three towers.

## Setup Guide

Follow these steps for initial setup of a colinear tripteron printer:

1. **Measure arm angle**: Measure the angle of the rigid arms from
   vertical as accurately as possible. Use a protractor, digital angle
   gauge, or CAD model. Set `arm_angle` in `[printer]` to the
   measured value. Even 1° of error will cause noticeable XY
   positioning errors.

2. **Set rotation_distance**: Calculate the rotation distance for your
   lead screws or belt drives. For a lead screw, this is
   pitch × starts. For example, an 8mm-pitch lead screw has a
   rotation distance of 8.

3. **Configure endstop pins**: Wire one endstop per tower at the top
   of each rail. Configure the `endstop_pin` in each stepper section.

4. **Set initial position_endstop**: Measure the approximate distance
   from the bed to the nozzle when the carriage triggers the endstop.
   Set `position_endstop` to this value for all three towers
   initially.

5. **Test homing**: Run `G28` and verify all three carriages move
   upward and trigger their endstops. The toolhead should be near the
   top of the build volume.

6. **Verify motion directions**: After homing, send small move
   commands and verify:
   - `G1 Z-10` moves the nozzle toward the bed.
   - `G1 X10` moves the nozzle in the expected X direction.
   - `G1 Y10` moves the nozzle in the expected Y direction.
   If any direction is inverted, adjust the `dir_pin` for the
   relevant stepper (add or remove `!` prefix).

7. **Calibrate endstops**: Follow the endstop calibration procedure
   below to level the bed.

8. **Set up bed mesh**: Configure `[bed_mesh]` for fine surface
   compensation.

## Calibration

### Arm Angle Calibration

The `arm_angle` parameter has a direct effect on XY positioning
accuracy. If the configured angle does not match the physical angle,
all XY moves will be scaled incorrectly.

To fine-tune `arm_angle`:

1. Home the printer with `G28`.
2. Position the nozzle at a known point on the bed (e.g., center).
3. Command a known XY move — for example, `G1 X50`.
4. Measure the actual displacement of the nozzle using calipers or a
   ruler against a reference.
5. If the actual displacement differs from the commanded distance,
   adjust `arm_angle`:
   - If the nozzle moved **less** than commanded, increase
     `arm_angle` slightly.
   - If the nozzle moved **more** than commanded, decrease
     `arm_angle` slightly.
6. Repeat until commanded and measured distances match.

### Tower Rotation Calibration

If the three towers are not spaced exactly 120° apart (due to frame
tolerances), adjust `alpha_tower_rotation`, `beta_tower_rotation`,
and `gamma_tower_rotation` to match the actual angles.

Misaligned tower rotations cause XY moves to not be orthogonal —
commanding a pure X move may produce some Y displacement, or vice
versa. If you observe this behavior, measure the actual tower angles
and update the rotation parameters.

### Endstop Calibration

All three endstops must be calibrated precisely for the bed to be
level. This is similar to endstop calibration on delta printers.

1. Home the printer with `G28`.
2. Move the nozzle to a point near the center of the bed and lower it
   until a piece of paper just drags under the nozzle (the standard
   "paper test").
3. Note the Z position. This will be your reference.
4. Move the nozzle to a point near tower A and repeat the paper test.
   If the nozzle is too high or too low relative to the center,
   adjust `position_endstop` in `[stepper_a]`.
   - If the nozzle is **too close** to the bed near tower A,
     **decrease** `position_endstop` for `[stepper_a]`.
   - If the nozzle is **too far** from the bed near tower A,
     **increase** `position_endstop` for `[stepper_a]`.
5. Repeat for towers B and C.
6. Re-home (`G28`) after each adjustment and re-test until the bed
   is level across all positions.

### Bed Mesh

After endstop calibration, it is recommended to use
[bed mesh](Bed_Mesh.md) (`[bed_mesh]`) for fine compensation of
remaining bed surface irregularities. Configure the mesh to cover the
printable area of your bed.

### rotation_distance Verification

Verify that each tower's `rotation_distance` is correct by commanding
a known Z move and measuring the actual carriage displacement. For
example, command `G1 Z100` from a reference point and measure the
actual travel. Adjust `rotation_distance` if the measured distance
does not match.

## Limitations and Notes

- **No dual carriage support**: The colinear tripteron kinematics do
  not support dual carriage or multi-extruder tool-changing
  configurations.
- **Build volume**: The printable XY area depends on `arm_angle` and
  the vertical travel of the actuators. A larger arm angle gives more
  XY range for a given Z travel but reduces Z height.
- **Arm rigidity**: The rigid arms must be stiff and free of play at
  the joints. Any flex or backlash in the arm linkages will directly
  affect print quality.
- **Symmetric vs asymmetric towers**: While the default tower
  rotations (0°, 120°, 240°) give a symmetric layout, asymmetric
  layouts are supported by setting custom rotation values. However,
  symmetric layouts are strongly recommended for predictable behavior.
