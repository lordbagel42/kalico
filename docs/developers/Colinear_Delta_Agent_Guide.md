# Colinear Delta: Implementation Guide for AI Agents

This guide is written for an automated coding agent (or a human) that needs to
**implement, modify, or extend** the `colinear_delta` kinematic in Kalico, or
build a similar multi-mechanism kinematic. It is deliberately explicit about the
math, the invariants, the source layout, and the failure modes, so that an agent
can reproduce or safely change the feature from first principles.

For the user-facing description see [Colinear_Delta.md](../Colinear_Delta.md).
For general kinematics background see [Kinematics.md](../Kinematics.md) and the
"Adding new kinematics" section of [Code_Overview.md](../Code_Overview.md).

---

## 1. What the machine is (the model you must implement)

A colinear delta is **two independent linear-delta mechanisms sharing three
physical vertical rails**:

- toolhead delta: steppers `a`, `b`, `c`, effector = nozzle, world position `T`.
- bed delta: steppers `d`, `e`, `f`, effector = build plate, world position `B`.

Each rail carries two carriages (one per mechanism). So there are **6 actuators
and 6 endstops**, but the printer exposes only **3 cartesian axes**: the
commanded coordinate `P = (x, y, z)` is the **nozzle position relative to the
part**.

The two mechanisms share every move. Define:

- `s = motion_split` (fraction of the move done by the toolhead, default 0.5),
- `theta = secondary_rotation` (rotation of the bed frame about Z, default 180 deg),
- `Rz(theta)` the standard Z rotation matrix.

The **defining physical relation** is:

```
T - B = Rz(theta) * P
```

and the split assigns each mechanism a share of that relative motion:

```
T = T_c + s       * Rz(theta) * P
B = B_c - (1 - s) * Rz(theta) * P
```

`T_c`, `B_c` are constant offsets fixed at homing (they drop out of all relative
motion; see §5). Sanity check at `s = 0.5`, `theta = 180`: `T` and `B` move by
half of `P` in opposite world directions, and `T - B = Rz(180) P` has the full
magnitude of `P`. At `s = 1`, `theta = 0` this collapses to an ordinary delta.

> Assumptions baked into the model (document them, do not silently break them):
> all six rails are vertical; each effector hangs between its two carriages
> (standard `+sqrt` delta solution); the machine homes centered; the two
> mechanisms home at opposite ends of the shared rails.

---

## 2. Why this maps cleanly onto Klipper's solver

Klipper generates steps with a per-stepper C callback
`calc_position(sk, move, move_time)` that receives **only** the move coordinate
`c = (c.x, c.y, c.z) = P` and must return that stepper's carriage position. It
is stateless and independent per stepper. Therefore any kinematic that is a
**fixed affine map of `P` per stepper** fits with no changes to the solver.

For a colinear delta each carriage runs the ordinary delta sphere solve, but on
an *effector coordinate* that is a linear function of `P`:

```
ex = cos_coef * x - sin_coef * y
ey = sin_coef * x + cos_coef * y
ez = z_coef  * z
carriage = sqrt(arm2 - (tower_x - ex)^2 - (tower_y - ey)^2) + ez
```

The per-mechanism coefficients follow directly from §1
(`T = s*Rz(theta)*P`, `B = -(1-s)*Rz(theta)*P`):

| mechanism | cos_coef        | sin_coef        | z_coef     |
|-----------|-----------------|-----------------|------------|
| toolhead  | `s*cos(theta)`  | `s*sin(theta)`  | `s`        |
| bed       | `-(1-s)*cos(theta)` | `-(1-s)*sin(theta)` | `-(1-s)` |

The shared `tower_x, tower_y` are the world positions of the three rails
(angles default `210/330/90`, radius `delta_radius`), identical for both
mechanisms because the rails are shared.

Key consequence: **no new solver machinery is needed**, only a small C alloc
that stores the three extra coefficients. `kin_delta.c`'s `delta_stepper_alloc`
cannot be reused as-is because it has no coefficients (it assumes
`ex,ey,ez == x,y,z`).

---

## 3. Forward kinematics (used for homing / status)

`calc_position` must recover `P` from the six carriage positions. Trilateration
(`klippy/mathutil.py:trilateration`, exactly three spheres) gives each
mechanism's **world** effector position from its three carriages:

```
T = trilateration([(tower_x_i, tower_y_i, carriage_i) for i in a,b,c], arm2_abc)
B = trilateration([(tower_x_i, tower_y_i, carriage_i) for i in d,e,f], arm2_def)
P = Rz(-theta) * (T - B)
```

This is **independent of `s`** (because `T - B = Rz(theta) P` regardless of how
the move was split), which makes it robust. It also means FK only needs three
spheres per mechanism - never try to trilaterate all six at once.

Verify any change with a round-trip *and* a physical check: for several `P`,
compute the six carriages via the coefficient map, run `calc_position`, and
assert you recover `P`; separately assert `T_eff - B_eff == Rz(theta) P`. The
round-trip alone only proves internal self-consistency, not that the coefficients
match the machine - both checks are required.

---

## 4. The redundancy, and why homing is sequential

Six actuators, three controlled DOF (`P`). The extra three DOF are the
"common mode" (roughly, moving both effectors together): `P`-moves never command
it, but the six endstops *do* pin it at home. This is the single most important
subtlety.

For a relative-Z move the toolhead carriages and bed carriages move in
**opposite** directions (`z_coef` is `+s` vs `-(1-s)`). A single coordinated
`P`-space homing move therefore cannot bring both sets onto endstops that sit at
opposite ends of the rails: whichever mechanism triggers first would have its
steppers halted by trsync while the coupled move keeps demanding they move -
i.e. binding, or a "No trigger after full movement" error.

Solution used here: **home the two mechanisms sequentially**, each as an
ordinary 3-tower delta (three endstops, no redundancy within a mechanism):

1. `home_rails(bed_rails, forcepos_bed, home_bed)` - bed carriages rise to their
   top endstops as relative Z drops.
2. `home_rails(toolhead_rails, home_bed, home_tool)` - toolhead carriages rise to
   their top endstops as relative Z climbs; the bed carriages ride along.

Each mechanism's home position is obtained by inverting the coefficient map from
its endstop-derived effector:

```
T_eff = trilateration(toolhead abs_endstops)   # = s*Rz(theta)*home_tool
home_tool = (1/s)     * Rz(-theta) * T_eff
B_eff = trilateration(bed abs_endstops)         # = -(1-s)*Rz(theta)*home_bed
home_bed  = (-1/(1-s)) * Rz(-theta) * B_eff
```

where `abs_endstop_i = position_endstop_i + sqrt(arm2_i - radius^2)` (the delta
convention). The machine ends homing at `home_tool` (nozzle parked far from the
part), and that becomes `home_position` / `max_z`.

Pitfalls encountered (and handled) here:

- The **second** home must start from the first home's position so it does not
  lie about the already-homed mechanism (pass `home_bed` as the toolhead home's
  `forcepos`).
- `check_move` must permit the homing moves. Homing drives the mechanisms to
  **opposite Z extremes**, so the bed home target is *below* `min_z`. The
  homing exception therefore allows a centered move (`xy == home xy`) anywhere in
  `[home_bed_z, home_tool_z]`, not just up to `max_z` as a plain delta does.

---

## 5. Why the constants `T_c`, `B_c` do not need to be represented

The coefficient map uses no additive constant (it assumes each effector is at
the origin when `P = 0`). That is fine because:

- Only **relative** motion matters for printing, and `carriage(P+dP) -
  carriage(P)` depends solely on the linear coefficients, not on any constant.
- Homing calls `set_position(home_*)`, which recomputes each stepper's step
  reference from the map at that point, absorbing the physical Z offset - exactly
  as a plain delta absorbs its endstop offset. This is valid as long as the
  machine homes **centered** (effector XY = 0 at home), so the only unmodelled
  constant is in Z, where the map is linear.

If you ever add an XY home offset you must reintroduce the constants, because the
sphere solve is nonlinear in XY.

---

## 6. Source map (what to touch)

Loading is by module name: `kinematics: colinear_delta` causes
`klippy/toolhead.py` to `import_module("klippy.kinematics.colinear_delta")` and
call its `load_kinematics(toolhead, config)`. There is no registry table.

- **`klippy/chelper/kin_colinear_delta.c`** - the per-stepper solve.
  `struct colinear_delta_stepper` holds `arm2, tower_x, tower_y, cos_coef,
  sin_coef, z_coef`; `colinear_delta_stepper_alloc(...)` sets
  `calc_position_cb` and `active_flags = AF_X | AF_Y | AF_Z`.
- **`klippy/chelper/__init__.py`** - three edits to register the C:
  add `"kin_colinear_delta.c"` to `SOURCE_FILES`; add a
  `defs_kin_colinear_delta` cdef string matching the alloc signature; append it
  to `defs_all`. The shared library rebuilds automatically when a source file's
  mtime changes.
- **`klippy/kinematics/colinear_delta.py`** - `ColinearDeltaKinematics` plus
  `load_kinematics`. It reuses `DeltaCalibration` by importing it from
  `klippy.kinematics.delta`.
- **Docs**: [`docs/Config_Reference.md`](../Config_Reference.md) (allowed-values
  list + a "Colinear Delta Kinematics" section), [`docs/Colinear_Delta.md`](../Colinear_Delta.md),
  and this file. New pages must be added to the nav in
  `docs/_kalico/mkdocs.yml` because docs are built with `mkdocs build --strict`.

No changes are needed to `toolhead.py`, `mathutil.py`, `stepper.py`, or
`klippy/kinematics/__init__.py`.

---

## 7. Required method contract (`ColinearDeltaKinematics`)

Implement the standard Kalico kinematic interface. Deltas from
`klippy/kinematics/delta.py`:

- `__init__(self, toolhead, config)` - read `motion_split`, `secondary_rotation`,
  `delta_radius`, `print_radius`, arm lengths and angles; build six rails via
  `stepper.LookupMultiRail(cfg, need_position_minmax=False,
  default_position_endstop=...)` (`a`->b/c share a's endstop, `d`->e/f share
  d's); compute per-mechanism coefficients; `setup_itersolve(
  "colinear_delta_stepper_alloc", arm2, tx, ty, cos_coef, sin_coef, z_coef)` per
  rail; register `set_trapq` + `register_step_generator` for all six steppers;
  compute `home_tool`/`home_bed` (§4); compute the envelope (§8); validate
  (§9); `supports_dual_carriage = False`.
- `get_steppers` - all six.
- `calc_position(stepper_positions)` - FK per §3; index all six rail names.
- `set_position(newpos, homing_axes)` - loop **all six** rails; clear
  `need_home` when `homing_axes == (0,1,2)`.
- `home(homing_state)` - `set_axes([0,1,2])` then the two sequential
  `home_rails` calls of §4.
- `clear_homing_state`, `_motor_off` - as delta.
- `check_move(move)` - envelope + Z speed limiting + the widened homing
  exception (§4).
- `get_status(eventtime)` - `homed_axes`, `axis_minimum`, `axis_maximum`,
  `cone_start_z`.
- `get_calibration()` - a `DeltaCalibration` built from the **toolhead** rails
  only (the bed is assumed symmetric). `DELTA_CALIBRATE` is duck-typed on this
  method and only ever reads `stepper_a/b/c`.
- `load_kinematics(toolhead, config)` - return the instance.

---

## 8. Envelope

The sphere argument depends only on effector **XY**, so validity is a pure XY
reach test per mechanism. A move at relative XY radius `rho` puts the toolhead
effector at `s*rho` and the bed effector at `(1-s)*rho`, so the reachable
relative radius is bounded by the tighter of:

```
tool_reach = (min_tool_arm - radius) / s
bed_reach  = (min_bed_arm  - radius) / (1 - s)
```

Reject configs whose `print_radius` reaches or exceeds `min(tool_reach,
bed_reach)`. Compute the "slow" and "very slow" speed-taper radii with the same
per-mechanism formula as delta, divided by each mechanism's XY scale, and take
the minimum across mechanisms. `max_z = home_tool_z`; `min_z` from
`minimum_z_position`.

---

## 9. Validation and gotchas checklist

- `0 < motion_split < 1` (both mechanisms must move; `getfloat(above=0, below=1)`).
- `arm_length > delta_radius` for every tower.
- `print_radius < min(tool_reach, bed_reach)` (raise `config.error` with the
  computed maximum).
- FK uses three spheres per mechanism, never six.
- `calc_position` tolerates the full six-key stepper dict that homing and
  `delta_calibrate` pass in.
- The homing exception in `check_move` must span `[home_bed_z, home_tool_z]`,
  which includes Z values below `min_z`.
- Coefficients must satisfy the physical relation `T_eff - B_eff == Rz(theta)P`,
  not merely round-trip. If you only test the round-trip you can ship a sign
  error that silently cancels XY motion at `s = 0.5, theta = 180`.

---

## 10. How to verify a change

1. **C compiles / symbol loads.** In a venv with `cffi`:
   `python -c "import klippy.chelper as c; _,lib=c.get_ffi();
   print(hasattr(lib,'colinear_delta_stepper_alloc'))"`.
2. **Math.** A standalone script (no Klipper import needed - inline the
   `trilateration` from `mathutil.py`) that checks both the round-trip and
   `T_eff - B_eff == Rz(theta)P` for several `s`/`theta`/`P`. Expect errors at
   machine precision (~1e-13).
3. **End to end in batch mode.** Build the host simulator MCU dictionary
   (`make distclean; echo CONFIG_MACH_SIMU=y > .config; make olddefconfig; make`
   -> `out/klipper.dict`; needs only `gcc`), write a six-stepper config (use
   **numeric** pin names - the simulator dict has no pin enumeration), and run
   `python -m klippy config.cfg -i moves.gcode -o /dev/null -d out/klipper.dict -v`.
   Confirm: config loads, `G28` completes with no "out of range"/"no trigger"
   error, in-envelope `G1` moves are accepted, out-of-envelope moves are
   rejected, and `STEPPER_BUZZ` works for both an `a` and a `d` stepper. Compare
   `GET_POSITION` after `G28` against a plain delta on the same simulator to
   distinguish real bugs from the simulator's homing-retract artifact (both
   under-report by `homing_retract_dist`).
4. **Lint/format.** `ruff check` and `ruff format --check` on the Python module.
