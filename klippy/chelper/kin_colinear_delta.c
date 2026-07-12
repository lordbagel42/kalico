// Colinear delta kinematics stepper pulse time generation
//
// A colinear delta is two independent linear-delta mechanisms that share the
// same three physical (colinear) rails: a "toolhead" delta positions the
// nozzle and a "bed" delta positions the build plate. The printed geometry is
// the relative position of the nozzle with respect to the part, so each move
// coordinate is shared between the two mechanisms.
//
// Every carriage obeys the standard delta sphere constraint, but on an
// effector coordinate that is a fixed affine map of the requested move
// coordinate. The map encodes the motion split factor and the (Z-axis)
// rotation of the secondary/bed frame. See docs/Colinear_Delta.md and
// docs/developers/Colinear_Delta_Agent_Guide.md for the derivation.
//
// Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2026  Kalico contributors
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord

struct colinear_delta_stepper {
    struct stepper_kinematics sk;
    double arm2, tower_x, tower_y;
    // Affine map from the move coordinate to this carriage's effector coord:
    //   ex = cos_coef*x - sin_coef*y
    //   ey = sin_coef*x + cos_coef*y
    //   ez = z_coef*z
    double cos_coef, sin_coef, z_coef;
};

static double
colinear_delta_stepper_calc_position(struct stepper_kinematics *sk
                                     , struct move *m, double move_time)
{
    struct colinear_delta_stepper *ds = container_of(
        sk, struct colinear_delta_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
    double ex = ds->cos_coef * c.x - ds->sin_coef * c.y;
    double ey = ds->sin_coef * c.x + ds->cos_coef * c.y;
    double ez = ds->z_coef * c.z;
    double dx = ds->tower_x - ex, dy = ds->tower_y - ey;
    return sqrt(ds->arm2 - dx*dx - dy*dy) + ez;
}

struct stepper_kinematics * __visible
colinear_delta_stepper_alloc(double arm2, double tower_x, double tower_y
                             , double cos_coef, double sin_coef, double z_coef)
{
    struct colinear_delta_stepper *ds = malloc(sizeof(*ds));
    memset(ds, 0, sizeof(*ds));
    ds->arm2 = arm2;
    ds->tower_x = tower_x;
    ds->tower_y = tower_y;
    ds->cos_coef = cos_coef;
    ds->sin_coef = sin_coef;
    ds->z_coef = z_coef;
    ds->sk.calc_position_cb = colinear_delta_stepper_calc_position;
    ds->sk.active_flags = AF_X | AF_Y | AF_Z;
    return &ds->sk;
}
