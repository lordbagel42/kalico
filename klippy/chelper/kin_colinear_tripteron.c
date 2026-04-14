// Colinear tripteron kinematics stepper pulse time generation
//
// Copyright (C) 2025
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "compiler.h"
#include "itersolve.h"
#include "trapq.h"

struct colinear_tripteron_stepper {
    struct stepper_kinematics sk;
    double cx, cy;
};

static double
colinear_tripteron_calc_position(struct stepper_kinematics *sk,
                                  struct move *m, double move_time)
{
    struct colinear_tripteron_stepper *cs =
        container_of(sk, struct colinear_tripteron_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
    return cs->cx * c.x + cs->cy * c.y + c.z;
}

struct stepper_kinematics * __visible
colinear_tripteron_stepper_alloc(double cx, double cy)
{
    struct colinear_tripteron_stepper *cs = malloc(sizeof(*cs));
    memset(cs, 0, sizeof(*cs));
    cs->sk.calc_position_cb = colinear_tripteron_calc_position;
    cs->sk.active_flags = AF_X | AF_Y | AF_Z;
    cs->cx = cx;
    cs->cy = cy;
    return &cs->sk;
}
