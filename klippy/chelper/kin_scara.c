// SCARA kinematics stepper pulse time generation
//
// Copyright (C) 2025       Tomáš Batelka <tomas.batelka@email.cz>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "compiler.h"  // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h"     // move_get_coord
#include <math.h>      // sqrt
#include <stddef.h>    // offsetof
#include <stdlib.h>    // malloc
#include <string.h>    // memset
#include <stdio.h>     // fprintf
#include "pyhelper.h"  // errorf

struct scara_calc_cache
{
    double L1_squared, L2_squared;
};

struct scara_stepper
{
    struct stepper_kinematics sk;
    double L1, L2, ecr;
    char type;
    struct scara_calc_cache cache;
    double offset_x, offset_y;
};

static double
scara_stepper_calc_position(struct stepper_kinematics* sk,
                            struct move* m, double move_time)
{
    struct scara_stepper* ss = container_of(sk, struct scara_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
    
    double x = c.x - ss->offset_x;
    double y = c.y - ss->offset_y;

    double cos_q2 = (x * x + y * y - ss->L1 * ss->L1 - ss->L2 * ss->L2) / (2 * ss->L1 * ss->L2);
    double q2 = (fabs(cos_q2) < 1e-9) ? acos(1e-6) : acos(cos_q2);
    double q1 = atan2(y, x) - atan2(ss->L2 * sin(q2), ss->L1 + ss->L2 * cos(q2));

    if (ss->type == 'a')
        return q1;
    else {
        return q2 + (q1 / ss->ecr);
    }
}

struct stepper_kinematics* __visible
scara_stepper_alloc(char type, double L1, double L2, 
    double offset_x, double offset_y, double ecr)
{
    struct scara_stepper* ss = malloc(sizeof(*ss));
    memset(ss, 0, sizeof(*ss));
    ss->type = type;
    ss->L1 = L1;
    ss->L2 = L2;
    ss->offset_x = offset_x;
    ss->offset_y = offset_y;
    ss->ecr = ecr;
    ss->cache.L1_squared = ss->L1 * ss->L1;
    ss->cache.L2_squared = ss->L2 * ss->L2;
    ss->sk.calc_position_cb = scara_stepper_calc_position;
    ss->sk.active_flags = AF_X | AF_Y;

    return &ss->sk;
}
