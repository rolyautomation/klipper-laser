// mmsa multi motor single axis kinematics
//
// Copyright (C) 2024-2028  jinqiang <jinqiang@ecomedge.io>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // struct move

#define DUMMY_T 500.0

struct multimotor_axis_stepper {
    struct stepper_kinematics sk;
    struct stepper_kinematics *orig_sk;
    struct move m;
    double a_scale, a_offs;
};

double
multimotor_axis_calc_position(struct stepper_kinematics *sk, struct move *m
                            , double move_time)
{
    struct multimotor_axis_stepper *ma = container_of(
            sk, struct multimotor_axis_stepper, sk);
    struct coord pos = move_get_coord(m, move_time);
    ma->m.start_pos.x = pos.x;
    ma->m.start_pos.y = pos.y;
    ma->m.start_pos.z = pos.z;
    //ma->m.start_pos.a = pos.a * ma->a_scale + ma->a_offs;
    ma->m.start_pos.a = pos.a * ma->a_scale;  
    ma->m.start_pos.b = pos.b; 
    ma->m.start_pos.c = pos.c;           
    return ma->orig_sk->calc_position_cb(ma->orig_sk, &ma->m, DUMMY_T);

}

void __visible
multimotor_axis_set_sk(struct stepper_kinematics *sk
                     , struct stepper_kinematics *orig_sk)
{
    struct multimotor_axis_stepper *ma = container_of(
            sk, struct multimotor_axis_stepper, sk);
    ma->sk.calc_position_cb = multimotor_axis_calc_position;
    ma->sk.active_flags = orig_sk->active_flags;
    ma->orig_sk = orig_sk;
}

int __visible
multimotor_axis_set_transform(struct stepper_kinematics *sk, char axis
                            , double scale, double offs)
{
    struct multimotor_axis_stepper *ma = container_of(
            sk, struct multimotor_axis_stepper, sk);
    if (axis == 'a') {
        ma->a_scale = scale;
        ma->a_offs = offs;
        if (!scale)
            ma->sk.active_flags &= ~AF_A;
        else if (scale && ma->orig_sk->active_flags & AF_A)
            ma->sk.active_flags |= AF_A;
        return 0;
    }
    return -1;
    
}

struct stepper_kinematics * __visible
multimotor_axis_alloc(void)
{
    struct multimotor_axis_stepper *ma = malloc(sizeof(*ma));
    memset(ma, 0, sizeof(*ma));
    ma->m.move_t = 2. * DUMMY_T;
    return &ma->sk;
}
