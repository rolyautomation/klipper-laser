// Galvo kinematics stepper pulse time generation
//
// Copyright (C) 2024  Leo QU <leo@rolyautomation.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord

//#define MAX_GALVO_ANGLE 0.35        //rad
//#define THETA_FOCUS_DISTANCE 160    //mm
//#define HALF_GALVO_DISTANCE THETA_FOCUS_DISTANCE*tan(MAX_GALVO_ANGLE)


struct galvo_stepper {
    struct stepper_kinematics sk;
    double hradiation_angle, focus_distance, half_distance_galvo;
};


//#define M_GALVO_MOD_BC_AXIS
#ifdef M_GALVO_MOD_BC_AXIS


static double
galvo_stepper_b_calc_position(struct stepper_kinematics *sk, struct move *m 
                              ,double move_time)
{
    double adjusted b = move_get_coord(m, move time).b - HALF_GALVO_DISTANCE;
    double angle = atan2(adjusted_b, THETA_FOCUS_DISTANCE);
    return angle;

}
static double
galvo_stepper_c_calc_position(struct stepper_kinematics *sk, struct move *m
                               ,double move time)
{
    double adjusted_c = move_get_coord(m, move time).c- HALF_GALVO_DISTANCE;
    double angle = atan2(adjusted_c, THETA_FOCUS_DISTANCE);
    return angle;

}


struct stepper_kinematics * __visible
galvo_stepper_alloc(char axis)
{
    struct stepper_kinematics *sk= malloc(sizeof(*sk));
    memset(sk,0,sizeof(*sk));
    if (axis =='b') {
        sk->calc_position_cb = galvo_stepper_b_calc_position;
        sk->active_flags = AF_Z;
    }else if(axis =='c') {
        sk->calc_position_cb = galvo_stepper_c_calc_position;
        sk->active_flags = AF_Z;
    }
    return sk;
}

#else



static double
galvo_stepper_b_calc_position(struct stepper_kinematics *sk, struct move *m
                                  , double move_time)
{
    struct galvo_stepper *gs = container_of(sk, struct galvo_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
    double adjusted_b = c.x - gs->half_distance_galvo;
    double angle = atan2(adjusted_b, gs->focus_distance);
    angle = angle + gs->hradiation_angle;  
    return angle;    
    //return c.x + c.y;


}

static double
galvo_stepper_c_calc_position(struct stepper_kinematics *sk, struct move *m
                                   , double move_time)
{

    struct galvo_stepper *gs = container_of(sk, struct galvo_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
    double adjusted_c = c.y - gs->half_distance_galvo;
    double angle = atan2(adjusted_c, gs->focus_distance);
    angle = angle + gs->hradiation_angle;
    return angle;   
    //return c.x - c.y;


}

struct stepper_kinematics * __visible
galvo_stepper_alloc(char type,double hradiation_angle, double focus_distance, double half_distance_galvo)
{
    struct galvo_stepper *gs = malloc(sizeof(*gs));
    memset(gs, 0, sizeof(*gs));

    gs->hradiation_angle = hradiation_angle;
    gs->focus_distance = focus_distance; 
    gs->half_distance_galvo = half_distance_galvo;

    if (type == '+')
        gs->sk.calc_position_cb = galvo_stepper_b_calc_position;
    else if (type == '-')
        gs->sk.calc_position_cb = galvo_stepper_c_calc_position;
    gs->sk.active_flags = AF_X | AF_Y;

    return &gs->sk;
    

}


#endif

