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
    double magnify_factor;
};


#define M_GALVO_MOD_BC_AXIS
#ifdef M_GALVO_MOD_BC_AXIS



static double
galvo_stepper_b_calc_position(struct stepper_kinematics *sk, struct move *m
                                  , double move_time)
{
    struct galvo_stepper *gs = container_of(sk, struct galvo_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
    double adjusted_b = c.b - gs->half_distance_galvo;
    //double angle = atan2(adjusted_b, gs->focus_distance);
    double angle = adjusted_b/gs->focus_distance;
    angle = (angle + gs->hradiation_angle) * gs->magnify_factor;  
    return angle;    
    //return c.x + c.y;


}

static double
galvo_stepper_c_calc_position(struct stepper_kinematics *sk, struct move *m
                                   , double move_time)
{

    struct galvo_stepper *gs = container_of(sk, struct galvo_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
    double adjusted_c = c.c - gs->half_distance_galvo;
    //double angle = atan2(adjusted_c, gs->focus_distance);
    double angle =  adjusted_c/gs->focus_distance;
    angle = (angle + gs->hradiation_angle) * gs->magnify_factor;
    return angle;   
    //return c.x - c.y;


}


static double
cart_stepper_a_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    return move_get_coord(m, move_time).a;

}

static double
cart_stepper_d_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    return move_get_coord(m, move_time).d;

}


struct stepper_kinematics * __visible
galvo_stepper_alloc(char type,double hradiation_angle, double focus_distance, double half_distance_galvo, double magnify_factor)
{
    struct galvo_stepper *gs = malloc(sizeof(*gs));
    memset(gs, 0, sizeof(*gs));

    gs->hradiation_angle = hradiation_angle;
    gs->focus_distance = focus_distance; 
    gs->half_distance_galvo = half_distance_galvo;
    gs->magnify_factor = magnify_factor;

  
    if (type == 'b')
    {
         gs->sk.calc_position_cb = galvo_stepper_b_calc_position;
         gs->sk.active_flags = AF_B;
    }
    else if (type == 'c')
    {
         gs->sk.calc_position_cb = galvo_stepper_c_calc_position;
         gs->sk.active_flags= AF_C;
    }
    else if (type == 'a')
    {
          gs->sk.calc_position_cb = cart_stepper_a_calc_position;
          gs->sk.active_flags = AF_A;
    }
    else if (type == 'd')
    {
        gs->sk.calc_position_cb = cart_stepper_d_calc_position;
        gs->sk.active_flags = AF_D;
    }
    return &gs->sk;
    

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

