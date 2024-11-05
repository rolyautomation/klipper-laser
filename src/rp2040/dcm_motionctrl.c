// Handling of dcm_motioncrtl.
//
// Copyright (C) 2024-2028  jinqiang <jinqiang@ecomedge.io>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_*
#include "board/armcm_boot.h" // armcm_enable_irq
#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_is_before
#include "command.h" // DECL_COMMAND
#include "internal.h" // PIO0_IRQ_0_IRQn
#include "sched.h" // struct timer
//#include "stepper.h" // stepper_event
//#include "xy2_stepper.h"  // 
//#include "trsync.h" // trsync_add_signal

//int  send_dcmctrlrun_instr(uint ab_pin, uint32_t dcm_instr);
int  send_dcmctrlrun_instr(unsigned int ab_pin, uint32_t dcm_instr);



#define M_IDEL_MOTOR     (0)
#define M_CW_MOTOR       (1)
#define M_CCW_MOTOR      (2)
#define M_BRAKE_MOTOR    (3)

#define M_MOTOR_BIT      (24)
#define M_BIT_FRPOS      (2)


struct dcm_motion_data {
    uint8_t rundir;
    uint8_t runmode;
    uint32_t dcm_runtime, dcm_waittime;
    uint8_t ab_wkmode;
    uint8_t ab_pin_start;

};



void
command_config_ab_dcmotor(uint32_t *args)
{

    struct dcm_motion_data *s = oid_alloc(args[0], command_config_ab_dcmotor, sizeof(*s));
	s->ab_pin_start = args[1];
	s->ab_wkmode = args[2];

}


DECL_COMMAND(command_config_ab_dcmotor, "config_ab_dcmotor oid=%c ab_min_pin=%c"
             " wkm=%c");



// Return the 'struct dcm_motion_data' for a given dcm_motion_data oid
static struct dcm_motion_data *
ab_dcmotor_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_ab_dcmotor);
}


void
command_rinstr_ab_dcmotor(uint32_t *args)
{
    uint32_t cur_instr0_val = 0;
    uint32_t cur_instr1_val = 0;
    uint32_t mcw_val = M_CW_MOTOR;
    uint32_t mccw_val = M_CCW_MOTOR;    
    uint8_t oid = args[0];
    struct dcm_motion_data *s = ab_dcmotor_oid_lookup(oid);
    s->dcm_runtime = args[1];
    s->dcm_waittime = args[2];	
    s->rundir = args[3];
    s->runmode = args[4];  
    if (s->ab_wkmode > 0)
    {
        mcw_val = M_CCW_MOTOR;
        mccw_val = M_CW_MOTOR;          
    }
    //irq_disable();
    //irq_enable();
    if (s->runmode == 1) 
    {
        cur_instr0_val  =  (((M_IDEL_MOTOR << M_BIT_FRPOS) | M_IDEL_MOTOR ) <<  M_MOTOR_BIT);
        cur_instr0_val  =  cur_instr0_val | s->dcm_runtime;

    } 
    else if (s->runmode == 2) 
    {
        cur_instr0_val  =  (((M_BRAKE_MOTOR << M_BIT_FRPOS) | M_BRAKE_MOTOR ) <<  M_MOTOR_BIT);
        cur_instr0_val  =  cur_instr0_val | s->dcm_runtime;
    }
    else if (s->runmode == 3) 
    {
        cur_instr0_val  =  (((M_BRAKE_MOTOR << M_BIT_FRPOS) | M_IDEL_MOTOR ) <<  M_MOTOR_BIT);
        cur_instr0_val  =  cur_instr0_val | s->dcm_runtime;
    }    
    else if (s->runmode == 4) 
    {
        cur_instr0_val  =  (((M_IDEL_MOTOR << M_BIT_FRPOS) | M_BRAKE_MOTOR ) <<  M_MOTOR_BIT);
        cur_instr0_val  =  cur_instr0_val | s->dcm_runtime;
    } 
    else
    {
        cur_instr0_val  =  M_BRAKE_MOTOR << M_BIT_FRPOS;
        if ( s->rundir > 0 )
        {
            cur_instr0_val =  cur_instr0_val | mcw_val;
        }
        else
        {
            cur_instr0_val =  cur_instr0_val | mccw_val;
        }
        cur_instr0_val =  (cur_instr0_val << M_MOTOR_BIT) | s->dcm_runtime;
        cur_instr1_val  =  (((M_IDEL_MOTOR << M_BIT_FRPOS) | M_BRAKE_MOTOR ) <<  M_MOTOR_BIT);
        cur_instr1_val  =  cur_instr0_val | s->dcm_waittime;

    } 
    if (cur_instr0_val > 0)
    {
        send_dcmctrlrun_instr(s->ab_pin_start, cur_instr0_val);    
    }
    if (cur_instr1_val > 0)
    {
        send_dcmctrlrun_instr(s->ab_pin_start, cur_instr1_val);    
    }    
   

}
DECL_COMMAND(command_rinstr_ab_dcmotor, "rinstr_ab_dcmotor oid=%c rtm=%u wtm=%u dir=%c wmod=%c");

