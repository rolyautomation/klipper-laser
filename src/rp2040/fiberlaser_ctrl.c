// Handling of fiberlaser control drivers.
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
#include "fiberlaser_ctrl.h"  // 
#include "trsync.h" // trsync_add_signal



#define M_FIBERLASER_CTRL_FUN   (1)
#ifdef M_FIBERLASER_CTRL_FUN

struct stepper_move_fiber {
    struct move_node node;
    uint8_t workmode;
    uint8_t workstep;
    uint8_t workpower;
    uint8_t flags;
};



enum {
    SF_LAST_DIR=1<<0, SF_NEXT_DIR=1<<1, SF_INVERT_STEP=1<<2, SF_NEED_RESET=1<<3,
    SF_SINGLE_SCHED=1<<4, SF_HAVE_ADD=1<<5
};

//10s: from power on to normal
//pin1-8 (D0-D7)
#define  M_POWER_IO_TOTAL           (8)
//pin9
#define  M_GPIO_LATCH_NUM           (8)
//pin18
#define  M_GPIO_MOEE_NUM            (9)
//pin19
#define  M_GPIO_BSEM_NUM            (10)
//pin20
#define  M_GPIO_PRRSYNC_NUM         (11)
//pin22
#define  M_GPIO_POINTERL_NUM        (12)
//pin23
#define  M_GPIO_EMERGENCYOFF_NUM    (13)

//pin16
#define  M_GPIO_IN_ALARM16_NUM       (14)
//pin21
#define  M_GPIO_IN_ALARM21_NUM       (15)


//Emission Modulation(booster,Laser Modulation), Emission Enable(Master Oscillator)

#define  M_WK_IDLE_MODE         (0)
#define  M_WK_CHGPOWER_MODE     (1)
#define  M_WK_POWERON_MODE      (2)
#define  M_WK_POWEROFF_MODE     (3)
#define  M_WK_LASERON_MODE      (4)
#define  M_WK_LASEROFF_MODE     (5)
#define  M_WK_REDLEDON_MODE      (6)
#define  M_WK_REDLEDOFF_MODE     (7)
#define  M_WK_ESTOPON_MODE       (8)
#define  M_WK_ESTOPOFF_MODE      (9)



#define  M_WK_CHGPOWER_STEP     (5)
#define  M_WK_POWERON_STEP      (3)
#define  M_WK_POWEROFF_STEP     (3)
//#define  M_WK_LASERONOFF_STEP (3)

#define  M_LASER_EE_FLAG          (0x0001)
#define  M_LASER_EM_FLAG          (0x0002)
#define  M_LASER_RLED_FLAG        (0x0004)
#define  M_LASER_STOP_FLAG        (0x0008)
#define  M_LASER_SETPOWER_FLAG    (0x0010)

#define  M_ST_PIN16_BIT           (0x4000)
#define  M_ST_PIN21_BIT           (0x8000)

#define  M_HIGH_IO    (1)
#define  M_LOW_IO     (0)
#define  M_MIN_DEDAYTIME_US    (1)


#define  M_MIN_US_SYSTEM     (4)

//255*0.97
#define  M_MAX_FIBER_POWER_VAL    (247)
//255*0.1
#define  M_MIN_FIBER_POWER_VAL    (25)
//255*0.2
#define  M_P20_FIBER_POWER_VAL    (51)

#define  M_LASER_ON_CMD       (1)
#define  M_LASER_OFF_CMD      (2)

//#define  M_DEBUG_INFO_EN       (1)
#define  M_DEBUG_INFO_EN       (0)

struct stepper_fiber {
    struct timer time;
    uint32_t STA_EE_EM_TIME_us; //5ms = 5000us
    uint32_t PLATCH_BEFORE_TIME_us; //1us =   
    uint32_t PLATCH_AFTER_TIME_us; //2us  
    uint32_t PLATCH_CHG_INTERTIME_us; //4us      
    uint32_t STA_EM_EE_TIME_us; //1us 
    uint32_t STA_MIN_DELAYTIME_us; //1us 
    uint8_t  gpio_base_pin;
    uint8_t  workmode;
    uint8_t  workstep;
    uint8_t  workpower;    
    //uint32_t  interval;
    uint8_t  recvmode;
    uint8_t  recvpower;
    uint32_t workflag;

    uint32_t period;
    uint32_t level;  
    uint8_t  fibertype;  

    struct move_queue_head mq;
    struct trsync_signal stop_signal;
    // gcc (pre v6) does better optimization when uint8_t are bitfields
    uint8_t flags : 8;
};

int handle_fiber_timeseq(struct stepper_fiber *s)
{
    int iret = 0;
    uint8_t  workmode = s->workmode;
    uint8_t  workstep = s->workstep;
    uint32_t curtime = timer_read_time();
    //uint32_t min_next_time = 0; //curtime + timer_from_us(10000); //10ms    

    switch(workmode)
    {
        case M_WK_POWERON_MODE :
            if (workstep > 0)
            {
                if (M_WK_POWERON_STEP == workstep) 
                {
                    set_agpio_outstate(s->gpio_base_pin+M_GPIO_MOEE_NUM, M_HIGH_IO);
                    s->time.waketime =  curtime +  timer_from_us(s->STA_EE_EM_TIME_us);
                    s->workstep = s->workstep - 1;
                    s->workflag = s->workflag | M_LASER_EE_FLAG; 
                    iret = 0;

                } else if ((M_WK_POWERON_STEP-1) == workstep)  
                {
                    set_agpio_outstate(s->gpio_base_pin+M_GPIO_BSEM_NUM, M_HIGH_IO);
                    s->workstep = 0;
                    s->workflag = s->workflag | M_LASER_EM_FLAG;
                    s->workmode = M_WK_IDLE_MODE;
                    s->time.waketime =  curtime +  timer_from_us(s->STA_MIN_DELAYTIME_us);
                    iret = 1;
                }

            }
            else
            {
                s->workmode = M_WK_IDLE_MODE;
                s->time.waketime =  curtime +  timer_from_us(s->STA_MIN_DELAYTIME_us);
                iret = 1;
            }
            break;
        case M_WK_CHGPOWER_MODE :
            if (workstep > 0)
            {
                if (M_WK_CHGPOWER_STEP == workstep) 
                {
                    
                    set_manygpio_outstate(s->gpio_base_pin, M_POWER_IO_TOTAL, s->workpower);
                    s->time.waketime =  curtime +  timer_from_us(s->PLATCH_BEFORE_TIME_us);
                    s->workstep = s->workstep - 1;
                    s->workflag = s->workflag | M_LASER_SETPOWER_FLAG; 
                    #if M_DEBUG_INFO_EN
                    //output("fiber:[%u,%u]",3,s->PLATCH_BEFORE_TIME_us);
                    #endif                      
                    iret = 0;

                } else if ((M_WK_CHGPOWER_STEP-1) == workstep)  
                {
                    set_agpio_outstate(s->gpio_base_pin+M_GPIO_LATCH_NUM, M_HIGH_IO);
                    s->workstep = s->workstep - 1;
                    s->time.waketime =  curtime +  timer_from_us(s->PLATCH_AFTER_TIME_us);
                    #if M_DEBUG_INFO_EN
                    //output("fiber:[%u,%u]",M_HIGH_IO,s->PLATCH_AFTER_TIME_us);
                    #endif 
                    iret = 0;
                }else if ((M_WK_CHGPOWER_STEP-2) == workstep)  
                {
                    set_agpio_outstate(s->gpio_base_pin+M_GPIO_LATCH_NUM, M_LOW_IO);
                    s->workstep = s->workstep - 1;
                    if (s->PLATCH_CHG_INTERTIME_us > 0)
                    {
                        s->time.waketime =  curtime +  timer_from_us(s->PLATCH_CHG_INTERTIME_us);
                        #if M_DEBUG_INFO_EN
                        //output("fiber:[%u,%u]",M_LOW_IO,s->PLATCH_CHG_INTERTIME_us);
                        #endif                         
                        iret = 0;
                    }
                    else
                    {
                        s->workstep = 0;
                        s->workflag = s->workflag & (~M_LASER_SETPOWER_FLAG);
                        s->workmode = M_WK_IDLE_MODE;
                        s->time.waketime =  curtime +  timer_from_us(s->STA_MIN_DELAYTIME_us);
                        iret = 1;                        
                    }

                }else if ((M_WK_CHGPOWER_STEP-3) == workstep)  
                {
                    s->workstep = 0;
                    s->workflag = s->workflag & (~M_LASER_SETPOWER_FLAG);
                    s->workmode = M_WK_IDLE_MODE;
                    s->time.waketime =  curtime +  timer_from_us(s->STA_MIN_DELAYTIME_us);
                    iret = 1;  

                }

            }
            else
            {
                s->workmode = M_WK_IDLE_MODE;
                s->time.waketime =  curtime +  timer_from_us(s->STA_MIN_DELAYTIME_us);
                iret = 1;
            }
            break;
        case M_WK_POWEROFF_MODE :
            if (workstep > 0)
            {
                if (M_WK_POWEROFF_STEP == workstep) 
                {   
                    set_agpio_outstate(s->gpio_base_pin+M_GPIO_BSEM_NUM, M_LOW_IO);
                    s->time.waketime =  curtime +  timer_from_us(s->STA_EM_EE_TIME_us);
                    s->workstep = s->workstep - 1;
                    s->workflag = s->workflag &  (~M_LASER_EM_FLAG);
                    iret = 0;

                } else if ((M_WK_POWEROFF_STEP-1) == workstep)  
                {
                    set_agpio_outstate(s->gpio_base_pin+M_GPIO_MOEE_NUM, M_LOW_IO);
                    s->workstep = 0;
                    s->workflag = s->workflag & (~M_LASER_EE_FLAG);
                    s->workmode = M_WK_IDLE_MODE;
                    s->time.waketime =  curtime +  timer_from_us(s->STA_MIN_DELAYTIME_us);
                    iret = 1;
                }

            }
            else
            {
                s->workmode = M_WK_IDLE_MODE;
                s->time.waketime =  curtime +  timer_from_us(s->STA_MIN_DELAYTIME_us);                
                iret = 1;
            }

            break;

        case M_WK_LASERON_MODE:
            set_agpio_outstate(s->gpio_base_pin+M_GPIO_BSEM_NUM, M_HIGH_IO);
            s->workstep = 0;
            s->workflag = s->workflag | M_LASER_EM_FLAG;
            s->workmode = M_WK_IDLE_MODE;
            s->time.waketime =  curtime +  timer_from_us(s->STA_MIN_DELAYTIME_us);
            iret = 1;        
            break; 

        case M_WK_LASEROFF_MODE:
            set_agpio_outstate(s->gpio_base_pin+M_GPIO_BSEM_NUM, M_LOW_IO);
            s->workstep = 0;
            s->workflag = s->workflag &  (~M_LASER_EM_FLAG);
            s->workmode = M_WK_IDLE_MODE;
            s->time.waketime =  curtime +  timer_from_us(s->STA_MIN_DELAYTIME_us);            
            iret =1;
            break;   


        case M_WK_REDLEDON_MODE:
            set_agpio_outstate(s->gpio_base_pin+M_GPIO_POINTERL_NUM, M_HIGH_IO);
            s->workstep = 0;
            s->workflag = s->workflag | M_LASER_RLED_FLAG;
            s->workmode = M_WK_IDLE_MODE;
            s->time.waketime =  curtime +  timer_from_us(s->STA_MIN_DELAYTIME_us);
            iret = 1;        
            break; 

        case M_WK_REDLEDOFF_MODE:
            set_agpio_outstate(s->gpio_base_pin+M_GPIO_POINTERL_NUM, M_LOW_IO);
            s->workstep = 0;
            s->workflag = s->workflag &  (~M_LASER_RLED_FLAG);
            s->workmode = M_WK_IDLE_MODE;
            s->time.waketime =  curtime +  timer_from_us(s->STA_MIN_DELAYTIME_us);            
            iret =1;
            break; 

        case M_WK_ESTOPON_MODE:
            set_agpio_outstate(s->gpio_base_pin+M_GPIO_EMERGENCYOFF_NUM, M_LOW_IO);  //low
            s->workstep = 0;
            s->workflag = s->workflag | M_LASER_STOP_FLAG;
            s->workmode = M_WK_IDLE_MODE;
            s->time.waketime =  curtime +  timer_from_us(s->STA_MIN_DELAYTIME_us);
            iret = 1;        
            break; 

        case M_WK_ESTOPOFF_MODE:
            set_agpio_outstate(s->gpio_base_pin+M_GPIO_EMERGENCYOFF_NUM, M_HIGH_IO);//high
            s->workstep = 0;
            s->workflag = s->workflag &  (~M_LASER_STOP_FLAG);
            s->workmode = M_WK_IDLE_MODE;
            s->time.waketime =  curtime +  timer_from_us(s->STA_MIN_DELAYTIME_us);            
            iret =1;
            break;            

        case M_WK_IDLE_MODE:
            s->workstep = 0;
            s->workmode = M_WK_IDLE_MODE;
            s->time.waketime =  curtime +  timer_from_us(s->STA_MIN_DELAYTIME_us);            
            iret =1;        
            break;

    }
    return(iret);

}
// Setup a stepper for the next move in its queue
static uint_fast8_t
stepper_load_next_fiber(struct stepper_fiber *s)
{
    if (move_queue_empty(&s->mq)) {
        // There is no next move - the queue is empty
        s->workstep = 0;
        return SF_DONE;
    }

    // Load next 'struct stepper_move' into 'struct stepper'
    struct move_node *mn = move_queue_pop(&s->mq);
    struct stepper_move_fiber *m = container_of(mn, struct stepper_move_fiber, node);
    s->workmode =  m->workmode;
    s->workstep =  m->workstep;
    s->workpower =  m->workpower; 
    int iret = handle_fiber_timeseq(s);
    move_free(m);
    if (iret == 0)
    {
        return SF_RESCHEDULE; 
    }
    else
    {
        if(s->STA_MIN_DELAYTIME_us > 0)
        {
            return SF_RESCHEDULE; 
        }
        else
        {
            s->time.waketime =  timer_read_time() +  timer_from_us(M_MIN_DEDAYTIME_US);  
            return SF_RESCHEDULE; 

        }

    }
    //return SF_RESCHEDULE;
}

// Optimized step function to step on each step pin edge
uint_fast8_t
stepper_event_edge_fiber(struct timer *t)
{
    struct stepper_fiber *s = container_of(t, struct stepper_fiber, time);
    uint32_t count = s->workstep;
    if (likely(count)) {

        int iret = handle_fiber_timeseq(s);
        if (iret == 0)
        {
            return SF_RESCHEDULE; 
        }
    }
    return stepper_load_next_fiber(s);

}



// Optimized entry point for step function (may be inlined into sched.c code)
uint_fast8_t
stepper_event_fiber(struct timer *t)
{

    return stepper_event_edge_fiber(t);

}
/*
    uint32_t STA_EE_EM_TIME_us; //5ms = 5000us
    uint32_t PLATCH_BEFORE_TIME_us; //1us =   
    uint32_t PLATCH_AFTER_TIME_us; //2us  
    uint32_t PLATCH_CHG_INTERTIME_us; //4us      
    uint32_t STA_EM_EE_TIME_us; //1us 
    uint32_t STA_MIN_DELAYTIME_us; //1us 

*/



//pin20
//#define  M_GPIO_PRRSYNC_NUM         (11)
//#define M_DEFAULT_POWER_VAL         (0)
#define M_DEFAULT_POWER_VAL           M_MIN_FIBER_POWER_VAL



int fiber_laser_init(struct stepper_fiber *s)
{

    set_manygpio_out(s->gpio_base_pin, M_POWER_IO_TOTAL, M_DEFAULT_POWER_VAL);
    set_agpio_out(s->gpio_base_pin+M_GPIO_LATCH_NUM, 0);
    set_agpio_out(s->gpio_base_pin+M_GPIO_MOEE_NUM, 0);
    set_agpio_out(s->gpio_base_pin+M_GPIO_BSEM_NUM, 0);
    set_agpio_out(s->gpio_base_pin+M_GPIO_POINTERL_NUM, 0);
    set_agpio_out(s->gpio_base_pin+M_GPIO_EMERGENCYOFF_NUM, 1);

    set_agpio_in(s->gpio_base_pin+M_GPIO_IN_ALARM16_NUM);
    set_agpio_in(s->gpio_base_pin+M_GPIO_IN_ALARM21_NUM);    
    //PRR
    setup_pio_pwm(s->gpio_base_pin+M_GPIO_PRRSYNC_NUM, s->period,s->level);

    return(0);

}



void
command_config_stepper_fiber(uint32_t *args)
{
    struct stepper_fiber *s = oid_alloc(args[0], command_config_stepper_fiber, sizeof(*s));

    s->gpio_base_pin = args[1];
    s->STA_EE_EM_TIME_us = args[2];
    s->STA_EM_EE_TIME_us = args[3];
    s->PLATCH_BEFORE_TIME_us = M_MIN_US_SYSTEM;  //1; //10; //1us 
    s->PLATCH_AFTER_TIME_us =  M_MIN_US_SYSTEM;  //2; //2000;  //2us  
    //s->PLATCH_CHG_INTERTIME_us = M_MIN_US_SYSTEM;  //4; //40; //4us 
    s->PLATCH_CHG_INTERTIME_us = 0;
    s->STA_MIN_DELAYTIME_us = 0; //0: nodelay
    s->period  = args[4];
    s->fibertype = args[5];

    #if M_DEBUG_INFO_EN
    output("fiber:[%c,%u,%u,%u,:%u,%u]",args[0],args[1],args[2],args[3],args[4],args[5]);
    #endif 

    s->level = s->period/2;
    move_queue_setup(&s->mq, sizeof(struct stepper_move_fiber));
    s->time.func = stepper_event_edge_fiber;
    fiber_laser_init(s);

}
DECL_COMMAND(command_config_stepper_fiber, "config_stepper_fiber oid=%c start_pin=%c"
             " sta_ee_em=%u sta_em_ee=%u psyncpwm=%u type=%c");

// Return the 'struct stepper' for a given stepper oid
static struct stepper_fiber *
stepper_oid_lookup_fiber(uint8_t oid)
{
    return oid_lookup(oid, command_config_stepper_fiber);
}



 // Schedule a set of steps with a given timing
void
command_queue_step_fiber(uint32_t *args)
{

    uint8_t recoid = args[0];
    uint8_t recmode = 0;
    //uint8_t reconf = 0;
    uint8_t recpower = 0;  
    uint8_t laser_on_off = 0;  

    recmode = args[1];
    recpower = args[2];
    if (recpower > 0)
    {
        recpower = chg_power_val(args[2]);
        laser_on_off = M_LASER_ON_CMD;
    }
    else
    {
        recpower = M_MIN_FIBER_POWER_VAL;
        laser_on_off = M_LASER_OFF_CMD;
        
    }
    handle_rec_command(recoid, recmode, recpower, laser_on_off);

 
}
//DECL_COMMAND(command_queue_step_fiber,
             //"queue_step_fiber oid=%c cmdmod=%c onf=%c pwmv=%hu");
DECL_COMMAND(command_queue_step_fiber,
             "queue_step_fiber oid=%c cmdmod=%c pwmv=%hu");            


int  handle_rec_command(uint8_t foid, uint8_t recmode_in, uint8_t recpower_in, uint8_t  laser_on_off)
{
    int iret = 0;
    struct stepper_fiber *s = stepper_oid_lookup_fiber(foid);
    struct stepper_move_fiber *m = move_alloc();
    uint8_t recmode = 0;
    //uint8_t reconf = 0;
    uint8_t recpower = 0;  
    uint8_t runflag = 0;  

    recmode = recmode_in;
    //reconf = args[2];
    recpower = recpower_in;
    m->workmode =  recmode;
    m->workstep =  M_WK_CHGPOWER_STEP;
    m->workpower = recpower;
    switch(recmode)
    {
        case M_WK_POWERON_MODE : 
            runflag = 1;
            m->workstep =  M_WK_POWERON_STEP;
            if (s->workflag & M_LASER_EE_FLAG)
            {
                runflag = 0;
            }
        break;
        case M_WK_POWEROFF_MODE : 
            runflag = 1;
            m->workstep =  M_WK_POWEROFF_STEP;   
            if (s->workflag & M_LASER_EE_FLAG)
            {
                 runflag = 1;
            }   
            else
            {
                runflag = 0;
            }                  
        break;
        case M_WK_LASERON_MODE : 
            runflag = 1;
        break;
        case M_WK_LASEROFF_MODE : 
            runflag = 1;
        break;  
        case M_WK_REDLEDON_MODE  : 
            runflag = 1;
        break;
        case M_WK_REDLEDOFF_MODE  : 
            runflag = 1;
        break;  
        case M_WK_ESTOPON_MODE : 
            runflag = 1;
        break;
        case M_WK_ESTOPOFF_MODE : 
            runflag = 1;
        break;                  
        case M_WK_CHGPOWER_MODE : 
            runflag = 1;
            m->workstep =  M_WK_CHGPOWER_STEP;
            if(laser_on_off == M_LASER_ON_CMD)
            {
                if ((s->workflag & M_LASER_EM_FLAG) == 0 )
                {
                    set_agpio_outstate(s->gpio_base_pin+M_GPIO_BSEM_NUM, M_HIGH_IO);
                    s->workflag = s->workflag | M_LASER_EM_FLAG;     
                }
           

            }else if (laser_on_off == M_LASER_OFF_CMD)
            {
                if ((s->workflag & M_LASER_EM_FLAG) > 0 )
                {
                    set_agpio_outstate(s->gpio_base_pin+M_GPIO_BSEM_NUM, M_LOW_IO);
                    s->workflag = s->workflag & (~M_LASER_EM_FLAG);
                }
            }
        break;  
        case M_WK_IDLE_MODE : 
             runflag = 0;
        break;                                     
    }  

    if (runflag)
    {
        if ((s->recvmode == recmode ) && (s->recvpower == recpower) && (recmode == M_WK_CHGPOWER_MODE) )
        {
            runflag = 0;
        }
        else if ((s->recvmode == recmode ) && (recmode != M_WK_CHGPOWER_MODE) )
        {
            runflag = 0;
        }
        else 
        {
            s->recvmode = recmode;
            s->recvpower = recpower;

        }
         
    }
    irq_disable();
    if (runflag)
    {
        uint8_t flags = s->flags;
        if (s->workstep) {
            s->flags = flags;
            move_queue_push(&m->node, &s->mq);
        } else if (flags & SF_NEED_RESET) {
            move_free(m);
        } else {
            s->flags = flags;
            move_queue_push(&m->node, &s->mq);
            stepper_load_next_fiber(s);
            sched_add_timer(&s->time);
        }
    }
    else
    {
        move_free(m);
    }
    irq_enable();
    return iret;
    
}





uint8_t  chg_power_val(uint32_t val)
{

    uint8_t  power_val = 0;
    if (val > M_MAX_FIBER_POWER_VAL)
    {
        power_val = M_MAX_FIBER_POWER_VAL;
        return(power_val);
    }
    if (val < M_P20_FIBER_POWER_VAL)
    {
        val = val/2 + M_MIN_FIBER_POWER_VAL;
        power_val = val & 0xFF;

    }
    else
    {
        power_val = val & 0xFF;
    }
    return(power_val);


}



void 
direct_set_pwm_pulse_width_fibertype(uint8_t pwd_oid, uint32_t val)
{
    
    uint8_t recoid = pwd_oid;
    uint8_t recmode = 0;
    //uint8_t reconf = 0;
    uint8_t recpower = 0;  
    recpower = val & 0xFF;
    recmode =  M_WK_CHGPOWER_MODE;
    uint8_t laser_on_off = 0;  
    //todo 

    if (recpower > 0)
    {
        recpower = chg_power_val(val);
        laser_on_off = M_LASER_ON_CMD;
    }
    else
    {
        recpower = M_MIN_FIBER_POWER_VAL;
        laser_on_off = M_LASER_OFF_CMD;
        
    }
    handle_rec_command(recoid, recmode, recpower,laser_on_off);


   
}


// Set an absolute time that the next step will be relative to
void
command_reset_step_clock_fiber(uint32_t *args)
{
    struct stepper_fiber *s = stepper_oid_lookup_fiber(args[0]);
    uint32_t waketime = args[1];
    irq_disable();
    if (s->workstep)
        shutdown("Can't reset time when stepper active");
    s->time.waketime = waketime;
    //s->next_step_time = 
    s->flags &= ~SF_NEED_RESET;
    irq_enable();
}
DECL_COMMAND(command_reset_step_clock_fiber, "reset_step_clock_fiber oid=%c clock=%u");


// Report the current position of the stepper
void
command_stepper_get_position_fiber(uint32_t *args)
{
    uint8_t oid = args[0];
    struct stepper_fiber *s = stepper_oid_lookup_fiber(oid);

    s->workflag  = s->workflag & (~(M_ST_PIN16_BIT+M_ST_PIN21_BIT));
    if(1 == get_agpio_in(s->gpio_base_pin+M_GPIO_IN_ALARM16_NUM))
    {
        s->workflag  = s->workflag | M_ST_PIN16_BIT;
    }
    if(1 == get_agpio_in(s->gpio_base_pin+M_GPIO_IN_ALARM21_NUM)) 
    {
         s->workflag  = s->workflag | M_ST_PIN21_BIT;
    } 
    irq_disable();
    uint32_t val = s->workflag ;
    irq_enable();
    sendf("stepper_fiber oid=%c flag=%i", oid, val);
}
DECL_COMMAND(command_stepper_get_position_fiber, "stepper_get_fiber oid=%c");



// Stop all moves for a given stepper (caller must disable IRQs)
static void
stepper_stop_fiber(struct trsync_signal *tss, uint8_t reason)
{
    struct stepper_fiber *s = container_of(tss, struct stepper_fiber, stop_signal);
    sched_del_timer(&s->time);
    s->time.waketime = 0;
    s->workstep = 0;
    s->flags = (s->flags ) | SF_NEED_RESET;

    while (!move_queue_empty(&s->mq)) {
        struct move_node *mn = move_queue_pop(&s->mq);
        struct stepper_move_fiber *m = container_of(mn, struct stepper_move_fiber, node);
        move_free(m);
    }
}

// Set the stepper to stop on a "trigger event" (used in homing)
void
command_stepper_stop_on_trigger_fiber(uint32_t *args)
{
    struct stepper_fiber *s = stepper_oid_lookup_fiber(args[0]);
    struct trsync *ts = trsync_oid_lookup(args[1]);
    trsync_add_signal(ts, &s->stop_signal, stepper_stop_fiber);
}
DECL_COMMAND(command_stepper_stop_on_trigger_fiber,
             "stepper_stop_on_trigger_fiber oid=%c trsync_oid=%c");




void
stepper_shutdown_fiber(void)
{
    uint8_t i;
    struct stepper_fiber *s;
    foreach_oid(i, s, command_config_stepper_fiber) {
        move_queue_clear(&s->mq);
        stepper_stop_fiber(&s->stop_signal, 0);
    }


}
DECL_SHUTDOWN(stepper_shutdown_fiber);


/*
uint_fast8_t
xy2_100t_event_tm(struct timer *t)
{

    uint32_t curtime = timer_read_time();
    //uint32_t min_next_time = curtime + s->step_pulse_ticks;
    uint32_t min_next_time = curtime + timer_from_us(10000); //10ms
    sched_wake_task(&xy2_100_wake);
	t->waketime = min_next_time;

	
    return SF_RESCHEDULE;


	
}
*/
#endif







