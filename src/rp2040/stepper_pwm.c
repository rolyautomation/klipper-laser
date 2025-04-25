// Handling of stepper_pwm drivers.
//
// Copyright (C) 2024-2028  jinqiang <jinqiang@ecomedge.io>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy
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
#include "trsync.h" // trsync_add_signal

extern void direct_set_pwm_pulse_width(uint8_t pwd_oid, uint32_t val);
void  set_pwm_pulse_width(uint8_t flag,uint8_t pwd_oid, uint32_t val);
extern void  direct_set_pwm_pulse_width_fibertype(uint8_t pwd_oid, uint32_t val, uint8_t  pwm_on_off);
void  set_pwm_pulse_width_fiberlaser(uint8_t flag,uint8_t pwd_oid, uint32_t val, uint8_t  pwm_on_off);


#if CONFIG_INLINE_STEPPER_HACK && CONFIG_HAVE_STEPPER_BOTH_EDGE
 #define HAVE_SINGLE_SCHEDULE 1
 #define HAVE_EDGE_OPTIMIZATION 1
 #define HAVE_AVR_OPTIMIZATION 0
 DECL_CONSTANT("STEPPER_BOTH_EDGE", 1);
#elif CONFIG_INLINE_STEPPER_HACK && CONFIG_MACH_AVR
 #define HAVE_SINGLE_SCHEDULE 1
 #define HAVE_EDGE_OPTIMIZATION 0
 #define HAVE_AVR_OPTIMIZATION 1
#else
 #define HAVE_SINGLE_SCHEDULE 0
 #define HAVE_EDGE_OPTIMIZATION 0
 #define HAVE_AVR_OPTIMIZATION 0
#endif

//#define  M_TEST_VIR_STEP_GPIO   (0)
#define  M_TEST_VIR_STEP_GPIO   (1)

//#define  M_FIFO_ISR_WORK_EN     (1)
//#define  M_LINK_GALVO_EN        (1)
#define  M_COUNT_MUL_TWO          (1)
#define  M_PWM_OUT_EN             (1)

//#define  M_OUTINFO_EN             (1)  //1
#define  M_OUTINFO_EN             (0)  //0

struct stepper_move_pwm {
    struct move_node node;
    uint32_t interval;
    int16_t add;
    uint16_t count;
    #ifdef M_PWM_OUT_EN    
    uint8_t  pwm_on_off;    
    uint8_t  mode;
    uint16_t pwmval;
    uint32_t speed_pulse_ticks;  
    uint8_t  composite_itemlen;
    uint8_t  composite_fifo_index;
    uint32_t composite_dcount;  
    uint32_t composite_count_offset;    
    uint8_t  composite_mode;
    #endif       
    uint8_t flags;
};

enum { MF_DIR=1<<0 };

struct stepper_pwm {
    struct timer time;
    uint32_t interval;
    int16_t add;
    uint32_t count;
    uint32_t next_step_time, step_pulse_ticks;
    struct gpio_out step_pin, dir_pin;
    #ifdef M_LINK_GALVO_EN
    uint8_t step_pin_num;
    #endif
    #ifdef M_PWM_OUT_EN
    uint8_t  pwm_on_off;
    uint8_t  mode;
    uint16_t pwmval;
    uint32_t speed_pulse_ticks;
    uint8_t  composite_itemlen;
    uint8_t  composite_fifo_index;
    uint32_t composite_dcount;  
    uint32_t composite_count_offset;    
    uint8_t  composite_mode;
    uint8_t  oid_pwm;    //must pwm stepper on same mcu 
    uint8_t  oid_pwm_flag;
    uint8_t  laser_type; 
    uint8_t  pauseresume_sw;
    uint8_t  min_power_value;
    #endif
    uint32_t position;
    struct move_queue_head mq;
    struct trsync_signal stop_signal;
    // gcc (pre v6) does better optimization when uint8_t are bitfields
    uint8_t flags : 8;
};

enum { POSITION_BIAS=0x40000000 };

enum {
    SF_LAST_DIR=1<<0, SF_NEXT_DIR=1<<1, SF_INVERT_STEP=1<<2, SF_NEED_RESET=1<<3,
    SF_SINGLE_SCHED=1<<4, SF_HAVE_ADD=1<<5
};


struct pwm_ctrl_s_t {
    uint32_t interval;
    int16_t  add;
    uint16_t count;

    uint8_t  pwm_on_off;    
    uint8_t  mode;
    uint16_t pwmval;
    uint32_t speed_pulse_ticks;  

    
    uint8_t  oid_pwm;    //must pwm stepper on same mcu 
    uint8_t  oid_pwm_flag;  
    uint8_t  laser_type; 
    uint8_t  min_power_value;

    //uint32_t cur_pwm_val;
    uint32_t last_pwm_val;

    uint8_t  pauseresume_sw;


    uint8_t  composite_fifo_index;
    uint8_t  composite_itemlen; 
    uint8_t  composite_cindex_run;
    uint8_t  composite_cpower_run;  
    uint32_t composite_sum_run;  
    uint8_t  *p_power_table;       
    uint32_t composite_dcount;  
    uint32_t composite_count_offset;    
    uint8_t  composite_mode;  
    uint32_t composite_cindex_dcount_run; 
    uint32_t composite_chgb_dcount_run; 


};


uint32_t sts_speed_pulse_ticks;  
uint32_t sts_speed_pulse_ticks_max; 


typedef struct pwm_ctrl_s_t pwm_ctrl_s_t;


#define M_PROCESS_PWM_IDLE   (0)
#define M_PROCESS_PWM_START  (1)
#define M_PROCESS_PWM_RUN    (2)
#define M_PROCESS_PWM_NCHG   (3)
#define M_PROCESS_PWM_NOCMD  (4)


#define M_MIN_POWER_INIT     (1)



#define PWM_MODE_IDLE   (0)
//#Continuous
#define PWM_MODE_M3     (1)
//#Dynamic
#define PWM_MODE_M4     (2)
#define PWM_MODE_CLS    (3)

pwm_ctrl_s_t  g_pwm_ctrl_data;

#define  M_FIFO_DATA_LEN  (64)
#define  M_FIFO_NUM_MAX   (3)
#define  M_BASE_RATIO     (128)
#define  M_BASE_BIT       (7)


struct fifo_buff_s_t {
    uint8_t  buff[M_FIFO_NUM_MAX][M_FIFO_DATA_LEN]; 
    uint8_t  head;
    uint8_t  tail;
};


typedef enum {
    FIFO_SUCCESS = 0,
    FIFO_ERR_INVALID_LEN = -1,
    FIFO_ERR_FULL = -2,
    FIFO_ERR_EMPTY = -3,
    FIFO_ERR_INVALID_INDEX = -4
} fifo_error_t;



typedef struct fifo_buff_s_t fifo_buff_s_t;
fifo_buff_s_t  g_fifo_buff_data;


static inline uint8_t is_fifo_empty(void) {
    return g_fifo_buff_data.head == g_fifo_buff_data.tail;
}

static inline uint8_t is_fifo_full(void) {
    return ((g_fifo_buff_data.head + 1) % M_FIFO_NUM_MAX) == g_fifo_buff_data.tail;
}

static inline uint8_t get_fifo_used_count(void) {
    return (g_fifo_buff_data.head + M_FIFO_NUM_MAX - g_fifo_buff_data.tail) % M_FIFO_NUM_MAX;
}

void init_fifo_buff_compmode(void)
{
    g_fifo_buff_data.head = 0;
    g_fifo_buff_data.tail = 0;
}

int8_t push_fifo_buff_compmode(uint8_t *data, uint8_t len)
{
    int8_t iret = 0;
    if(data == NULL ||len == 0 || len > M_FIFO_DATA_LEN)
        return FIFO_ERR_INVALID_LEN;    
    if((g_fifo_buff_data.head + 1) % M_FIFO_NUM_MAX == g_fifo_buff_data.tail)
        return FIFO_ERR_FULL;
    memcpy(g_fifo_buff_data.buff[g_fifo_buff_data.head], data, len);
    iret = g_fifo_buff_data.head;
    g_fifo_buff_data.head = (g_fifo_buff_data.head + 1) % M_FIFO_NUM_MAX;
    return iret;
}



int  pop_fifo_buff_compmode(uint8_t curindex)
{
    if(g_fifo_buff_data.head == g_fifo_buff_data.tail)
        return FIFO_ERR_EMPTY;
    if(curindex != g_fifo_buff_data.tail)
        return FIFO_ERR_INVALID_INDEX;
    g_fifo_buff_data.tail = (g_fifo_buff_data.tail + 1) % M_FIFO_NUM_MAX;
    return FIFO_SUCCESS;

}

uint8_t * get_fifo_buff_compmode(uint8_t curindex)
{
    if(g_fifo_buff_data.head == g_fifo_buff_data.tail)
        return NULL;
    if(curindex != g_fifo_buff_data.tail)
        return NULL;
    return g_fifo_buff_data.buff[g_fifo_buff_data.tail];

}



void  set_pwm_ctrl_data(uint8_t oid_pwm_flag, uint8_t  oid_pwm, uint8_t  laser_type)
{
    if(oid_pwm_flag > 0)
        g_pwm_ctrl_data.last_pwm_val = 0;   
    g_pwm_ctrl_data.oid_pwm_flag = oid_pwm_flag;
    g_pwm_ctrl_data.oid_pwm  = oid_pwm;
    g_pwm_ctrl_data.laser_type = laser_type;

}

void set_pwm_pause_resume_flag(uint8_t pauseresume_sw)
{
    g_pwm_ctrl_data.pauseresume_sw =  pauseresume_sw;

}


void set_pwm_min_power_value(uint8_t min_power_value)
{
    g_pwm_ctrl_data.min_power_value = min_power_value;

}

void  load_next_pwm_ctrl_data(uint32_t interval, int16_t  add, uint16_t count, uint8_t  mode, uint8_t  pwm_on_off, uint16_t pwmval, uint32_t speed_pulse_ticks)
{
      
    g_pwm_ctrl_data.interval = interval;
    g_pwm_ctrl_data.add = add;
    g_pwm_ctrl_data.count = count;
    g_pwm_ctrl_data.mode = mode;
    g_pwm_ctrl_data.pwm_on_off = pwm_on_off;    
    g_pwm_ctrl_data.pwmval = pwmval;
    g_pwm_ctrl_data.speed_pulse_ticks = speed_pulse_ticks;     


}

void  update_composite_chgb_dcount(void)
{

    if (g_pwm_ctrl_data.composite_cindex_run+1 < g_pwm_ctrl_data.composite_itemlen)
    {

        //g_pwm_ctrl_data.composite_chgb_dcount_run = (uint32_t)(((uint64_t)g_pwm_ctrl_data.composite_sum_run * g_pwm_ctrl_data.composite_dcount) / M_BASE_RATIO);
        g_pwm_ctrl_data.composite_chgb_dcount_run = (uint32_t)(((uint64_t)g_pwm_ctrl_data.composite_sum_run * g_pwm_ctrl_data.composite_dcount) >> M_BASE_BIT);
    }
    else
    {
        g_pwm_ctrl_data.composite_chgb_dcount_run = g_pwm_ctrl_data.composite_dcount;
        //last item
        pop_fifo_buff_compmode(g_pwm_ctrl_data.composite_fifo_index); 
        #if M_OUTINFO_EN
        output("popfifo:[%c]",g_pwm_ctrl_data.composite_fifo_index); 
        #endif        

    }

}


static inline void  update_power_pwm_value(uint16_t pwmval)
{

    g_pwm_ctrl_data.pwmval = pwmval;

}

void  load_next_pwm_ctrl_data_composite(uint8_t composite_mode, uint8_t composite_fifo_index, uint8_t composite_itemlen, uint32_t composite_count_offset, uint32_t composite_dcount)
{

    g_pwm_ctrl_data.composite_mode = composite_mode;
    if (g_pwm_ctrl_data.composite_mode > 0)
    {
        g_pwm_ctrl_data.composite_fifo_index = composite_fifo_index;
        g_pwm_ctrl_data.composite_itemlen = composite_itemlen; 
        g_pwm_ctrl_data.composite_dcount = composite_dcount;  
        g_pwm_ctrl_data.composite_count_offset = composite_count_offset;
        if (composite_count_offset == 0)  
        {

            g_pwm_ctrl_data.p_power_table = get_fifo_buff_compmode(composite_fifo_index);
            if (g_pwm_ctrl_data.p_power_table != NULL)
            {
                g_pwm_ctrl_data.composite_cindex_run = 0;
                g_pwm_ctrl_data.composite_sum_run = g_pwm_ctrl_data.p_power_table[0];
                g_pwm_ctrl_data.composite_cpower_run = g_pwm_ctrl_data.p_power_table[1];
                g_pwm_ctrl_data.composite_cindex_dcount_run = 0;
                //g_pwm_ctrl_data.composite_chgb_dcount_run
                update_composite_chgb_dcount();
                update_power_pwm_value(g_pwm_ctrl_data.composite_cpower_run);

            }
            else
            {
                g_pwm_ctrl_data.composite_cindex_run = 0;
                g_pwm_ctrl_data.composite_sum_run = 0;
                g_pwm_ctrl_data.composite_cpower_run = 0;
                g_pwm_ctrl_data.composite_cindex_dcount_run = 0;
                g_pwm_ctrl_data.composite_chgb_dcount_run = g_pwm_ctrl_data.composite_dcount;
            }


        }
        else
        {
            #if M_OUTINFO_EN
            output("correction:[%u,%u]",g_pwm_ctrl_data.composite_count_offset,g_pwm_ctrl_data.composite_cindex_dcount_run); 
            #endif             
            g_pwm_ctrl_data.composite_cindex_dcount_run = g_pwm_ctrl_data.composite_count_offset;
            update_power_pwm_value(g_pwm_ctrl_data.composite_cpower_run);

        }
 

    } 
 

}


void  update_composite_cindex_dcount(void)
{
    if (g_pwm_ctrl_data.composite_mode > 0)
    {
        g_pwm_ctrl_data.composite_cindex_dcount_run++;
        if (g_pwm_ctrl_data.composite_cindex_dcount_run  >  g_pwm_ctrl_data.composite_chgb_dcount_run)
        {
            g_pwm_ctrl_data.composite_cindex_run++;
            if (g_pwm_ctrl_data.composite_cindex_run < g_pwm_ctrl_data.composite_itemlen)
            {
                //uint8_t index  = g_pwm_ctrl_data.composite_cindex_run << 1;
                uint8_t index  = g_pwm_ctrl_data.composite_cindex_run*2;
                g_pwm_ctrl_data.composite_sum_run += g_pwm_ctrl_data.p_power_table[index];
                g_pwm_ctrl_data.composite_cpower_run = g_pwm_ctrl_data.p_power_table[index + 1];
                update_composite_chgb_dcount();
                update_power_pwm_value(g_pwm_ctrl_data.composite_cpower_run);
                #if M_OUTINFO_EN
                output("next item:[%c]",g_pwm_ctrl_data.composite_cindex_run); 
                output("value:[%u:%u,%c]",g_pwm_ctrl_data.composite_cindex_dcount_run,g_pwm_ctrl_data.composite_chgb_dcount_run,g_pwm_ctrl_data.composite_cpower_run);                 
                #endif                  
                // if (g_pwm_ctrl_data.composite_cindex_run+1 >= g_pwm_ctrl_data.composite_itemlen)
                // {   
                //     //last item
                //     pop_fifo_buff_compmode(g_pwm_ctrl_data.composite_fifo_index);

                // }

            }
            else
            {
                g_pwm_ctrl_data.composite_mode = 0;
                #if M_OUTINFO_EN
                output("end composite mode"); 
                #endif              


            }


        }

    }


}


//#define M_MIN_PULSE_TICKS   (10)
#define M_MIN_PULSE_TICKS       (4)
#define M_MIN_POWER_VALUE_PER   (0.1)
//0%
//#define M_POWER_VAL_LOWERLIMIT   (0)
//10%
#define M_POWER_VAL_LOWERLIMIT (26)
//15%
//#define M_POWER_VAL_LOWERLIMIT  (38)



uint32_t cacl_power_var_value(uint32_t inter_pulse_ticks)
{

    uint32_t result_value = 0;
    uint32_t speed_pulse = g_pwm_ctrl_data.speed_pulse_ticks;
    uint32_t pwmval = g_pwm_ctrl_data.pwmval;    
    
    if (inter_pulse_ticks == 0)
    {
        return(result_value);
    } 
      
    if (speed_pulse >= inter_pulse_ticks) {
        result_value = pwmval;
    } else {
        result_value = (uint32_t)(((uint64_t)pwmval * speed_pulse) / inter_pulse_ticks);
    }
    //min power 255*0.1=25.5
    /*
    if((result_value < M_POWER_VAL_LOWERLIMIT) && (g_pwm_ctrl_data.pwmval > 0))
    {
        result_value = M_POWER_VAL_LOWERLIMIT;     
    }
    */
    if((result_value < g_pwm_ctrl_data.min_power_value) && (g_pwm_ctrl_data.pwmval > 0))
    {
        result_value = g_pwm_ctrl_data.min_power_value;     
    }   
    return(result_value);
            
}


//sts
void report_speed_stauts_ontest(void)
{
     output("speed_value:[d%u:%u,%u]",sts_speed_pulse_ticks,sts_speed_pulse_ticks_max,g_pwm_ctrl_data.speed_pulse_ticks); 
}


void update_next_pwm_ctrl_data(uint8_t runstep, uint16_t count, uint32_t inter_pulse_ticks)
{
    uint32_t cur_pwm_val=0;
    uint8_t  flag = 0;
    uint8_t  pwm_on_off_transfer = 0; 


    // //pause_resume 
    // pwm_on_off_transfer = g_pwm_ctrl_data.pwm_on_off;
    // if (g_pwm_ctrl_data.pauseresume_sw > 0 )
    // {
    //     cur_pwm_val = 0;
    //     flag = 1;
    //     pwm_on_off_transfer = 0;
    //     goto  runpwmout;

    // }

    // if (g_pwm_ctrl_data.pwm_on_off == 0)
    // {
    //     cur_pwm_val = 0;
    //     flag = 1;
    //     goto  runpwmout;
    // }
    //turn on
    if (g_pwm_ctrl_data.count <= 1)
    {
        ;//if (inter_pulse_ticks)

    }
    switch(runstep)
    {
        case M_PROCESS_PWM_START:
             update_composite_cindex_dcount();
             if (g_pwm_ctrl_data.mode == PWM_MODE_M3)
             {
                cur_pwm_val = g_pwm_ctrl_data.pwmval;
                flag = 1;
             } 
             else if (g_pwm_ctrl_data.mode == PWM_MODE_M4)
             {
                cur_pwm_val = cacl_power_var_value(inter_pulse_ticks);
                flag = 1;

             } 
             else if (g_pwm_ctrl_data.mode == PWM_MODE_CLS)
             {
                if ( g_pwm_ctrl_data.last_pwm_val == 0)
                     g_pwm_ctrl_data.last_pwm_val= 1;  //force run one
                cur_pwm_val = 0;
                flag = 1;
             }                          
            break;
        case M_PROCESS_PWM_RUN:
             update_composite_cindex_dcount();
             if (g_pwm_ctrl_data.mode == PWM_MODE_M3)
             {
                cur_pwm_val = g_pwm_ctrl_data.pwmval;
                flag = 1;
             } 
             else if (g_pwm_ctrl_data.mode == PWM_MODE_M4)
             {

                cur_pwm_val = cacl_power_var_value(inter_pulse_ticks);
                flag = 1;

             } 
             else if (g_pwm_ctrl_data.mode == PWM_MODE_CLS)
             {
                //g_pwm_ctrl_data.last_pwm_val = 1;  //force run one
                cur_pwm_val = 0;
                flag = 1;
             }           
            break;   
        case M_PROCESS_PWM_NCHG:
             if (g_pwm_ctrl_data.mode == PWM_MODE_M3)
             {
                flag = 0;
             } 
             else if (g_pwm_ctrl_data.mode == PWM_MODE_M4)
             {
                 flag = 0;
             } 
             else if (g_pwm_ctrl_data.mode == PWM_MODE_CLS)
             {

                cur_pwm_val = 0;
                flag = 0;
             }          
            break;
        case M_PROCESS_PWM_NOCMD:
             if (g_pwm_ctrl_data.mode == PWM_MODE_M3)
             {
 
                    cur_pwm_val = 0;
                    flag = 1;
                       

             } 
             else if (g_pwm_ctrl_data.mode == PWM_MODE_M4)
             {

                    cur_pwm_val = 0;
                    flag = 1;
             } 
             else if (g_pwm_ctrl_data.mode == PWM_MODE_CLS)
             {
                //g_pwm_ctrl_data.last_pwm_val = 1;  //force run one
                cur_pwm_val = 0;
                flag = 1;

             }          
            break;                      
        default :
            flag = 0;

    }

    //pause_resume 
    pwm_on_off_transfer = g_pwm_ctrl_data.pwm_on_off;
    if (g_pwm_ctrl_data.pauseresume_sw > 0 )
    {
        cur_pwm_val = 0;
        flag = 1;
        pwm_on_off_transfer = 0;
        goto  runpwmout;

    }

    if (g_pwm_ctrl_data.pwm_on_off == 0)
    {
        cur_pwm_val = 0;
        flag = 1;
        //goto  runpwmout;
    }

runpwmout:    
    if ( (flag) && (cur_pwm_val != g_pwm_ctrl_data.last_pwm_val))
    {
        g_pwm_ctrl_data.last_pwm_val = cur_pwm_val;
        if (g_pwm_ctrl_data.laser_type == 0)
        {
            set_pwm_pulse_width(g_pwm_ctrl_data.oid_pwm_flag ,g_pwm_ctrl_data.oid_pwm, cur_pwm_val);
        }
        else
        {
            //set_pwm_pulse_width_fiberlaser(g_pwm_ctrl_data.oid_pwm_flag ,g_pwm_ctrl_data.oid_pwm, cur_pwm_val, g_pwm_ctrl_data.pwm_on_off);
            set_pwm_pulse_width_fiberlaser(g_pwm_ctrl_data.oid_pwm_flag ,g_pwm_ctrl_data.oid_pwm, cur_pwm_val, pwm_on_off_transfer);
            
        }
        
    }



}

// Setup a stepper for the next move in its queue
static uint_fast8_t
stepper_load_next_pwm(struct stepper_pwm *s)
{
    if (move_queue_empty(&s->mq)) {
        // There is no next move - the queue is empty
     
        s->count = 0;
        #ifdef  M_LINK_GALVO_EN
        update_pwm_postion_info(s->step_pin_num, s->position, s->count, 0);  
        #endif  
        #ifdef M_PWM_OUT_EN
        //set_pwm_pulse_width(s->oid_pwm_flag,s->oid_pwm, 0);
        update_next_pwm_ctrl_data(M_PROCESS_PWM_NOCMD, 0, 0);
        #endif               
        return SF_DONE;
    }

    // Load next 'struct stepper_move' into 'struct stepper'
    struct move_node *mn = move_queue_pop(&s->mq);
    struct stepper_move_pwm *m = container_of(mn, struct stepper_move_pwm, node);
    s->add = m->add;
    s->interval = m->interval + m->add;
    if (HAVE_SINGLE_SCHEDULE && s->flags & SF_SINGLE_SCHED) {
        s->time.waketime += m->interval;
        if (HAVE_AVR_OPTIMIZATION)
            s->flags = m->add ? s->flags|SF_HAVE_ADD : s->flags & ~SF_HAVE_ADD;
        s->count = m->count;
    } else {
        // It is necessary to schedule unstep events and so there are
        // twice as many events.
        s->next_step_time += m->interval;
        s->time.waketime = s->next_step_time;
        s->count = (uint32_t)m->count * 2;
    }
    // Add all steps to s->position (stepper_get_position() can calc mid-move)
    if (m->flags & MF_DIR) {
        s->position = -s->position + m->count;
        #if M_TEST_VIR_STEP_GPIO
        gpio_out_toggle_noirq(s->dir_pin);
        #endif
    } else {
        s->position += m->count;
    }


    #ifdef  M_LINK_GALVO_EN
    if (HAVE_SINGLE_SCHEDULE && s->flags & SF_SINGLE_SCHED) {
        update_pwm_postion_info(s->step_pin_num, s->position, s->count, 0);  
    } else {
        update_pwm_postion_info(s->step_pin_num, s->position, s->count, M_COUNT_MUL_TWO);   
    }
    #endif   
    #ifdef M_PWM_OUT_EN
    //set_pwm_pulse_width(s->oid_pwm_flag,s->oid_pwm, 50);
    load_next_pwm_ctrl_data(m->interval, s->add, s->count, m->mode, m->pwm_on_off, m->pwmval, m->speed_pulse_ticks);
    load_next_pwm_ctrl_data_composite(m->composite_mode, m->composite_fifo_index, m->composite_itemlen, m->composite_count_offset, m->composite_dcount);
    update_next_pwm_ctrl_data(M_PROCESS_PWM_START, s->count, m->interval);   //+s->add
    #endif      

    move_free(m);
    return SF_RESCHEDULE;
}

// Optimized step function to step on each step pin edge
uint_fast8_t
stepper_event_edge_pwm(struct timer *t)
{
    struct stepper_pwm *s = container_of(t, struct stepper_pwm, time);
    #if M_TEST_VIR_STEP_GPIO
    gpio_out_toggle_noirq(s->step_pin);
    #endif 
    #ifdef  M_LINK_GALVO_EN
    update_pwm_postion_info(s->step_pin_num, s->position, s->count, 0);  
    #endif  
    #ifdef M_PWM_OUT_EN
    //set_pwm_pulse_width(s->oid_pwm_flag,s->oid_pwm, 50);
    //update_next_pwm_ctrl_data(M_PROCESS_PWM_RUN, s->count-1); 
    update_next_pwm_ctrl_data(M_PROCESS_PWM_RUN, s->count, s->interval); //+s->add
    #endif
    uint32_t count = s->count - 1;
    if (likely(count)) {
        s->count = count;
        s->time.waketime += s->interval;
        s->interval += s->add;
        return SF_RESCHEDULE;
    }
    return stepper_load_next_pwm(s);
}

#define AVR_STEP_INSNS 40 // minimum instructions between step gpio pulses

// AVR optimized step function
static uint_fast8_t
stepper_event_avr_pwm(struct timer *t)
{
    struct stepper_pwm *s = container_of(t, struct stepper_pwm, time);
    #if M_TEST_VIR_STEP_GPIO
    gpio_out_toggle_noirq(s->step_pin);
    #endif
    #ifdef  M_LINK_GALVO_EN
    update_pwm_postion_info(s->step_pin_num, s->position, s->count, 0);  
    #endif  
    #ifdef M_PWM_OUT_EN
    //set_pwm_pulse_width(s->oid_pwm_flag,s->oid_pwm, 50);
    update_next_pwm_ctrl_data(M_PROCESS_PWM_RUN, s->count, s->interval); 
    #endif         
    uint16_t *pcount = (void*)&s->count, count = *pcount - 1;
    if (likely(count)) {
        *pcount = count;
        s->time.waketime += s->interval;
        #if M_TEST_VIR_STEP_GPIO
        gpio_out_toggle_noirq(s->step_pin);
        #endif
        if (s->flags & SF_HAVE_ADD)
            s->interval += s->add;
        return SF_RESCHEDULE;
    }
    uint_fast8_t ret = stepper_load_next_pwm(s);
    #if M_TEST_VIR_STEP_GPIO    
    gpio_out_toggle_noirq(s->step_pin);
    #endif    
    return ret;
}

// Regular "double scheduled" step function
uint_fast8_t
stepper_event_full_pwm(struct timer *t)
{
    struct stepper_pwm *s = container_of(t, struct stepper_pwm, time);
    #if M_TEST_VIR_STEP_GPIO
    gpio_out_toggle_noirq(s->step_pin);
    #endif
    #ifdef  M_LINK_GALVO_EN
    update_pwm_postion_info(s->step_pin_num, s->position, s->count, M_COUNT_MUL_TWO);  
    #endif  
    #ifdef M_PWM_OUT_EN
    //set_pwm_pulse_width(s->oid_pwm_flag,s->oid_pwm, 50);
    update_next_pwm_ctrl_data(M_PROCESS_PWM_RUN, s->count, s->interval); 
    #endif         
    uint32_t curtime = timer_read_time();
    uint32_t min_next_time = curtime + s->step_pulse_ticks;
    s->count--;
    if (likely(s->count & 1))
        // Schedule unstep event
        goto reschedule_min;
    if (likely(s->count)) {
        s->next_step_time += s->interval;
        s->interval += s->add;
        if (unlikely(timer_is_before(s->next_step_time, min_next_time)))
            // The next step event is too close - push it back
            goto reschedule_min;
        s->time.waketime = s->next_step_time;
        return SF_RESCHEDULE;
    }
    uint_fast8_t ret = stepper_load_next_pwm(s);
    if (ret == SF_DONE || !timer_is_before(s->time.waketime, min_next_time))
        return ret;
    // Next step event is too close to the last unstep
    int32_t diff = s->time.waketime - min_next_time;
    if (diff < (int32_t)-timer_from_us(1000))
        shutdown("Stepper too far in past");
reschedule_min:
    s->time.waketime = min_next_time;
    return SF_RESCHEDULE;
}

// Optimized entry point for step function (may be inlined into sched.c code)
uint_fast8_t
stepper_event_pwm(struct timer *t)
{
    if (HAVE_EDGE_OPTIMIZATION)
        return stepper_event_edge_pwm(t);
    if (HAVE_AVR_OPTIMIZATION)
        return stepper_event_avr_pwm(t);
    return stepper_event_full_pwm(t);
}


void
command_config_stepper_pwm(uint32_t *args)
{
    struct stepper_pwm *s = oid_alloc(args[0], command_config_stepper_pwm, sizeof(*s));
    int_fast8_t invert_step = args[3];
    s->flags = invert_step > 0 ? SF_INVERT_STEP : 0;
    s->step_pin = gpio_out_setup(args[1], s->flags & SF_INVERT_STEP);
    #ifdef M_LINK_GALVO_EN
    s->step_pin_num = args[1];
    set_record_axis_info(args[1]);
    #endif
    #ifdef M_PWM_OUT_EN 
    set_pwm_ctrl_data(0, 0, 0);
    set_pwm_pause_resume_flag(0);
    set_pwm_min_power_value(0);
    init_fifo_buff_compmode();
    #endif
    s->dir_pin = gpio_out_setup(args[2], 0);
    s->position = -POSITION_BIAS;
    s->step_pulse_ticks = args[4];
    move_queue_setup(&s->mq, sizeof(struct stepper_move_pwm));
    if (HAVE_EDGE_OPTIMIZATION) {

        s->time.func = stepper_event_edge_pwm;

        if (!s->step_pulse_ticks && invert_step < 0)
            s->flags |= SF_SINGLE_SCHED;
        else
            s->time.func = stepper_event_full_pwm;
    } else if (HAVE_AVR_OPTIMIZATION) {
        s->time.func = stepper_event_avr_pwm;

        if (s->step_pulse_ticks <= AVR_STEP_INSNS)
            s->flags |= SF_SINGLE_SCHED;
        else
            s->time.func = stepper_event_full_pwm;
    } else if (!CONFIG_INLINE_STEPPER_HACK) {
        s->time.func = stepper_event_full_pwm;
    }
    //sts
    sts_speed_pulse_ticks = 0;
    sts_speed_pulse_ticks_max = 0;
}
DECL_COMMAND(command_config_stepper_pwm, "config_stepper_pwm oid=%c step_pin=%c"
             " dir_pin=%c invert_step=%c step_pulse_ticks=%u");

// Return the 'struct stepper' for a given stepper oid
static struct stepper_pwm *
stepper_oid_lookup_pwm(uint8_t oid)
{
    return oid_lookup(oid, command_config_stepper_pwm);
}

// Schedule a set of steps with a given timing
void
command_queue_step_pwm(uint32_t *args)
{
    struct stepper_pwm *s = stepper_oid_lookup_pwm(args[0]);
    struct stepper_move_pwm *m = move_alloc();
    m->interval = args[1];
    m->count = args[2];
    if (!m->count)
        shutdown("Invalid count parameter");
    m->add = args[3];
    m->flags = 0;
#ifdef M_PWM_OUT_EN 
     m->pwm_on_off = s->pwm_on_off;
     m->mode = s->mode;
     m->pwmval = s->pwmval;
     m->speed_pulse_ticks = s->speed_pulse_ticks;
     m->composite_mode = s->composite_mode;
     if (s->composite_mode > 0) {
         m->composite_itemlen = s->composite_itemlen;
         m->composite_dcount = s->composite_dcount;
         m->composite_count_offset = s->composite_count_offset;
         m->composite_fifo_index = s->composite_fifo_index;
         s->composite_count_offset = s->composite_count_offset + m->count;
         if (s->composite_count_offset >= s->composite_dcount)
         {
            s->composite_mode = 0;

         }
     }
    #if M_OUTINFO_EN
    output("val:[%c,%u,%hu,%hi,:%c,%c,%u,%u]",args[0],args[1],args[2],args[3],m->pwm_on_off,m->mode,m->pwmval,m->speed_pulse_ticks); 
    output("coff:[%c,%c,%c,%u,%u]",s->composite_mode,s->composite_fifo_index,s->composite_itemlen,s->composite_dcount,s->composite_count_offset); 
    #endif
#endif
    irq_disable();
    uint8_t flags = s->flags;
    if (!!(flags & SF_LAST_DIR) != !!(flags & SF_NEXT_DIR)) {
        flags ^= SF_LAST_DIR;
        m->flags |= MF_DIR;
    }
    if (s->count) {
        s->flags = flags;
        move_queue_push(&m->node, &s->mq);
    } else if (flags & SF_NEED_RESET) {
        move_free(m);
    } else {
        s->flags = flags;
        move_queue_push(&m->node, &s->mq);
        stepper_load_next_pwm(s);
        sched_add_timer(&s->time);
    }
    irq_enable();
}


#ifdef M_PWM_OUT_EN 

DECL_COMMAND(command_queue_step_pwm,
             "queue_step_pwm oid=%c interval=%u count=%hu add=%hi");
//DECL_COMMAND(command_queue_step_pwm,
//            "queue_step_pwm oid=%c interval=%u count=%hu add=%hi pm=%c p1v=%u p2v=%u");   
#else
DECL_COMMAND(command_queue_step_pwm,
             "queue_step_pwm oid=%c interval=%u count=%hu add=%hi");
#endif

// Set the direction of the next queued step
void
command_set_next_step_dir_pwm(uint32_t *args)
{
    struct stepper_pwm *s = stepper_oid_lookup_pwm(args[0]);
    uint8_t nextdir = args[1] ? SF_NEXT_DIR : 0;
    irq_disable();
    s->flags = (s->flags & ~SF_NEXT_DIR) | nextdir;
    irq_enable();
}
DECL_COMMAND(command_set_next_step_dir_pwm, "set_next_step_dir_pwm oid=%c dir=%c");


#ifdef M_PWM_OUT_EN

void
command_bind_pwm_oid_stepper_pwm(uint32_t *args)
{
    struct stepper_pwm *s = stepper_oid_lookup_pwm(args[0]);
    irq_disable();
    s->oid_pwm = args[1];
    s->oid_pwm_flag  = 1;
    s->laser_type = args[2];
    irq_enable();
    //#ifdef M_PWM_OUT_EN 
    set_pwm_ctrl_data(s->oid_pwm_flag, s->oid_pwm, s->laser_type);
    //#endif

}
//DECL_COMMAND(command_bind_pwm_oid_stepper_pwm, "bind_oid_pwm oid=%c pwmoid=%c");
DECL_COMMAND(command_bind_pwm_oid_stepper_pwm, "bind_oid_pwm oid=%c pwmoid=%c ltype=%c");


void
command_pauseresume_oid_stepper_pwm(uint32_t *args)
{
    struct stepper_pwm *s = stepper_oid_lookup_pwm(args[0]);

    irq_disable();
    s->pauseresume_sw = args[1];
    //irq_disable();
    //s->oid_pwm = args[1];
    //s->oid_pwm_flag  = 1;
    //s->laser_type = args[2];
    irq_enable();
    //#ifdef M_PWM_OUT_EN 
    set_pwm_pause_resume_flag(s->pauseresume_sw);
    //#endif

}
DECL_COMMAND(command_pauseresume_oid_stepper_pwm, "pauseresume_pwm oid=%c sw=%c");


void
command_setminpower_oid_stepper_pwm(uint32_t *args)
{
    struct stepper_pwm *s = stepper_oid_lookup_pwm(args[0]);
    //uint8_t min_power_value =  0;

    //min_power_value = args[1];
    irq_disable();
    s->min_power_value = args[1];
    irq_enable();
    //#ifdef M_PWM_OUT_EN 
    set_pwm_min_power_value(s->min_power_value);
    //output("testsetminp:[%u]",g_pwm_ctrl_data.min_power_value); 
    
    //#endif

}
DECL_COMMAND(command_setminpower_oid_stepper_pwm, "setminpower oid=%c pv=%c");


void
command_set_pwm_sw_stepper_pwm(uint32_t *args)
{
    struct stepper_pwm *s = stepper_oid_lookup_pwm(args[0]);
    irq_disable();
    s->pwm_on_off = args[1];
    irq_enable();

}
DECL_COMMAND(command_set_pwm_sw_stepper_pwm, "set_pwm_onf oid=%c onf=%c");


void
command_set_pwm_modepower_stepper_pwm(uint32_t *args)
{
    struct stepper_pwm *s = stepper_oid_lookup_pwm(args[0]);
    irq_disable();
    //s->composite_mode = 0;      
    s->mode = args[1];
    s->pwmval = args[2];
    s->speed_pulse_ticks = args[3];        
    irq_enable();

}
DECL_COMMAND(command_set_pwm_modepower_stepper_pwm, "set_pwm_power oid=%c mod=%c pwmv=%hu pticks=%u");


void
command_set_pwmpower_lbandwidth_stepper_pwm(uint32_t *args)
{
    struct stepper_pwm *s = stepper_oid_lookup_pwm(args[0]);
    irq_disable();
    //s->composite_mode = 0;  
    s->pwmval = args[1];
    irq_enable();

}
DECL_COMMAND(command_set_pwmpower_lbandwidth_stepper_pwm, "set_pwmpower_lbandwidth oid=%c pwmval=%c");


void
command_set_pticks_lbandwidth_stepper_pwm(uint32_t *args)
{
    struct stepper_pwm *s = stepper_oid_lookup_pwm(args[0]);
    irq_disable();
    s->speed_pulse_ticks = args[1];        
    irq_enable();

}
DECL_COMMAND(command_set_pticks_lbandwidth_stepper_pwm, "set_pticks_lbandwidth oid=%c pticks=%u");


void
command_powerfunc_table_stepper_pwm(uint32_t *args)
{
    struct stepper_pwm *s = stepper_oid_lookup_pwm(args[0]);
    irq_disable();
    s->composite_mode = 1;    
    s->composite_dcount = args[1];
    s->composite_count_offset = 0;
    uint8_t data_len = args[2];
    uint8_t *data = command_decode_ptr(args[3]); 
    #if M_OUTINFO_EN
    output("powerf:[%c,%u,%c]",args[0],args[1],args[2]); 
    output("powerval:[%c,%c]",data[0],data[data_len-1]); 
    #endif    
    irq_enable();
    int8_t iret = push_fifo_buff_compmode(data, data_len);
    if(iret < 0)
    {
        if (iret == FIFO_ERR_FULL)
        {
            shutdown("FIFO buffer overflow");
        }
        else
        {
            shutdown("FIFO buffer invalid length");
        }
    }
    else
    {
        if (data_len & 0x01)
            shutdown("Invalid length, must be even");        
        s->composite_itemlen = data_len >> 1 ; //data_len/2
        s->composite_fifo_index = iret;
    }    

}
DECL_COMMAND(command_powerfunc_table_stepper_pwm, "set_powerfunc_table oid=%c tdc=%u data=%*s");


void
command_powerfunc_speed_table_stepper_pwm(uint32_t *args)
{
    struct stepper_pwm *s = stepper_oid_lookup_pwm(args[0]);
    irq_disable();
    s->composite_mode = 1;
    s->speed_pulse_ticks = args[1];
    s->composite_dcount = args[2];
    s->composite_count_offset = 0;
    s->composite_fifo_index = 0;
    uint8_t data_len = args[3];
    uint8_t *data = command_decode_ptr(args[4]); 
    irq_enable();
    int8_t iret = push_fifo_buff_compmode(data, data_len);
    if(iret < 0)
    {
        if (iret == FIFO_ERR_FULL)
        {
            shutdown("FIFO buffer overflow");
        }
        else
        {
            shutdown("FIFO buffer invalid length");
        }
    }
    else
    {
        if (data_len & 0x01)
            shutdown("Invalid length, must be even");
        s->composite_itemlen = data_len >> 1 ; //data_len/2
        s->composite_fifo_index = iret;
    }

}
DECL_COMMAND(command_powerfunc_speed_table_stepper_pwm, "set_powerfunc_speed_table oid=%c pticks=%u tdc=%u data=%*s");


void 
set_pwm_pulse_width(uint8_t flag,uint8_t pwd_oid, uint32_t val)
{
    if (flag)
    {
        direct_set_pwm_pulse_width(pwd_oid, val);

    }

}


void 
set_pwm_pulse_width_fiberlaser(uint8_t flag,uint8_t pwd_oid, uint32_t val, uint8_t  pwm_on_off)
{
    if (flag)
    {
        direct_set_pwm_pulse_width_fibertype(pwd_oid, val, pwm_on_off);

    }

}


#endif



// Set an absolute time that the next step will be relative to
void
command_reset_step_clock_pwm(uint32_t *args)
{
    struct stepper_pwm *s = stepper_oid_lookup_pwm(args[0]);
    uint32_t waketime = args[1];
    irq_disable();
    if (s->count)
        shutdown("Can't reset time when stepper active");
    s->next_step_time = s->time.waketime = waketime;
    s->flags &= ~SF_NEED_RESET;
    irq_enable();
}
DECL_COMMAND(command_reset_step_clock_pwm, "reset_step_clock_pwm oid=%c clock=%u");

// Return the current stepper position.  Caller must disable irqs.
static uint32_t
stepper_get_position_pwm(struct stepper_pwm *s)
{
    uint32_t position = s->position;
    // If stepper is mid-move, subtract out steps not yet taken
    if (HAVE_SINGLE_SCHEDULE && s->flags & SF_SINGLE_SCHED)
        position -= s->count;
    else
        position -= s->count / 2;
    // The top bit of s->position is an optimized reverse direction flag
    if (position & 0x80000000)
        return -position;
    return position;
}

// Report the current position of the stepper
void
command_stepper_get_position_pwm(uint32_t *args)
{
    uint8_t oid = args[0];
    struct stepper_pwm *s = stepper_oid_lookup_pwm(oid);
    irq_disable();
    uint32_t position = stepper_get_position_pwm(s);
    irq_enable();
    sendf("stepper_position_pwm oid=%c pos=%i", oid, position - POSITION_BIAS);
}
DECL_COMMAND(command_stepper_get_position_pwm, "stepper_get_position_pwm oid=%c");

// Stop all moves for a given stepper (caller must disable IRQs)
static void
stepper_stop_pwm(struct trsync_signal *tss, uint8_t reason)
{
    struct stepper_pwm *s = container_of(tss, struct stepper_pwm, stop_signal);
    sched_del_timer(&s->time);
    s->next_step_time = s->time.waketime = 0;
    s->position = -stepper_get_position_pwm(s);
    s->count = 0;
    s->flags = (s->flags & (SF_INVERT_STEP|SF_SINGLE_SCHED)) | SF_NEED_RESET;
    #if M_TEST_VIR_STEP_GPIO
    gpio_out_write(s->dir_pin, 0);
    #endif
    if (!(HAVE_EDGE_OPTIMIZATION && s->flags & SF_SINGLE_SCHED))
        #if M_TEST_VIR_STEP_GPIO   
        gpio_out_write(s->step_pin, s->flags & SF_INVERT_STEP);
        #else
        ;
        #endif
    while (!move_queue_empty(&s->mq)) {
        struct move_node *mn = move_queue_pop(&s->mq);
        struct stepper_move_pwm *m = container_of(mn, struct stepper_move_pwm, node);
        move_free(m);
    }
}

// Set the stepper to stop on a "trigger event" (used in homing)
void
command_stepper_stop_on_trigger_pwm(uint32_t *args)
{
    struct stepper_pwm *s = stepper_oid_lookup_pwm(args[0]);
    struct trsync *ts = trsync_oid_lookup(args[1]);
    trsync_add_signal(ts, &s->stop_signal, stepper_stop_pwm);
}
DECL_COMMAND(command_stepper_stop_on_trigger_pwm,
             "stepper_stop_on_trigger_pwm oid=%c trsync_oid=%c");





void
stepper_shutdown_pwm(void)
{
    uint8_t i;
    struct stepper_pwm *s;
    foreach_oid(i, s, command_config_stepper_pwm) {
        move_queue_clear(&s->mq);
        stepper_stop_pwm(&s->stop_signal, 0);
    }

}
DECL_SHUTDOWN(stepper_shutdown_pwm);



