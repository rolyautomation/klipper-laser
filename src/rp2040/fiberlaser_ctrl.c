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
    uint8_t flags;
};


//10s: from power on to normal
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

#define  M_GPIO_IN_ALARM16_NUM       (14)
#define  M_GPIO_IN_ALARM21_NUM       (15)


//Emission Modulation(booster,Laser Modulation), Emission Enable(Master Oscillator)

#define  M_WK_IDLE_MODE         (0)
#define  M_WK_CHGPOWER_MODE     (1)
#define  M_WK_POWERON_MODE      (2)
#define  M_WK_POWEROFF_MODE     (3)
#define  M_WK_LASERONOFF_MODE   (4)


#define  M_WK_CHGPOWER_STEP     (3)
#define  M_WK_POWERON_STEP      (3)
#define  M_WK_POWEROFF_STEP     (3)
#define  M_WK_LASERONOFF_STEP   (3)


struct stepper_fiber {
    struct timer time;
    uint32_t STA_EE_EM_TIME; //5ms
    uint32_t PLATCH_BEFORE_TIME; //1us  
    uint32_t PLATCH_AFTER_TIME; //2us  
    uint32_t PLATCH_CHG_INTERTIME; //4us      
    uint32_t STA_EM_EE_TIME; //1us 
    uint8_t  gpio_base_pin;
    uint8_t  workmode;

    uint32_t  interval;

    struct move_queue_head mq;
    struct trsync_signal stop_signal;
    // gcc (pre v6) does better optimization when uint8_t are bitfields
    uint8_t flags : 8;
};



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
    s->workstep =  m->workstep;

     

    move_free(m);
    return SF_RESCHEDULE;
}

// Optimized step function to step on each step pin edge
uint_fast8_t
stepper_event_edge_fiber(struct timer *t)
{
    struct stepper_fiber *s = container_of(t, struct stepper_fiber, time);
    #if M_TEST_VIR_STEP_GPIO
    gpio_out_toggle_noirq(s->step_pin);
    #endif 
    #ifdef  M_LINK_GALVO_EN
    update_fiber_postion_info(s->step_pin_num, s->position, s->count, 0);  
    #endif      
    uint32_t count = s->count - 1;
    if (likely(count)) {
        s->count = count;
        s->time.waketime += s->interval;
        s->interval += s->add;
        return SF_RESCHEDULE;
    }
    return stepper_load_next_fiber(s);
}

#define AVR_STEP_INSNS 40 // minimum instructions between step gpio pulses

// AVR optimized step function
static uint_fast8_t
stepper_event_avr_fiber(struct timer *t)
{
    struct stepper_fiber *s = container_of(t, struct stepper_fiber, time);
    #if M_TEST_VIR_STEP_GPIO
    gpio_out_toggle_noirq(s->step_pin);
    #endif
    #ifdef  M_LINK_GALVO_EN
    update_fiber_postion_info(s->step_pin_num, s->position, s->count, 0);  
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
    uint_fast8_t ret = stepper_load_next_fiber(s);
    #if M_TEST_VIR_STEP_GPIO    
    gpio_out_toggle_noirq(s->step_pin);
    #endif    
    return ret;
}

// Regular "double scheduled" step function
uint_fast8_t
stepper_event_full_fiber(struct timer *t)
{
    struct stepper_fiber *s = container_of(t, struct stepper_fiber, time);
    #if M_TEST_VIR_STEP_GPIO
    gpio_out_toggle_noirq(s->step_pin);
    #endif
    #ifdef  M_LINK_GALVO_EN
    update_fiber_postion_info(s->step_pin_num, s->position, s->count, M_COUNT_MUL_TWO);  
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
    uint_fast8_t ret = stepper_load_next_fiber(s);
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
stepper_event_fiber(struct timer *t)
{
    if (HAVE_EDGE_OPTIMIZATION)
        return stepper_event_edge_fiber(t);
    if (HAVE_AVR_OPTIMIZATION)
        return stepper_event_avr_fiber(t);
    return stepper_event_full_fiber(t);
}


void
command_config_stepper_fiber(uint32_t *args)
{
    struct stepper_fiber *s = oid_alloc(args[0], command_config_stepper_fiber, sizeof(*s));
    int_fast8_t invert_step = args[3];
    s->flags = invert_step > 0 ? SF_INVERT_STEP : 0;
    s->step_pin = gpio_out_setup(args[1], s->flags & SF_INVERT_STEP);
    #ifdef M_LINK_GALVO_EN
    s->step_pin_num = args[1];
    set_record_axis_info(args[1]);
    #endif
    s->dir_pin = gpio_out_setup(args[2], 0);
    s->position = -POSITION_BIAS;
    s->step_pulse_ticks = args[4];
    move_queue_setup(&s->mq, sizeof(struct stepper_move_fiber));
    if (HAVE_EDGE_OPTIMIZATION) {

        s->time.func = stepper_event_edge_fiber;

        if (!s->step_pulse_ticks && invert_step < 0)
            s->flags |= SF_SINGLE_SCHED;
        else
            s->time.func = stepper_event_full_fiber;
    } else if (HAVE_AVR_OPTIMIZATION) {
        s->time.func = stepper_event_avr_fiber;

        if (s->step_pulse_ticks <= AVR_STEP_INSNS)
            s->flags |= SF_SINGLE_SCHED;
        else
            s->time.func = stepper_event_full_fiber;
    } else if (!CONFIG_INLINE_STEPPER_HACK) {
        s->time.func = stepper_event_full_fiber;
    }
}
DECL_COMMAND(command_config_stepper_fiber, "config_stepper_fiber oid=%c step_pin=%c"
             " dir_pin=%c invert_step=%c step_pulse_ticks=%u");

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
    struct stepper_fiber *s = stepper_oid_lookup_fiber(args[0]);
    struct stepper_move_fiber *m = move_alloc();
    m->interval = args[1];
    m->count = args[2];
    if (!m->count)
        shutdown("Invalid count parameter");
    m->add = args[3];
    m->flags = 0;

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
        stepper_load_next_fiber(s);
        sched_add_timer(&s->time);
    }
    irq_enable();
}
DECL_COMMAND(command_queue_step_fiber,
             "queue_step_fiber oid=%c interval=%u count=%hu add=%hi");

// Set the direction of the next queued step
void
command_set_next_step_dir_fiber(uint32_t *args)
{
    struct stepper_fiber *s = stepper_oid_lookup_fiber(args[0]);
    uint8_t nextdir = args[1] ? SF_NEXT_DIR : 0;
    irq_disable();
    s->flags = (s->flags & ~SF_NEXT_DIR) | nextdir;
    irq_enable();
}
DECL_COMMAND(command_set_next_step_dir_fiber, "set_next_step_dir_fiber oid=%c dir=%c");

// Set an absolute time that the next step will be relative to
void
command_reset_step_clock_fiber(uint32_t *args)
{
    struct stepper_fiber *s = stepper_oid_lookup_fiber(args[0]);
    uint32_t waketime = args[1];
    irq_disable();
    if (s->count)
        shutdown("Can't reset time when stepper active");
    s->next_step_time = s->time.waketime = waketime;
    s->flags &= ~SF_NEED_RESET;
    irq_enable();
}
DECL_COMMAND(command_reset_step_clock_fiber, "reset_step_clock_fiber oid=%c clock=%u");

// Return the current stepper position.  Caller must disable irqs.
static uint32_t
stepper_get_position_fiber(struct stepper_fiber *s)
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
command_stepper_get_position_fiber(uint32_t *args)
{
    uint8_t oid = args[0];
    struct stepper_fiber *s = stepper_oid_lookup_fiber(oid);
    irq_disable();
    uint32_t position = stepper_get_position_fiber(s);
    irq_enable();
    sendf("stepper_position_fiber oid=%c pos=%i", oid, position - POSITION_BIAS);
}
DECL_COMMAND(command_stepper_get_position_fiber, "stepper_get_position_fiber oid=%c");



// Stop all moves for a given stepper (caller must disable IRQs)
static void
stepper_stop_fiber(struct trsync_signal *tss, uint8_t reason)
{
    struct stepper_fiber *s = container_of(tss, struct stepper_fiber, stop_signal);
    sched_del_timer(&s->time);
    s->next_step_time = s->time.waketime = 0;
    s->position = -stepper_get_position_fiber(s);
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







