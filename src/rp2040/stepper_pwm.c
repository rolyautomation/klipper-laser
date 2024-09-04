// Handling of stepper_pwm drivers.
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
#include "trsync.h" // trsync_add_signal



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

#define  M_FIFO_ISR_WORK_EN     (1)
#define  M_LINK_GALVO_EN        (1)
#define  M_COUNT_MUL_TWO        (1)


struct stepper_move_vir {
    struct move_node node;
    uint32_t interval;
    int16_t add;
    uint16_t count;
    uint8_t flags;
};

enum { MF_DIR=1<<0 };

struct stepper_vir {
    struct timer time;
    uint32_t interval;
    int16_t add;
    uint32_t count;
    uint32_t next_step_time, step_pulse_ticks;
    struct gpio_out step_pin, dir_pin;
    uint8_t step_pin_num;
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

// Setup a stepper for the next move in its queue
static uint_fast8_t
stepper_load_next_vir(struct stepper_vir *s)
{
    if (move_queue_empty(&s->mq)) {
        // There is no next move - the queue is empty
     
        s->count = 0;
        #ifdef  M_LINK_GALVO_EN
        update_vir_postion_info(s->step_pin_num, s->position, s->count, 0);  
        #endif         
        return SF_DONE;
    }

    // Load next 'struct stepper_move' into 'struct stepper'
    struct move_node *mn = move_queue_pop(&s->mq);
    struct stepper_move_vir *m = container_of(mn, struct stepper_move_vir, node);
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
        update_vir_postion_info(s->step_pin_num, s->position, s->count, 0);  
    } else {
        update_vir_postion_info(s->step_pin_num, s->position, s->count, M_COUNT_MUL_TWO);   
    }
    #endif     

    move_free(m);
    return SF_RESCHEDULE;
}

// Optimized step function to step on each step pin edge
uint_fast8_t
stepper_event_edge_vir(struct timer *t)
{
    struct stepper_vir *s = container_of(t, struct stepper_vir, time);
    #if M_TEST_VIR_STEP_GPIO
    gpio_out_toggle_noirq(s->step_pin);
    #endif 
    #ifdef  M_LINK_GALVO_EN
    update_vir_postion_info(s->step_pin_num, s->position, s->count, 0);  
    #endif      
    uint32_t count = s->count - 1;
    if (likely(count)) {
        s->count = count;
        s->time.waketime += s->interval;
        s->interval += s->add;
        return SF_RESCHEDULE;
    }
    return stepper_load_next_vir(s);
}

#define AVR_STEP_INSNS 40 // minimum instructions between step gpio pulses

// AVR optimized step function
static uint_fast8_t
stepper_event_avr_vir(struct timer *t)
{
    struct stepper_vir *s = container_of(t, struct stepper_vir, time);
    #if M_TEST_VIR_STEP_GPIO
    gpio_out_toggle_noirq(s->step_pin);
    #endif
    #ifdef  M_LINK_GALVO_EN
    update_vir_postion_info(s->step_pin_num, s->position, s->count, 0);  
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
    uint_fast8_t ret = stepper_load_next_vir(s);
    #if M_TEST_VIR_STEP_GPIO    
    gpio_out_toggle_noirq(s->step_pin);
    #endif    
    return ret;
}

// Regular "double scheduled" step function
uint_fast8_t
stepper_event_full_vir(struct timer *t)
{
    struct stepper_vir *s = container_of(t, struct stepper_vir, time);
    #if M_TEST_VIR_STEP_GPIO
    gpio_out_toggle_noirq(s->step_pin);
    #endif
    #ifdef  M_LINK_GALVO_EN
    update_vir_postion_info(s->step_pin_num, s->position, s->count, M_COUNT_MUL_TWO);  
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
    uint_fast8_t ret = stepper_load_next_vir(s);
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
stepper_event_vir(struct timer *t)
{
    if (HAVE_EDGE_OPTIMIZATION)
        return stepper_event_edge_vir(t);
    if (HAVE_AVR_OPTIMIZATION)
        return stepper_event_avr_vir(t);
    return stepper_event_full_vir(t);
}


void
command_config_stepper_vir(uint32_t *args)
{
    struct stepper_vir *s = oid_alloc(args[0], command_config_stepper_vir, sizeof(*s));
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
    move_queue_setup(&s->mq, sizeof(struct stepper_move_vir));
    if (HAVE_EDGE_OPTIMIZATION) {

        s->time.func = stepper_event_edge_vir;

        if (!s->step_pulse_ticks && invert_step < 0)
            s->flags |= SF_SINGLE_SCHED;
        else
            s->time.func = stepper_event_full_vir;
    } else if (HAVE_AVR_OPTIMIZATION) {
        s->time.func = stepper_event_avr_vir;

        if (s->step_pulse_ticks <= AVR_STEP_INSNS)
            s->flags |= SF_SINGLE_SCHED;
        else
            s->time.func = stepper_event_full_vir;
    } else if (!CONFIG_INLINE_STEPPER_HACK) {
        s->time.func = stepper_event_full_vir;
    }
}
DECL_COMMAND(command_config_stepper_vir, "config_stepper_vir oid=%c step_pin=%c"
             " dir_pin=%c invert_step=%c step_pulse_ticks=%u");

// Return the 'struct stepper' for a given stepper oid
static struct stepper_vir *
stepper_oid_lookup_vir(uint8_t oid)
{
    return oid_lookup(oid, command_config_stepper_vir);
}

// Schedule a set of steps with a given timing
void
command_queue_step_vir(uint32_t *args)
{
    struct stepper_vir *s = stepper_oid_lookup_vir(args[0]);
    struct stepper_move_vir *m = move_alloc();
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
        stepper_load_next_vir(s);
        sched_add_timer(&s->time);
    }
    irq_enable();
}
DECL_COMMAND(command_queue_step_vir,
             "queue_step_vir oid=%c interval=%u count=%hu add=%hi");

// Set the direction of the next queued step
void
command_set_next_step_dir_vir(uint32_t *args)
{
    struct stepper_vir *s = stepper_oid_lookup_vir(args[0]);
    uint8_t nextdir = args[1] ? SF_NEXT_DIR : 0;
    irq_disable();
    s->flags = (s->flags & ~SF_NEXT_DIR) | nextdir;
    irq_enable();
}
DECL_COMMAND(command_set_next_step_dir_vir, "set_next_step_dir_vir oid=%c dir=%c");

// Set an absolute time that the next step will be relative to
void
command_reset_step_clock_vir(uint32_t *args)
{
    struct stepper_vir *s = stepper_oid_lookup_vir(args[0]);
    uint32_t waketime = args[1];
    irq_disable();
    if (s->count)
        shutdown("Can't reset time when stepper active");
    s->next_step_time = s->time.waketime = waketime;
    s->flags &= ~SF_NEED_RESET;
    irq_enable();
}
DECL_COMMAND(command_reset_step_clock_vir, "reset_step_clock_vir oid=%c clock=%u");

// Return the current stepper position.  Caller must disable irqs.
static uint32_t
stepper_get_position_vir(struct stepper_vir *s)
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
command_stepper_get_position_vir(uint32_t *args)
{
    uint8_t oid = args[0];
    struct stepper_vir *s = stepper_oid_lookup_vir(oid);
    irq_disable();
    uint32_t position = stepper_get_position_vir(s);
    irq_enable();
    sendf("stepper_position_vir oid=%c pos=%i", oid, position - POSITION_BIAS);
}
DECL_COMMAND(command_stepper_get_position_vir, "stepper_get_position_vir oid=%c");

// Stop all moves for a given stepper (caller must disable IRQs)
static void
stepper_stop_vir(struct trsync_signal *tss, uint8_t reason)
{
    struct stepper_vir *s = container_of(tss, struct stepper_vir, stop_signal);
    sched_del_timer(&s->time);
    s->next_step_time = s->time.waketime = 0;
    s->position = -stepper_get_position_vir(s);
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
        struct stepper_move_vir *m = container_of(mn, struct stepper_move_vir, node);
        move_free(m);
    }
}

// Set the stepper to stop on a "trigger event" (used in homing)
void
command_stepper_stop_on_trigger_vir(uint32_t *args)
{
    struct stepper_vir *s = stepper_oid_lookup_vir(args[0]);
    struct trsync *ts = trsync_oid_lookup(args[1]);
    trsync_add_signal(ts, &s->stop_signal, stepper_stop_vir);
}
DECL_COMMAND(command_stepper_stop_on_trigger_vir,
             "stepper_stop_on_trigger_vir oid=%c trsync_oid=%c");




void stepper_init_position_value(void)
{

    uint8_t i;
    struct stepper_vir *s;
    foreach_oid(i, s, command_config_stepper_vir) {
        s->position = -POSITION_BIAS;
        //test is ok
        //shutdown("reset position value in test");
    }


}


void
stepper_shutdown_vir(void)
{
    uint8_t i;
    struct stepper_vir *s;
    foreach_oid(i, s, command_config_stepper_vir) {
        move_queue_clear(&s->mq);
        stepper_stop_vir(&s->stop_signal, 0);
    }
    stepper_init_position_value();

}
DECL_SHUTDOWN(stepper_shutdown_vir);



