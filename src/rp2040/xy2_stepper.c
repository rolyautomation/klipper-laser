// Handling of xy2 stepper drivers.
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
#include "xy2_stepper.h"  // 
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


static struct task_wake xy2_100_wake;

#if 0


struct xy2_stepper_move {
    struct move_node node;
    uint32_t interval;
	uint32_t timecount;
    uint16_t xy2_x_rto;    //new position
    uint16_t xy2_y_rto;	   //new position

};


struct xy2_stepper {
    struct timer time;
	uint32_t interval;
    int32_t addx;
    int32_t addy;	
    uint32_t runcount;
    uint16_t xy2_x_rto;    //new position
    uint16_t xy2_y_rto;	   //new position
    uint16_t xy2_x_rc;   //
    uint16_t xy2_y_rc;	
    uint16_t clk_pin_no, xbase_pin_no;
	uint32_t next_step_time;
    uint16_t xy2_x_pos;
    uint16_t xy2_y_pos;	//record x,y position
    struct move_queue_head mq;
	struct trsync_signal stop_signal;	
	
};



int32_t diff_approx_val(int32_t to_pos, int32_t from_pos, int32_t speedv)
{
	int32_t  diff_val = 0;
    diff_val = to_pos -  from_pos;
	diff_val = diff_val*1.0/speedv;
	return(diff_val);

}

uint16_t next_approx_val(int32_t to_pos, int32_t from_pos, int32_t diffv)
{

	uint16_t ret_val = to_pos;
	int32_t  pos_temp = 0;
	if (to_pos != from_pos)
	{
	    pos_temp = from_pos + diffv;
		if (diffv > 0)
		{
		   if (pos_temp > to_pos)
		   {
		      pos_temp = to_pos;
		   }

		}
		else
		{
		   if (pos_temp < to_pos)
		   {
			 pos_temp = to_pos;
		   }

		}
		ret_val = pos_temp;
		
	}
    return(ret_val);	
	
}



// Setup a stepper for the next move in its queue
static uint_fast8_t
xy2_stepper_load_next(struct xy2_stepper *s)
{
    if (move_queue_empty(&s->mq)) {
        // There is no next move - the queue is empty
        s->runcount = 0;
        return SF_DONE;
    }

    // Load next 'struct stepper_move' into 'struct stepper'
    struct move_node *mn = move_queue_pop(&s->mq);
    struct xy2_stepper_move *m = container_of(mn, struct xy2_stepper_move, node);


    s->interval = m->interval;
	s->xy2_x_rto = m->xy2_x_rto;
	s->xy2_y_rto = m->xy2_y_rto;
	s->runcount = m->timecount / m->interval;
	if (s->runcount == 0)
	{
	   s->runcount = 1;
	}
	s->addx = diff_approx_val(m->xy2_x_rto, s->xy2_x_pos, s->runcount);  //m->xy2_x_rto - s->xy2_x_pos;
	s->addy = diff_approx_val(m->xy2_y_rto, s->xy2_y_pos, s->runcount);  //m->xy2_y_rto - s->xy2_y_pos;

	s->xy2_x_rc = next_approx_val(m->xy2_x_rto, s->xy2_x_pos, s->addx);
    s->xy2_y_rc = next_approx_val(m->xy2_y_rto, s->xy2_y_pos, s->addy);
    //  only ready data but dont  write buffer
    
    s->next_step_time += m->interval;
    s->time.waketime = s->next_step_time;	
	

    move_free(m);
    return SF_RESCHEDULE;
}



// Regular "double scheduled" step function
uint_fast8_t
xy2_stepper_event(struct timer *t)
{
    struct xy2_stepper *s = container_of(t, struct xy2_stepper, time);
    //gpio_out_toggle_noirq(s->step_pin);
    uint32_t curtime = timer_read_time();
    //uint32_t min_next_time = curtime + s->step_pulse_ticks;
    uint32_t min_next_time = curtime

    //exist bug e
    //struct move_node *mn = move_queue_pop(&s->mq);
    //struct xy2_stepper_move *m = container_of(mn, struct xy2_stepper_move, node);	
    
    s->runcount--;
    if (likely(s->runcount)) {
		//s->next_step_time += m->interval;
         s->next_step_time += s->interval;
        //s->interval += s->add;

	    /*
        if (unlikely(timer_is_before(s->next_step_time, min_next_time)))
            // The next step event is too close - push it back
            goto reschedule_min;
        */    
		
        s->time.waketime = s->next_step_time;

        s->xy2_x_pos = s->xy2_x_rc;
	    s->xy2_y_pos = s->xy2_y_rc;	
		// write buffer
		send_xy_data(s->xy2_x_pos, s->xy2_y_pos, 0);

		
		s->xy2_x_rc = next_approx_val(s->xy2_x_rto, s->xy2_x_rc, s->addx);
	    s->xy2_y_rc = next_approx_val(s->xy2_y_rto, s->xy2_y_rc, s->addy);
        return SF_RESCHEDULE;
    }
    uint_fast8_t ret = xy2_stepper_load_next(s);
    //if (ret == SF_DONE || !timer_is_before(s->time.waketime, min_next_time))
    if (ret == SF_DONE)
        return ret;
/*	
    // Next step event is too close to the last unstep
    int32_t diff = s->time.waketime - min_next_time;
    if (diff < (int32_t)-timer_from_us(1000))
        shutdown("Stepper too far in past");
reschedule_min:
    s->time.waketime = min_next_time;
*/	
    return SF_RESCHEDULE;
}

#endif



#ifdef  M_FIFO_ISR_WORK_EN


typedef uint32_t ring_data_t;
typedef uint16_t hdata_t;
typedef uint8_t  ring_len_t;
//typedef size_t   ring_len_t;

struct ring_buffer_t {
  /** Buffer memory.  X,Y value */
  ring_data_t *buffer;   
  /** Buffer mask. */
  ring_len_t buffer_mask;
  /** Index of tail. */
  ring_len_t tail_index;
  /** Index of head. */
  ring_len_t head_index;
};

typedef struct ring_buffer_t ring_buffer_t;

struct mvirxy_sync_pio_t {
    uint32_t position;
    uint8_t  gpiono;
    uint8_t  flag;
};

typedef struct mvirxy_sync_pio_t mvirxy_sync_pio_t;



#define   M_XY2_P1      (0)
#define   M_XY2_P2      (1)
#define   M_XY2_RESTART (2)

struct ring_pio_t {

  ring_buffer_t   ring_data;
  uint8_t      cur_index;
  uint32_t     last_send_ar[2];
  mvirxy_sync_pio_t  * p_sync_pio;
  uint32_t     coord_factor;
  uint8_t      mirror_type;
};


#define  M_BUF_SIZE_LEN     (32)
//#define  M_BUF_SIZE_LEN   (64)
#define  RING_BUFFER_MASK(rb) (rb->buffer_mask)

void ring_buffer_init(ring_buffer_t *buffer, ring_data_t *buf, size_t buf_size) {
  buffer->buffer = buf;
  buffer->buffer_mask = buf_size - 1;
  buffer->tail_index = 0;
  buffer->head_index = 0;

}

struct ring_pio_t   * gp_ring_pio_mang = NULL;
mvirxy_sync_pio_t     g_sync_pio_data[2] = { {.flag=0,},{.flag=0,}};



int  set_record_axis_info(uint8_t  gpio_step_num)
{
     int iret = 0;
     int i = 0;
     uint8_t   temp;

     for(i=0;i < 2; i++)
     {
        if (g_sync_pio_data[i].flag == 0)
        {
            g_sync_pio_data[i].flag = 1;
            g_sync_pio_data[i].gpiono = gpio_step_num;
            //g_sync_pio_data[i].position = -POSITION_BIAS;
            g_sync_pio_data[i].position = 0;
            break;
        }

     }
     if ((g_sync_pio_data[0].flag) && (g_sync_pio_data[1].flag))
     {
         if (g_sync_pio_data[0].gpiono  >  g_sync_pio_data[1].gpiono) 
         {
            temp = g_sync_pio_data[0].gpiono;
            g_sync_pio_data[0].gpiono = g_sync_pio_data[1].gpiono;
            g_sync_pio_data[1].gpiono = temp;
            
         }  
                
     }
     return(iret);

 
}


static uint32_t
stepper_get_position_vir_funxy(uint32_t position_in, uint16_t count,uint8_t mode)
{
    uint32_t position = position_in;
    // If stepper is mid-move, subtract out steps not yet taken
    //if (HAVE_SINGLE_SCHEDULE && s->flags & SF_SINGLE_SCHED)
    if (mode == 0)
        position -= count;
    else
        position -= count / 2;
    // The top bit of s->position is an optimized reverse direction flag
    if (position & 0x80000000)
        position  = -position;

    position = position - POSITION_BIAS;
    return position;

}



int  update_vir_postion_info(uint8_t  gpio_step_num, uint32_t position, uint16_t count, uint8_t mode)
{

     int iret = 0;
     int i = 0;
     uint32_t posX_32 = 0;
     uint32_t posY_32 = 0;     
     uint16_t posX = 0;
     uint16_t posY = 0;
     int todoflag = 1;
     uint32_t realposition = 0; 
     unsigned char  bitv = gp_ring_pio_mang->coord_factor;
     uint8_t mirror_type = gp_ring_pio_mang->mirror_type;
     for(i=0; i < 2; i++)
     {

        if (g_sync_pio_data[i].gpiono == gpio_step_num)
        {
            //g_sync_pio_data[i].position = position;
            realposition = stepper_get_position_vir_funxy(position,count,mode);
            if (g_sync_pio_data[i].position == realposition)
            {
                todoflag = 0;
            }
            g_sync_pio_data[i].position = realposition;
            break;
        }


     }

     if (todoflag == 0)
     {
        iret = 1;
        return(iret);
     }

     //very important to G28 bug
     if (g_sync_pio_data[0].position  & 0x80000000)
     {
         g_sync_pio_data[0].position = -g_sync_pio_data[0].position;
     }

     if (g_sync_pio_data[1].position & 0x80000000)
     {
         g_sync_pio_data[1].position = -g_sync_pio_data[1].position;
     }
     
         
     //todo after
     #if 0
     posX = g_sync_pio_data[0].position & 0xFFFF;
     posY = g_sync_pio_data[1].position & 0xFFFF;    
     #endif 
     posX_32 = g_sync_pio_data[0].position << bitv ;
     posY_32 = g_sync_pio_data[1].position << bitv ;  
     posX = posX_32  & 0xFFFF;
     posY = posY_32  & 0xFFFF;     
     if (  (posX_32 >> 16)   > 0)
            posX = 0xFFFF;
     if (  (posY_32 >> 16)   > 0)
            posY = 0xFFFF;    
     // mirror transfer
     if (mirror_type == 1)  
     {
            posX =  posX^0xFFFF;

     } else if (mirror_type == 2)  
     {
            posY =  posY^0xFFFF;

     } else if (mirror_type == 3)  
     {
            posX =  posX^0xFFFF;
            posY =  posY^0xFFFF;            
     }

     upadte_new_onedata(posX, posY);
     return(iret);


          
}


/**
 * Returns whether a ring buffer is empty.
 * @param buffer The buffer for which it should be returned whether it is empty.
 * @return 1 if empty; 0 otherwise.
 */

inline uint8_t ring_buffer_is_empty(ring_buffer_t *buffer) {
  return (buffer->head_index == buffer->tail_index);
}

/**
 * Returns whether a ring buffer is full.
 * @param buffer The buffer for which it should be returned whether it is full.
 * @return 1 if full; 0 otherwise.
 */
inline uint8_t ring_buffer_is_full(ring_buffer_t *buffer) {
  return ((buffer->head_index - buffer->tail_index) & RING_BUFFER_MASK(buffer)) == RING_BUFFER_MASK(buffer);
}


uint8_t ring_buffer_queue(ring_buffer_t *buffer, hdata_t datax, hdata_t datay) {
  /* Is buffer full? */
  if(ring_buffer_is_full(buffer)) {
    /* Is going to overwrite the oldest byte */
    /* Increase tail index */
    //buffer->tail_index = ((buffer->tail_index + 1) & RING_BUFFER_MASK(buffer));
    //buff is small, or  send too slow
    return 0;
  }

  /* Place data in buffer */
  buffer->buffer[buffer->head_index] =  ( datay << 16 ) | datax;
  buffer->head_index = ((buffer->head_index + 1) & RING_BUFFER_MASK(buffer));
  return 1;

}


/**
 * Returns the oldest byte in a ring buffer.
 * @param buffer The buffer from which the data should be returned.
 * @param data A pointer to the location at which the data should be placed.
 * @return 1 if data was returned; 0 otherwise.
 */
uint8_t ring_buffer_dequeue(ring_buffer_t *buffer, hdata_t * pdatax, hdata_t * pdatay) {
  if(ring_buffer_is_empty(buffer)) {
    /* No items */
    return 0;
  }
  
  *pdatax = buffer->buffer[buffer->tail_index] & 0xFFFF;
  *pdatay = (buffer->buffer[buffer->tail_index] >> 16 ) & 0xFFFF;  
  buffer->tail_index = ((buffer->tail_index + 1) & RING_BUFFER_MASK(buffer));
  return 1;
}






void
xy2_100_pio_irq_handler(struct ring_pio_t * xy2_ring_pio)
{
    uint8_t  t_ring_st = 0;
    hdata_t  datax = 0; 
    hdata_t  datay = 0; 

    if (xy2_ring_pio == NULL)
        return;
    
    //if (pio_fifo_status() == 0)  
    while(pio_fifo_status() == 0)
    {
        
       if ( xy2_ring_pio->cur_index >  M_XY2_P2 )
       {
            xy2_ring_pio->cur_index = 0;
            //update data
            t_ring_st = ring_buffer_dequeue(&xy2_ring_pio->ring_data, &datax, &datay);
            if (t_ring_st)
            {
                comb_senddata_format(datax, datay, &(xy2_ring_pio->last_send_ar[0]),  &(xy2_ring_pio->last_send_ar[1]));

            }
            

       }
       pio_put_onedata(xy2_ring_pio->last_send_ar[xy2_ring_pio->cur_index]);
       xy2_ring_pio->cur_index += 1;
       
    }  
       
}

void
PIO0_IRQHandler_xy2(void)
{
    xy2_100_pio_irq_handler(gp_ring_pio_mang);

}

uint8_t upadte_new_onedata(uint16_t posX, uint16_t posY)
{
       uint8_t  iret = 0;
       iret = ring_buffer_queue(&(gp_ring_pio_mang->ring_data), posX, posY);
       return(iret);
       
}


uint8_t upadte_new_onedata_first(uint16_t posX, uint16_t posY)
{
       uint8_t  iret = 0;
       struct ring_pio_t * xy2_ring_pio = gp_ring_pio_mang;
       iret = ring_buffer_queue(&(gp_ring_pio_mang->ring_data), posX, posY);
       xy2_ring_pio->cur_index = M_XY2_RESTART;
       return(iret);
       
}

void  clean_gvar_toinit(void)
{
    g_sync_pio_data[0].flag = 0;
    g_sync_pio_data[1].flag = 0;

}


void  open_pio_isr(void)
{


      armcm_enable_irq(PIO0_IRQHandler_xy2, PIO0_IRQ_0_IRQn, 7);  //time  2；
      open_pio_isr_reg();




}


void  close_pio_isr(void)
{

     close_pio_isr_reg();
     clean_gvar_toinit();
    
}

#endif


struct xy2_stepper {
    struct timer time;
	uint32_t interval;
    uint16_t clk_pin_no, xbase_pin_no;
	uint32_t next_step_time;
    uint16_t xy2_x_pos;
    uint16_t xy2_y_pos;	//record x,y position

};



uint_fast8_t
xy2_stepper_event_tm(struct timer *t)
{
    struct xy2_stepper *s = container_of(t, struct xy2_stepper, time);
    //gpio_out_toggle_noirq(s->step_pin);
    uint32_t curtime = timer_read_time();
    //uint32_t min_next_time = curtime + s->step_pulse_ticks;
    //uint32_t min_next_time = curtime + timer_from_us(10000); //10ms
    //uint32_t min_next_time = curtime + timer_from_us(10);    
    uint32_t min_next_time = curtime + timer_from_us(3000000);

    sched_wake_task(&xy2_100_wake);

	s->time.waketime = min_next_time;
    return SF_RESCHEDULE;

	
}





void
command_config_xy2_stepper(uint32_t *args)
{
    struct xy2_stepper *s = oid_alloc(args[0], command_config_xy2_stepper, sizeof(*s));
	s->clk_pin_no = args[1];
	s->xbase_pin_no = args[2];
	s->xy2_x_pos = args[3];
	s->xy2_y_pos = args[4];

    ring_data_t * buff  = alloc_chunk(sizeof(ring_data_t)*M_BUF_SIZE_LEN);
    gp_ring_pio_mang = alloc_chunk(sizeof(*gp_ring_pio_mang));



    if( (gp_ring_pio_mang != NULL) && (buff != NULL))
        ring_buffer_init( &(gp_ring_pio_mang->ring_data), buff, M_BUF_SIZE_LEN);
    gp_ring_pio_mang->p_sync_pio = g_sync_pio_data;
    gp_ring_pio_mang->coord_factor = args[5];
    gp_ring_pio_mang->mirror_type  = args[6];

    //move_queue_setup(&s->mq, sizeof(struct xy2_stepper_move));
    s->time.func = xy2_stepper_event_tm;
	//to do xy2
    config_xy2_pio(s->clk_pin_no, s->xbase_pin_no);
#ifdef  M_FIFO_ISR_WORK_EN
    upadte_new_onedata_first(s->xy2_x_pos,s->xy2_y_pos);
    open_pio_isr();
#else
	send_xy_data(s->xy2_x_pos, s->xy2_y_pos, 0);
	s->time.waketime = timer_read_time()+ timer_from_us(10000);
	sched_add_timer(&s->time);    
#endif


}

/*
DECL_COMMAND(command_config_xy2_stepper, "config_xy2_stepper oid=%c clkb_pin=%c"
             " xb_pin=%c x_pos=%hu y_pos=%hu coord_factor=%u");
*/

DECL_COMMAND(command_config_xy2_stepper, "config_xy2_stepper oid=%c clkb_pin=%c"
             " xb_pin=%c x_pos=%hu y_pos=%hu coord_factor=%u mtype=%c");



// Return the 'struct xy2_stepper' for a given xy2_stepper oid
static struct xy2_stepper *
xy2_stepper_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_xy2_stepper);
}



// Report the current position of the xy2
void
command_xy2_stepper_get_position(uint32_t *args)
{
    uint8_t oid = args[0];
    struct xy2_stepper *s = xy2_stepper_oid_lookup(oid);
    irq_disable();
    //uint32_t position = stepper_get_position(s);
    uint16_t x_pos = s->xy2_x_pos;
    uint16_t y_pos = s->xy2_y_pos;    
    irq_enable();
    sendf("xy2_stepper_position oid=%c xpos=%hu ypos=%hu", oid, x_pos,y_pos);
}

DECL_COMMAND(command_xy2_stepper_get_position, "xy2_stepper_get_position oid=%c");


//void report_speed_stauts_ontest(void);

void
command_xy2_vir_get_position(uint32_t *args)
{
    uint8_t oid = args[0];
    //sts 
    //report_speed_stauts_ontest();
    
    //struct xy2_stepper *s = xy2_stepper_oid_lookup(oid);
    irq_disable();
    //uint32_t position = stepper_get_position(s);
    uint32_t x_pos_vir = g_sync_pio_data[0].position;
    uint32_t y_pos_vir = g_sync_pio_data[1].position;    
    irq_enable();
    sendf("xy2_vir_position oid=%c xvpos=%u yvpos=%u", oid, x_pos_vir,y_pos_vir);

}

DECL_COMMAND(command_xy2_vir_get_position, "xy2_vir_get_position oid=%c");



// Set the current position of the xy2
void
command_xy2_set_position(uint32_t *args)
{
    uint8_t oid = args[0];
    struct xy2_stepper *s = xy2_stepper_oid_lookup(oid);
    uint16_t x_pos = args[1];
    uint16_t y_pos = args[2];	
    irq_disable();
    //if (s->runcount)
    if (0)
    {
		shutdown("Can't set position time when xy2 100 active");
    }
	else
	{
    	s->xy2_x_pos = x_pos;
    	s->xy2_y_pos = y_pos;

        if ( (x_pos == 0) && (y_pos == 0) )
        {

            stepper_init_position_value();
            g_sync_pio_data[0].position = 0;
            g_sync_pio_data[1].position = 0;
            //  under test code
            //s->xy2_x_pos++;
            //s->xy2_y_pos++;
            
        }
		// write buff
#ifdef  M_FIFO_ISR_WORK_EN    
        upadte_new_onedata(s->xy2_x_pos, s->xy2_y_pos);
#else
		send_xy_data(s->xy2_x_pos, s->xy2_y_pos, 0);
#endif        

	}
    irq_enable();

	
}
DECL_COMMAND(command_xy2_set_position, "xy2_set_position oid=%c x_pos=%hu y_pos=%hu");




void
xy2_stepper_shutdown(void)
{
    uint8_t i;
    struct xy2_stepper *s;
    foreach_oid(i, s, command_config_xy2_stepper) {
		sched_del_timer(&s->time);
		#if 0
        move_queue_clear(&s->mq);
        xy2_stepper_stop(&s->stop_signal, 0);
		#endif
    }
#ifdef  M_FIFO_ISR_WORK_EN 
    close_pio_isr();

#endif

}
DECL_SHUTDOWN(xy2_stepper_shutdown);




//#define  M_START_PIO_TEST_EN   0
#ifdef  M_START_PIO_TEST_EN

#if 0
#define  M_SEEK_POSX   (0)
#define  M_SEEK_POSY   (0)
#else
#define  M_SEEK_POSX   (65535/2)
#define  M_SEEK_POSY   (65535/2)
#endif


struct timer gtest_time;



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


void
xy2_100_test_init(void)
{

    config_xy2_pio(0, 0);
	send_xy_data(M_SEEK_POSX, M_SEEK_POSY, 0);
    gtest_time.func = xy2_100t_event_tm;	
	gtest_time.waketime = timer_read_time()+ timer_from_us(10000); //10ms
	sched_add_timer(&gtest_time);	
	

}



DECL_INIT(xy2_100_test_init);



void
xy2_100_test_shutdown(void)
{

	sched_del_timer(&gtest_time);
		
}


DECL_SHUTDOWN(xy2_100_test_shutdown);




#endif



#if 0


// Schedule a set of steps with a given timing
void
command_queue_xy2_step(uint32_t *args)
{
    struct xy2_stepper *s = xy2_stepper_oid_lookup(args[0]);
    struct xy2_stepper_move *m = move_alloc();
    m->interval = args[1];
    m->timecount = args[2];
    if (!m->timecount)
        shutdown("Invalid count parameter");
    m->xy2_x_rto = args[3];
    m->xy2_y_rto = args[4];	
    //m->flags = 0;

    irq_disable();
    uint8_t flags = s->flags;
    if (s->runcount) {
        s->flags = flags;
        move_queue_push(&m->node, &s->mq);
    } else if (flags & SF_NEED_RESET) {
        move_free(m);
    } else {
        s->flags = flags;
        move_queue_push(&m->node, &s->mq);
        xy2_stepper_load_next(s);
        sched_add_timer(&s->time);
    }
    irq_enable();
}
DECL_COMMAND(command_queue_xy2_step,
             "queue_xy2_step oid=%c interval=%u timecount=%u x_pos=%hu y_pos=%hu");



/*
// Set the direction of the next queued step
void
command_set_next_step_dir(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    uint8_t nextdir = args[1] ? SF_NEXT_DIR : 0;
    irq_disable();
    s->flags = (s->flags & ~SF_NEXT_DIR) | nextdir;
    irq_enable();
}
DECL_COMMAND(command_set_next_step_dir, "set_next_step_dir oid=%c dir=%c");
*/


// Set an absolute time that the next step will be relative to
void
command_reset_xy2_step_clock(uint32_t *args)
{
    struct xy2_stepper *s = xy2_stepper_oid_lookup(args[0]);
    uint32_t waketime = args[1];
    irq_disable();
    if (s->runcount)
        shutdown("Can't reset time when xy2 stepper active");
    s->next_step_time = s->time.waketime = waketime;
    s->flags &= ~SF_NEED_RESET;
    irq_enable();
}
DECL_COMMAND(command_reset_xy2_step_clock, "reset_xy2_step_clock oid=%c clock=%u");

/*
// Return the current stepper position.  Caller must disable irqs.
static uint32_t
stepper_get_position(struct stepper *s)
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
*/






// Stop all moves for a given stepper (caller must disable IRQs)
static void
xy2_stepper_stop(struct trsync_signal *tss, uint8_t reason)
{
    struct xy2_stepper *s = container_of(tss, struct xy2_stepper, stop_signal);
    sched_del_timer(&s->time);
    s->next_step_time = s->time.waketime = 0;
    //s->position = -stepper_get_position(s);
    s->runcount = 0;
    s->flags = (s->flags & (SF_INVERT_STEP|SF_SINGLE_SCHED)) | SF_NEED_RESET;
    //gpio_out_write(s->dir_pin, 0);
    //if (!(HAVE_EDGE_OPTIMIZATION && s->flags & SF_SINGLE_SCHED))
        //gpio_out_write(s->step_pin, s->flags & SF_INVERT_STEP);
    while (!move_queue_empty(&s->mq)) {
        struct move_node *mn = move_queue_pop(&s->mq);
        struct xy2_stepper_move *m = container_of(mn, struct xy2_stepper_move, node);
        move_free(m);
    }
}

// Set the stepper to stop on a "trigger event" (used in homing)
void
command_xy2_stepper_stop_on_trigger(uint32_t *args)
{
    struct xy2_stepper *s = xy2_stepper_oid_lookup(args[0]);
    struct trsync *ts = trsync_oid_lookup(args[1]);
    trsync_add_signal(ts, &s->stop_signal, xy2_stepper_stop);
}
DECL_COMMAND(command_xy2_stepper_stop_on_trigger,
             "xy2_stepper_stop_on_trigger oid=%c trsync_oid=%c");

#endif


//sched_wake_task(&xy2_100_wake);
#define M_TESTSCAN_MODE  (0)


void
xy2_100_task(void)
{

    if (!sched_check_wake(&xy2_100_wake))
    {
        return;
    }

	if (M_TESTSCAN_MODE)
	{
	
        uint16_t x_pos = 0;
        uint16_t y_pos = 0;	
	    get_scan_dataxy(&x_pos, &y_pos);
		send_xy_data(x_pos, y_pos, 0);

	}
	else
	{
        send_xy_data(0, 0, 1);

	}
	
	
}
DECL_TASK(xy2_100_task);







