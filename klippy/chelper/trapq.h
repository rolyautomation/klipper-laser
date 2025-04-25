#ifndef TRAPQ_H
#define TRAPQ_H

#include "list.h" // list_node
#include "common_ptd.h"


struct coord {
    union {
        struct {
            double x, y, z, a, b, c;
        };
        double axis[6];
    };
};




struct pwm_synci {
    unsigned char enf;
    unsigned char acd_val;
    unsigned char on_off;
    unsigned char pdlen;
    unsigned char ptagcode;
    double pwmmode, pwmval;
    double speed_pulse_ticks;
    double frestartcmd_sn;
};

struct move {
    double print_time, move_t;
    double start_v, half_accel;
    struct coord start_pos, axes_r;
    struct pwm_synci pwm_syncd; 
    struct power_table_s power_tabled;

    struct list_node node;
};

struct trapq {
    struct list_head moves, history;
};

struct pull_move {
    double print_time, move_t;
    double start_v, accel;
    double start_x, start_y, start_z;
    double start_a, start_b, start_c;

    double x_r, y_r, z_r;
    double a_r, b_r, c_r;

};

struct move *move_alloc(void);
double move_get_distance(struct move *m, double move_time);
struct coord move_get_coord(struct move *m, double move_time);
struct trapq *trapq_alloc(void);
void trapq_free(struct trapq *tq);
void trapq_check_sentinels(struct trapq *tq);
void trapq_add_move(struct trapq *tq, struct move *m);
void trapq_append(struct trapq *tq, double print_time
                  , double accel_t, double cruise_t, double decel_t
                  , double start_pos_x, double start_pos_y, double start_pos_z
                  , double start_pos_a, double start_pos_b, double start_pos_c
                  , double axes_r_x, double axes_r_y, double axes_r_z
                  , double axes_r_a, double axes_r_b, double axes_r_c
                  , double start_v, double cruise_v, double accel, unsigned char pwm_sync_en);
void trapq_append_extend(struct trapq *tq, double print_time
             , double accel_t, double cruise_t, double decel_t
             , double start_pos_x, double start_pos_y, double start_pos_z
             , double start_pos_a, double start_pos_b, double start_pos_c
             , double axes_r_x, double axes_r_y, double axes_r_z
             , double axes_r_a, double axes_r_b, double axes_r_c             
             , double start_v, double cruise_v, double accel, unsigned char pwm_sync_en
             , unsigned char * power_table, unsigned char len_power_table, unsigned int dist_count, unsigned char ptagcode);
void trapq_finalize_moves(struct trapq *tq, double print_time
                          , double clear_history_time);
void trapq_set_position(struct trapq *tq, double print_time
                        , double pos_x, double pos_y, double pos_z
                        , double pos_a, double pos_b, double pos_c);
int trapq_extract_old(struct trapq *tq, struct pull_move *p, int max
                      , double start_time, double end_time);

#endif // trapq.h
