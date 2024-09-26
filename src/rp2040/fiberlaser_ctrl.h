#ifndef __FIBERLASER_CTRL_H
#define __FIBERLASER_CTRL_H

#include <stdint.h> // uint8_t


int set_agpio_out(unsigned int gpionum, uint8_t val);
int set_agpio_outstate(unsigned int gpionum, uint8_t val);

extern int set_agpio_in(unsigned int gpionum);
extern int get_agpio_in(unsigned int gpionum);

int set_manygpio_out(unsigned int gpio_startnum, unsigned int gpio_count, uint32_t val);
int set_manygpio_outstate(unsigned int gpio_startnum, unsigned int gpio_count, uint32_t val);

int  handle_rec_command(uint8_t foid, uint8_t recmode_in, uint8_t recpower_in, uint8_t  laser_on_off);
uint8_t  chg_power_val(uint32_t val);

int  setup_pio_pwm_old(unsigned int  pin, uint32_t period,uint32_t level);

int  set_power_value(uint8_t x);
int  setup_pio_pwm(unsigned int pwmpin, uint32_t period,uint32_t level, unsigned int pin_start, unsigned int pin_latch);

void change_pwm_duty(uint32_t level);

void  direct_set_pwm_pulse_width_fibertype(uint8_t pwd_oid, uint32_t val);

#endif // fiberlaser_ctrl.h
