#ifndef __FIBERLASER_CTRL_H
#define __FIBERLASER_CTRL_H

#include <stdint.h> // uint8_t


int set_agpio_out(unsigned int gpionum, uint8_t val);
int set_agpio_outstate(unsigned int gpionum, uint8_t val);

extern int set_agpio_in(unsigned int gpionum);
extern int get_agpio_in(unsigned int gpionum);

int set_manygpio_out(unsigned int gpio_startnum, unsigned int gpio_count, uint32_t val);
int set_manygpio_outstate(unsigned int gpio_startnum, unsigned int gpio_count, uint32_t val);

int  handle_rec_command(uint8_t foid, uint8_t recmode_in, uint8_t recpower_in);
int  setup_pio_pwm(unsigned int  pin, uint32_t period,uint32_t level);

#endif // fiberlaser_ctrl.h
