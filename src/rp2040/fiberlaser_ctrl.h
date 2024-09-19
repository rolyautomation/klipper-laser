#ifndef __FIBERLASER_CTRL_H
#define __FIBERLASER_CTRL_H

#include <stdint.h> // uint8_t


int set_agpio_out(unsigned int gpionum, uint8_t val);
int set_agpio_outstate(unsigned int gpionum, uint8_t val);

int set_manygpio_out(unsigned int gpio_startnum, unsigned int gpio_count, uint32_t val);
int set_manygpio_outstate(unsigned int gpio_startnum, unsigned int gpio_count, uint32_t val);


#endif // fiberlaser_ctrl.h
