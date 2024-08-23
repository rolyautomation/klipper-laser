#ifndef __XY2_STEPPER_H
#define __XY2_STEPPER_H

#include <stdint.h> // uint8_t



int config_xy2_pio(unsigned      int  ck_sync_base, unsigned int  xy_base);
int send_xy_data(uint16_t posX, uint16_t posY, unsigned char mode);
int get_scan_dataxy(uint16_t      * pposX, uint16_t *  pposY);



unsigned char pio_fifo_status(void);
void pio_put_onedata(uint32_t data);
void open_pio_isr_reg(void);
void close_pio_isr_reg(void);
int  comb_senddata_format(uint16_t x_d, uint16_t y_d, uint32_t * ps_1st_data, uint32_t * ps_sec_data);


int  set_record_axis_info(uint8_t  gpio_step_num);
int  update_vir_postion_info(uint8_t  gpio_step_num, uint32_t position, uint16_t count, uint8_t mode);
uint8_t upadte_new_onedata(uint16_t posX, uint16_t posY);


#endif // stepper.h
