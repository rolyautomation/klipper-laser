#ifndef __XY2_STEPPER_H
#define __XY2_STEPPER_H

#include <stdint.h> // uint8_t



int config_xy2_pio(unsigned      int  ck_sync_base, unsigned int  xy_base);
int send_xy_data(uint16_t posX, uint16_t posY, unsigned char mode);
int get_scan_dataxy(uint16_t      * pposX, uint16_t *  pposY);






#endif // stepper.h
