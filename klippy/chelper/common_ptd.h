#ifndef __COMMON_PTD_H
#define __COMMON_PTD_H


#define MAX_PTABLE_LEN  (64)
//#define MAX_PTABLE_LEN  (96)
//M_FIFO_DATA_LEN
//M_PTABLE_BYTE_MLEN

struct power_table_s {
    unsigned char ddata[MAX_PTABLE_LEN];
    unsigned int  dist_count;
};


#endif // common_ptd.h
