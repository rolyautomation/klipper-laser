//#include <stdio.h>
//#include "pico/stdlib.h"
#include <stdint.h> // uint32_t
void tight_loop_contents(void) {}

#include "sdk/pio.h"
#include "sched.h"
#include "hardware/structs/psm.h" // psm_hw
#include "hardware/structs/watchdog.h" // watchdog_hw

//#include "xy2_100.pio.h"
#define  M_XY2_100_V02   02

#define  M_NO_SDK_ONWIN
#ifdef   M_NO_SDK_ONWIN
void  open_piomodulclk(void);
#endif




#ifdef  M_XY2_100_V02

//#define M_DEBUG_INFO_XY   1
#ifdef  M_DEBUG_INFO_XY
int xy2_debug_info(const char* fmt, ...)
{
    int ret = 0;

#if  1   
    va_list args;
    va_start(args, fmt);
    ret = vprintf(fmt, args);
    //printf(fmt, args);  //use it have eeror 
    va_end(args);
#endif

    return(ret);

}
#endif


#define MS_H_C2   0
#define MS_H_C1   0
#define MS_H_C0   1
#define MS_H_C02  ( (MS_H_C2 << 2) | (MS_H_C1 << 1) | (MS_H_C0 << 0) )
#define MS_H_C_POS (16+1)
#define F_XY_VAL_P (1)
#define F_HIGH_RIGHT_POS  (12)



#define M_SEL_PIO  (pio0)
#define M_SEL_SM   (0)



#if  0
#define M_IO_CK_SYNC_BASE (4)
#define M_IO_XY_BASE      (2)
#else
#define M_IO_CK_SYNC_BASE (2)
#define M_IO_XY_BASE      (0)
#endif


#define M_TEST_SCAN_MODE  1
#if M_TEST_SCAN_MODE


uint16_t posX = 0;
uint16_t posY = 0;
bool increment_plus = true;
int increment_position(void) {
    if (increment_plus) {
        posX = posX + 8;
        posY = posY + 8;
    } else {
        posX = posX - 8;
        posY = posY - 8;
    }
    if (posX >= 65525) {
        increment_plus = false;
    }
    if (posX <= 10) {
        increment_plus = true;
    }
    return 0;
}


int get_scan_dataxy(uint16_t      * pposX, uint16_t *  pposY)
{
	int iret = 0;

    increment_position();
	if (pposX != NULL)
	{
		*pposX = posX;

	}
	if (pposY != NULL)
	{
		*pposY = posY;

	}
	return(iret);
	
}



#endif


uint16_t get_parity_bit(uint16_t value) {
    value ^= value >> 8;
    value ^= value >> 4;
    value ^= value >> 2;
    value ^= value >> 1;
    return value & 1;
}



//int  comb_senddata_format(uint16_t x_d, uint16_t y_d, uint32_t * ps_xd, uint32_t * ps_yd)
int  comb_senddata_format(uint16_t x_d, uint16_t y_d, uint32_t * ps_1st_data, uint32_t * ps_sec_data)
{
    int iret = 0;

    uint32_t  xd_temp = 0;
    uint32_t  yd_temp = 0;
    uint32_t  d_temp_1st = 0;
    uint32_t  d_temp_sec = 0;    
    int  i = 0;
    uint8_t   bitx = 0;
    uint8_t   bity = 0;

    xd_temp =  (MS_H_C02 <<  MS_H_C_POS) | (x_d << F_XY_VAL_P) | get_parity_bit(x_d);
    yd_temp =  (MS_H_C02 <<  MS_H_C_POS) | (y_d << F_XY_VAL_P) | get_parity_bit(y_d);

#ifdef  M_DEBUG_INFO_XY
    xy2_debug_info("xd_temp=%08X\n",xd_temp);
    xy2_debug_info("yd_temp=%08X\n",yd_temp);
#endif

    for(i=0; i < 20 ; i++)
    {
       bitx = xd_temp & 0x01; 
       bity = yd_temp & 0x01;
       xd_temp = xd_temp >> 1;
       yd_temp = yd_temp >> 1;      

       if (i < 10)
       {
           d_temp_sec = d_temp_sec >> 1;
           if ( bitx )
           {
             d_temp_sec = d_temp_sec | 0x80000000;
           }
           d_temp_sec = d_temp_sec >> 1;
           if ( bity )
           {
             d_temp_sec = d_temp_sec | 0x80000000;

           }

       }
       else
       {

           d_temp_1st = d_temp_1st >> 1;
           if ( bitx )
           {
             d_temp_1st = d_temp_1st | 0x80000000;
           }
           d_temp_1st = d_temp_1st >> 1;
           if ( bity )
           {
             d_temp_1st = d_temp_1st | 0x80000000;
             
           }

       }

    }
 
    if (ps_1st_data != NULL)
    {
        *ps_1st_data = d_temp_1st;
    }
    if (ps_sec_data != NULL)
    {
        *ps_sec_data = d_temp_sec;
    }   

    return(iret);

 
}




// --------------- //
// xy2_100_piocode //
// --------------- //

#define xy2_100_piocode_wrap_target 0
#define xy2_100_piocode_wrap 10

static const uint16_t xy2_100_piocode_program_instructions[] = {
            //     .wrap_target
    0x98a0, //  0: pull   block           side 3     
    0xf849, //  1: set    y, 9            side 3     
    0x7902, //  2: out    pins, 2         side 3 [1] 
    0x1182, //  3: jmp    y--, 2          side 2 [1] 
    0xf848, //  4: set    y, 8            side 3     
    0x98a0, //  5: pull   block           side 3     
    0x7902, //  6: out    pins, 2         side 3 [1] 
    0x1186, //  7: jmp    y--, 6          side 2 [1] 
    0x6902, //  8: out    pins, 2         side 1 [1] 
    0xa142, //  9: nop                    side 0 [1] 
    0xb942, // 10: nop                    side 3 [1] 
            //     .wrap
};

#if !PICO_NO_HARDWARE


static const struct pio_program xy2_100_piocode_program = {
    .instructions = xy2_100_piocode_program_instructions,
    .length = 11,
    .origin = -1,
};


static inline pio_sm_config xy2_100_piocode_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + xy2_100_piocode_wrap_target, offset + xy2_100_piocode_wrap);
    sm_config_set_sideset(&c, 2, false, false);
    return c;
}


#endif



void xy2_100_piocode_program_init(PIO pio, uint sm, uint offset,uint clk_base, uint xy_base) {

   pio_sm_config c = xy2_100_piocode_program_get_default_config(offset);

   //sm_config_set_in_pins(&c, pin_sio0);
   sm_config_set_set_pins(&c, xy_base, 2);

   sm_config_set_out_pins(&c, xy_base, 2);
   sm_config_set_out_shift(&c, false, false, 0);
   //sm_config_set_in_shift(&c, false, false, 0);
   sm_config_set_sideset_pins(&c, clk_base);

   //sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);  //add deep fifo

   //sm_config_set_in_pins(&c, PIN_CLOCK);
   pio_sm_set_consecutive_pindirs(pio, sm, clk_base, 2, true);
   pio_sm_set_consecutive_pindirs(pio, sm, xy_base, 2, true);

   pio_sm_set_pins_with_mask(pio, sm, (3u << clk_base), (3u << clk_base) | (3u << xy_base));


   //pio_sm_set_pins_with_mask(pio, sm, 0, (1u << pin_sck) | (1u << pin_mosi));
   //pio_sm_set_pindirs_with_mask(pio, sm, (1u << pin_sck) | (1u << pin_mosi), (1u << pin_sck)
   // | (1u << pin_mosi) | (1u << pin_miso));  

   pio_gpio_init(pio, clk_base);
   pio_gpio_init(pio, clk_base+1);
   pio_gpio_init(pio, xy_base);
   pio_gpio_init(pio, xy_base+1);      

   //sm_config_set_clkdiv(&c, 31.25f);
   //sm_config_set_clkdiv(&c, 62.5f);    //cycle 2us,500k
   sm_config_set_clkdiv(&c, 15.625f);  //cycle .5us, 2M

   pio_sm_init(pio, sm, offset, &c);




}







int config_xy2_pio(unsigned      int  ck_sync_base, unsigned int  xy_base)
{


    int iret = 0;
	PIO pio = M_SEL_PIO;
	uint sm_xy2m  = M_SEL_SM;



#ifdef   M_NO_SDK_ONWIN
    open_piomodulclk();
#endif	

    if(ck_sync_base == xy_base)
    {
    
		ck_sync_base = M_IO_CK_SYNC_BASE;
        xy_base  = M_IO_XY_BASE;
		

    }
	
    uint offset_xy2m = pio_add_program(pio, &xy2_100_piocode_program);
    //xy2_100_piocode_program_init(pio, sm_xy2m, offset_xy2m,M_IO_CK_SYNC_BASE,M_IO_XY_BASE);
    xy2_100_piocode_program_init(pio, sm_xy2m, offset_xy2m,ck_sync_base,xy_base);
    pio_sm_set_enabled(pio, sm_xy2m, true);
	return(iret);
	
		

}


int send_xy_data(uint16_t posX, uint16_t posY, unsigned char mode)
{
	int iret = 0;
	PIO pio = M_SEL_PIO;
	uint sm_xy2m  = M_SEL_SM;	
    static uint32_t data_snd_part1 = 0;
    static uint32_t data_snd_part2 = 0;  	

	


    if (mode == 0)
	{
    	comb_senddata_format(posX, posY, &data_snd_part1, &data_snd_part2);
		
	}
    pio_sm_put_blocking(pio, sm_xy2m, data_snd_part1);
    pio_sm_put_blocking(pio, sm_xy2m, data_snd_part2); 
	
	
	return iret;
	

}


// 1: full
unsigned char pio_fifo_status(void)
{

    PIO pio = M_SEL_PIO;
	uint sm_xy2m  = M_SEL_SM;	  
    unsigned char  ist = pio_sm_is_tx_fifo_full(pio, sm_xy2m);
    return (ist);

}


void pio_put_onedata(uint32_t data)
{

    PIO pio = M_SEL_PIO;
	uint sm_xy2m  = M_SEL_SM;	
    pio_sm_put(pio, sm_xy2m,data);


}


void open_pio_isr_reg(void)
{
    PIO pio = M_SEL_PIO;
	//uint sm_xy2m  = M_SEL_SM;
    pio_set_irq0_source_enabled(pio, pis_sm0_tx_fifo_not_full, true);


}

void close_pio_isr_reg(void)
{
    PIO pio = M_SEL_PIO;
    //uint sm_xy2m  = M_SEL_SM;
    pio_set_irq0_source_enabled(pio, pis_sm0_tx_fifo_not_full, false);

}



void run_pio_isr_reg(void)
{



}


#else   //M_XY2_100_V02
// ------------- //
// xy2_100_clock //
// ------------- //

#define xy2_100_clock_wrap_target 0
#define xy2_100_clock_wrap 1

static const uint16_t xy2_100_clock_program_instructions[] = {
            //     .wrap_target
    0xe001, //  0: set    pins, 1                    
    0xe000, //  1: set    pins, 0                    
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program xy2_100_clock_program = {
    .instructions = xy2_100_clock_program_instructions,
    .length = 2,
    .origin = -1,
};

static inline pio_sm_config xy2_100_clock_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + xy2_100_clock_wrap_target, offset + xy2_100_clock_wrap);
    return c;
}
#endif

// --------- //
// xy2_100_x //
// --------- //

#define xy2_100_x_wrap_target 0
#define xy2_100_x_wrap 9

static const uint16_t xy2_100_x_program_instructions[] = {
            //     .wrap_target
    0x8080, //  0: pull   noblock                    
    0xa027, //  1: mov    x, osr                     
    0xe052, //  2: set    y, 18                      
    0x2020, //  3: wait   0 pin, 0                   
    0x20a0, //  4: wait   1 pin, 0                   
    0x7801, //  5: out    pins, 1         side 1     
    0x0083, //  6: jmp    y--, 3                     
    0x2020, //  7: wait   0 pin, 0                   
    0x20a0, //  8: wait   1 pin, 0                   
    0x7001, //  9: out    pins, 1         side 0     
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program xy2_100_x_program = {
    .instructions = xy2_100_x_program_instructions,
    .length = 10,
    .origin = -1,
};

static inline pio_sm_config xy2_100_x_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + xy2_100_x_wrap_target, offset + xy2_100_x_wrap);
    sm_config_set_sideset(&c, 2, true, false);
    return c;
}
#endif

// --------- //
// xy2_100_y //
// --------- //

#define xy2_100_y_wrap_target 0
#define xy2_100_y_wrap 6

static const uint16_t xy2_100_y_program_instructions[] = {
            //     .wrap_target
    0x8080, //  0: pull   noblock                    
    0xa027, //  1: mov    x, osr                     
    0xe053, //  2: set    y, 19                      
    0x2020, //  3: wait   0 pin, 0                   
    0x20a0, //  4: wait   1 pin, 0                   
    0x6001, //  5: out    pins, 1                    
    0x0083, //  6: jmp    y--, 3                     
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program xy2_100_y_program = {
    .instructions = xy2_100_y_program_instructions,
    .length = 7,
    .origin = -1,
};

static inline pio_sm_config xy2_100_y_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + xy2_100_y_wrap_target, offset + xy2_100_y_wrap);
    return c;
}
#endif

int interval = 5;
uint16_t posX = 0;
uint16_t posY = 0;
const int PIN_X = 2;
const int PIN_Y = 3;
const int PIN_CLOCK = 4;
const int PIN_SYNC = 5;
bool increment_plus = true;

void xy2_100_clock_program_init(PIO pio, uint sm, uint offset) {
   pio_gpio_init(pio, PIN_CLOCK);
   pio_sm_set_consecutive_pindirs(pio, sm, PIN_CLOCK, 1, true);
   pio_sm_config c = xy2_100_clock_program_get_default_config(offset);
   sm_config_set_set_pins(&c, PIN_CLOCK, 1);
   sm_config_set_clkdiv(&c, 31.25f);
   pio_sm_init(pio, sm, offset, &c);
}

void xy2_100_x_program_init(PIO pio, uint sm, uint offset) {
   pio_gpio_init(pio, PIN_X);
   pio_gpio_init(pio, PIN_SYNC);
   pio_sm_set_consecutive_pindirs(pio, sm, PIN_X, 1, true);
   pio_sm_set_consecutive_pindirs(pio, sm, PIN_SYNC, 1, true);
   pio_sm_config c = xy2_100_x_program_get_default_config(offset);
   sm_config_set_out_pins(&c, PIN_X, 1);
   sm_config_set_out_shift(&c, false, false, 0);
   sm_config_set_sideset_pins(&c, PIN_SYNC);
   sm_config_set_in_pins(&c, PIN_CLOCK);
   pio_sm_init(pio, sm, offset, &c);
}

void xy2_100_y_program_init(PIO pio, uint sm, uint offset) {
   pio_gpio_init(pio, PIN_Y);
   pio_sm_set_consecutive_pindirs(pio, sm, PIN_Y, 1, true);
   pio_sm_config c = xy2_100_y_program_get_default_config(offset);
   sm_config_set_out_pins(&c, PIN_Y, 1);
   sm_config_set_in_pins(&c, PIN_CLOCK);
   sm_config_set_out_shift(&c, false, false, 0);
   pio_sm_init(pio, sm, offset, &c);
}

uint16_t get_parity_bit(uint16_t value) {
    value ^= value >> 8;
    value ^= value >> 4;
    value ^= value >> 2;
    value ^= value >> 1;
    return value & 1;
}

uint32_t build_frame(uint16_t value) {
    uint32_t frame = 0;
    frame = frame << 1; frame = frame | 0;
    frame = frame << 1; frame = frame | 0;
    frame = frame << 1; frame = frame | 1;
    frame = frame << 16; frame = frame | value;
    frame = frame << 1; frame = frame | get_parity_bit(value);
    frame = frame << 12;
    return frame;
}

int increment_position(void) {
    if (increment_plus) {
        posX = posX + 8;
        posY = posY + 8;
    } else {
        posX = posX - 8;
        posY = posY - 8;
    }
    if (posX >= 65525) {
        increment_plus = false;
    }
    if (posX <= 10) {
        increment_plus = true;
    }
    return 0;
}

int jump_position(int posX, int posY) {
    // TODO: Implement this function
    return 0;
}

int alternate_bit(void) {
    // Value in binary is: 0b1011001110000010
    // The pattern helps with debugging, parity (even) = 0
    posX = 45954;
    posY = 45954;
    return 0;
}

void
xy2_100_init(void)
{
    PIO pio = pio1;


#ifdef   M_NO_SDK_ONWIN
    open_piomodulclk();
#endif

    uint offset_clock = pio_add_program(pio, &xy2_100_clock_program);
    uint offset_x = pio_add_program(pio, &xy2_100_x_program);
    uint offset_y = pio_add_program(pio, &xy2_100_y_program);

    uint sm_clocksync = 0;
    uint sm_x = 1;
    uint sm_y = 2;

    xy2_100_clock_program_init(pio, sm_clocksync, offset_clock);
    xy2_100_x_program_init(pio, sm_x, offset_x);
    xy2_100_y_program_init(pio, sm_y, offset_y);

    pio_sm_set_enabled(pio, sm_x, true);
    pio_sm_set_enabled(pio, sm_y, true);
    pio_sm_set_enabled(pio, sm_clocksync, true);

    //alternate_bit();

    const int ledPin = 25;
    gpio_init(ledPin);
    gpio_set_dir(ledPin, GPIO_OUT);
    gpio_put(ledPin,1);

    while (true)
    {
        watchdog_reset();
        //No code whatsoever
        //Tested, the LED will blink once every second or so, likely the watchdog causing hard reset

        increment_position();
        //alternate_bit();
        uint32_t x_frame = build_frame(posX);
        uint32_t y_frame = build_frame(posY);
        pio_sm_put_blocking(pio, sm_x, x_frame);
        pio_sm_put_blocking(pio, sm_y, y_frame);
        //printf("X: %d, Y: %d\n", posX, posY);
        //sleep_us(100);
    }
    
}
DECL_INIT(xy2_100_init);


#endif //M_XY2_100_V02

int set_agpio_in(unsigned int gpionum);
int get_agpio_in(unsigned int gpionum);

#define M_GPIO_CTRL_EXTEND

#ifdef M_GPIO_CTRL_EXTEND

int set_agpio_out(unsigned int gpionum, uint8_t val)
{
    
    gpio_init(gpionum);
    gpio_set_dir(gpionum, GPIO_OUT);
    gpio_put(gpionum,val);
    return 0;

}


int set_agpio_outstate(unsigned int gpionum, uint8_t val)
{
    gpio_put(gpionum,val);
    return 0;
}



int set_agpio_in(unsigned int gpionum)
{
    gpio_init(gpionum);
    return 0;

}

int get_agpio_in(unsigned int gpionum)
{
    int iret = 0;
    iret = gpio_get(gpionum);
    return(iret);
}


static void gpio_init_mask_spec(uint gpio_mask) {

    for(uint i=0;i<NUM_BANK0_GPIOS;i++) {
        if (gpio_mask & 1) {
            gpio_init(i);
        }
        gpio_mask >>= 1;
    }

}


int set_manygpio_out(unsigned int gpio_startnum, unsigned int gpio_count, uint32_t val)
{
    uint32_t  gpio_mask =  ((1 << gpio_count) - 1) << gpio_startnum;
    uint32_t  gpio_val  =  val << gpio_startnum;
    gpio_init_mask_spec(gpio_mask);
    gpio_set_dir_out_masked(gpio_mask);
    gpio_put_masked(gpio_mask,gpio_val);

    return 0;

}


int set_manygpio_outstate(unsigned int gpio_startnum, unsigned int gpio_count, uint32_t val)
{
    uint32_t  gpio_mask =  ((1 << gpio_count) - 1) << gpio_startnum;
    uint32_t  gpio_val  =  val << gpio_startnum;    
    gpio_put_masked(gpio_mask,gpio_val);
    return 0;

}



#define M_SEL_PIO_PSYNC   (pio1)
#define M_SEL_SM_PSYNC      (0)
#define M_SEL_SM_SETPOWER   (1)


#define pwm_wrap_target 0
#define pwm_wrap 6


static const uint16_t pwm_program_instructions[] = {
            //     .wrap_target
    0x9080, //  0: pull   noblock         side 0     
    0xa027, //  1: mov    x, osr                     
    0xa046, //  2: mov    y, isr                     
    0x00a5, //  3: jmp    x != y, 5                  
    0x1806, //  4: jmp    6               side 1     
    0xa042, //  5: nop                               
    0x0083, //  6: jmp    y--, 3                     
            //     .wrap
};




static const struct pio_program pwm_program = {
    .instructions = pwm_program_instructions,
    .length = 7,
    .origin = -1,
};

static inline pio_sm_config pwm_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + pwm_wrap_target, offset + pwm_wrap);
    sm_config_set_sideset(&c, 2, true, false);
    return c;
}



// --------------- //
// fiber_set_power //
// --------------- //

#define fiber_set_power_wrap_target 0
#define fiber_set_power_wrap 6

static const uint16_t fiber_set_power_program_instructions[] = {
            //     .wrap_target
    0x80a0, //  0: pull   block           side 0     
    0x6008, //  1: out    pins, 8         side 0     
    0xa042, //  2: nop                    side 0     
    0xb042, //  3: nop                    side 1     
    0xb042, //  4: nop                    side 1     
    0xa042, //  5: nop                    side 0     
    0xa042, //  6: nop                    side 0     
            //     .wrap
};


static const struct pio_program fiber_set_power_program = {
    .instructions = fiber_set_power_program_instructions,
    .length = 7,
    .origin = -1,
};

static inline pio_sm_config fiber_set_power_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + fiber_set_power_wrap_target, offset + fiber_set_power_wrap);
    sm_config_set_sideset(&c, 1, false, false);
    return c;
}





static inline void pwm_program_init(PIO pio, uint sm, uint offset, uint pin) {
   pio_gpio_init(pio, pin);
   pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
   pio_sm_config c = pwm_program_get_default_config(offset);
   sm_config_set_sideset_pins(&c, pin);
   pio_sm_init(pio, sm, offset, &c);
}



// Write `period` to the input shift register
void pio_pwm_set_period(PIO pio, uint sm, uint32_t period) {
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_put_blocking(pio, sm, period);
    pio_sm_exec(pio, sm, pio_encode_pull(false, false));
    pio_sm_exec(pio, sm, pio_encode_out(pio_isr, 32));
    pio_sm_set_enabled(pio, sm, true);
}

// Write `level` to TX FIFO. State machine will copy this into X.
void pio_pwm_set_level(PIO pio, uint sm, uint32_t level) {
    pio_sm_put_blocking(pio, sm, level);
}

void change_pwm_duty(uint32_t level){

     pio_pwm_set_level(M_SEL_PIO_PSYNC, M_SEL_SM_PSYNC, level);       

}



static inline void fiber_set_power_program_init(PIO pio, uint sm, uint offset, uint data_pin, uint clk_pin, float clk_div) {

    int i = 0;
    for(i=0;i < 8; i++)
    {
       pio_gpio_init(pio, data_pin+i); 
    }
    pio_gpio_init(pio, clk_pin);

    pio_sm_set_consecutive_pindirs(pio, sm, data_pin, 8, true);
    pio_sm_set_consecutive_pindirs(pio, sm, clk_pin, 1, true);
    pio_sm_config c = fiber_set_power_program_get_default_config(offset);

    sm_config_set_sideset_pins(&c, clk_pin);
    sm_config_set_out_pins(&c, data_pin, 8);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv(&c, clk_div);
    //sm_config_set_out_shift(&c, false, true, 8);
    sm_config_set_out_shift(&c, true, false, 0);

    pio_sm_init(pio, sm, offset, &c);
    //pio_sm_set_enabled(pio, sm, true);
}



static inline int  fiber_set_power_put(PIO pio, uint sm, uint8_t x) {
    //while (pio_sm_is_tx_fifo_full(pio, sm))
    //;
    if(pio_sm_is_tx_fifo_full(pio, sm) > 0)
    {
        return 0;
    }
    else
    {
        *(volatile uint8_t*)&pio->txf[sm] = x;
        return 1;
    }
    
}

int  set_power_value(uint8_t x)
{
     int iret = 0;
     iret = fiber_set_power_put(M_SEL_PIO_PSYNC, M_SEL_SM_SETPOWER, x);
     return(iret);

}


int  setup_pio_pwm_old(uint pin, uint32_t period,uint32_t level)
{
    int iret = 0;

    //PIO pio = pio1;
    //int sm = 0;
    PIO pio = M_SEL_PIO_PSYNC;
    int sm = M_SEL_SM_PSYNC;   

#ifdef   M_NO_SDK_ONWIN
    open_piomodulclk();
#endif	

    uint offset = pio_add_program(pio, &pwm_program);
    pwm_program_init(pio, sm, offset, pin);
    pio_pwm_set_period(pio, sm, period);
    pio_pwm_set_level(pio, sm, level);
    return(iret);

}


//#define LATCH_CLK_DIV 20.f
//#define LATCH_CLK_DIV   (62.5f)   //1us
#define LATCH_CLK_DIV   (125.f)   //2us
//test 
//#define LATCH_CLK_DIV   (250.f)   //4us


int  setup_pio_pwm(uint pwmpin, uint32_t period,uint32_t level, uint pin_start, uint pin_latch)
{

    int iret = 0;

    //PIO pio = pio1;
    //int sm = 0;
    PIO pio = M_SEL_PIO_PSYNC;
    int sm = M_SEL_SM_PSYNC;   
    int sm_latch = M_SEL_SM_SETPOWER;

#ifdef   M_NO_SDK_ONWIN
    open_piomodulclk();
#endif	

    uint offset = pio_add_program(pio, &pwm_program);
    uint offset_latch = pio_add_program(pio, &fiber_set_power_program);


    pwm_program_init(pio, sm, offset, pwmpin);
    fiber_set_power_program_init(pio, sm_latch, offset_latch, pin_start, pin_latch, LATCH_CLK_DIV);

    pio_pwm_set_period(pio, sm, period);
    pio_pwm_set_level(pio, sm, level);


    pio_sm_set_enabled(pio, sm_latch, true);


    return(iret);


}



#endif



#define M_PIO_CTRL_DCMOTOR
#ifdef  M_PIO_CTRL_DCMOTOR

#define M_SEL_PIO_DCM     (pio1)
#define M_SEL_SM_DCM      (2)


// ---------------- //
// dcmotor_run_prog //
// ---------------- //
// note: 24 bits  waitcycle,  8 bits

#define dcmotor_run_prog_wrap_target 0
#define dcmotor_run_prog_wrap 5

static const uint16_t dcmotor_run_prog_program_instructions[] = {
            //     .wrap_target
    0x80a0, //  0: pull   block                      
    0x6058, //  1: out    y, 24                      
    0x6002, //  2: out    pins, 2                    
    0xa042, //  3: nop                               
    0x0083, //  4: jmp    y--, 3                     
    0x6002, //  5: out    pins, 2                    
            //     .wrap
};

//#if !PICO_NO_HARDWARE
static const struct pio_program dcmotor_run_prog_program = {
    .instructions = dcmotor_run_prog_program_instructions,
    .length = 6,
    .origin = -1,
};

static inline pio_sm_config dcmotor_run_prog_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + dcmotor_run_prog_wrap_target, offset + dcmotor_run_prog_wrap);
    return c;
}
//#endif




static inline void dcmotor_run_program_init(PIO pio, uint sm, uint offset, uint ab_pin, float clk_div) {

    pio_gpio_init(pio, ab_pin);
    pio_gpio_init(pio, ab_pin+1);

    pio_sm_set_consecutive_pindirs(pio, sm, ab_pin, 2, true);
    pio_sm_config c = dcmotor_run_prog_program_get_default_config(offset);

    sm_config_set_out_pins(&c, ab_pin, 2);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv(&c, clk_div);
    sm_config_set_out_shift(&c, true, false, 0);

    pio_sm_set_pins_with_mask(pio, sm, 0, (3u << ab_pin) );

    pio_sm_init(pio, sm, offset, &c);
    //pio_sm_set_enabled(pio, sm, true);
    

}




//#define DCMU_CLK_DIV   (62.5f)   //1us
#define   DCMU_CLK_DIV   (125.f)   //2us
//#define DCMU_CLK_DIV   (250.f)   //4us

int  setup_pio_dcmctrlrun(uint ab_pin)
{
    int iret = 0;

    PIO pio = M_SEL_PIO_DCM;
    int dcm_sm = M_SEL_SM_DCM;   

#ifdef   M_NO_SDK_ONWIN
    open_piomodulclk();
#endif	

    uint offset_dcm = pio_add_program(pio, &dcmotor_run_prog_program);
    dcmotor_run_program_init(pio, dcm_sm, offset_dcm, ab_pin, DCMU_CLK_DIV);
    pio_sm_set_enabled(pio, dcm_sm, true);

    if ( offset_dcm > 0)
        iret = 1;

    return(iret);

}



static inline void pio_dcmctrlrun_fun(PIO pio, uint sm, uint32_t dcm_instr) {
    pio_sm_put_blocking(pio, sm, dcm_instr);
}


//  micro us  ctrl
int  send_dcmctrlrun_instr(uint ab_pin, uint32_t dcm_instr)
{
     int iret = 0;
     static int run_startflag = 0;

     if (run_startflag == 0)
     {
        run_startflag = 1;
        setup_pio_dcmctrlrun(ab_pin);

     }
     pio_dcmctrlrun_fun(M_SEL_PIO_DCM, M_SEL_SM_DCM, dcm_instr);
     return(iret);

}

#endif
