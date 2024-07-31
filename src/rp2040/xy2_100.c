//#include <stdio.h>
//#include "pico/stdlib.h"
#include <stdint.h> // uint32_t
#include "sdk/pio.h"
#include "sched.h"
#include "hardware/structs/psm.h" // psm_hw
#include "hardware/structs/watchdog.h" // watchdog_hw

//#include "xy2_100.pio.h"

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
const int PIN_X = 0;
const int PIN_Y = 1;
const int PIN_CLOCK = 22;
const int PIN_SYNC = 3;
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
        pio_sm_put(pio, sm_x, x_frame);
        pio_sm_put(pio, sm_y, y_frame);
        //printf("X: %d, Y: %d\n", posX, posY);
        //sleep_us(100);
    }
    
}
DECL_INIT(xy2_100_init);