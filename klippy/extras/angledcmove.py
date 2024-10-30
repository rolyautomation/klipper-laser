# Support angle dc move
#
# Copyright (C) 2024-2028  jinqiang <jinqiang@ecomedge.io>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import bus

REPORT_TIME = .8
AS5600_CHIP_ADDR = 0x36



'''
#define REG_ZMCO        _u(0x00)
#define REG_ZPOS        _u(0x01)
#define REG_MPOS        _u(0x03)
#define REG_MANG        _u(0x05)
#define REG_CONF        _u(0x07)
#define REG_RAW_ANGLE   _u(0x0C)
#define REG_ANGLE       _u(0x0E)
#define REG_STATUS      _u(0x0B)
#define REG_AGC         _u(0x1A)
#define REG_MAGNITUDE   _u(0x1B)
#define REG_BURN        _u(0xFF)
/*
# PM(1:0)     1:0     Power Mode      00 = NOM, 01 = LPM1, 10 = LPM2, 11 = LPM3
# HYST(1:0)   3:2     Hysteresis      00 = OFF, 01 = 1 LSB, 10 = 2 LSBs, 11 = 3 LSBs
# OUTS(1:0)   5:4     Output Stage    00 = analog (full range from 0% to 100% between GND and VDD, 01 = analog (reduced range from 10% to 90% between GND and VDD, 10 = digital PWM
# PWMF(1:0)   7:6     PWM Frequency   00 = 115 Hz; 01 = 230 Hz; 10 = 460 Hz; 11 = 920 Hz
# SF(1:0)     9:8     Slow Filter     00 = 16x (1); 01 = 8x; 10 = 4x; 11 = 2x
# FTH(2:0)    12:10   Fast Filter Threshold   000 = slow filter only, 001 = 6 LSBs, 010 = 7 LSBs, 011 = 9 LSBs,100 = 18 LSBs, 101 = 21 LSBs, 110 = 24 LSBs, 111 = 10 LSBs
# WD          13      Watchdog        0 = OFF, 1 = ON
*/

'''


AS5600_REGS = {
    'REG_ZMCO': 0x00, 'REG_ZPOS': 0x01, 'REG_MPOS': 0x03, 'REG_MANG': 0x05,
    'REG_CONF': 0x07, 'REG_RAW_ANGLE': 0x0C, 'REG_ANGLE': 0x0E,
    'REG_STATUS': 0x0B, 'REG_AGC': 0x1A, 'REG_MAGNITUDE': 0x1B, 'REG_BURN': 0xFF
}


STATUS_MD = 1 << 5
STATUS_ML = 1 << 4
STATUS_MH = 1 << 3


class Angledcmove:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split(' ')[-1]
        self.reactor = self.printer.get_reactor()
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=AS5600_CHIP_ADDR, default_speed=100000)
        self.mcu = self.i2c.get_mcu()
        self.chip_registers = AS5600_REGS
        self.last_angle = 0
        self.magnet_status = 'nom'
        self.magnet_flag = 0
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_mux_command("ANGLE_AS5600_INIT", "ANGLE", self.name,
                                        self.cmd_AS5600_INIT,
                                        desc=self.cmd_AS5600_INIT_help)

        self.gcode.register_command("ANGLE_LAST_VAL", self.cmd_ANGLE_LAST_VAL)  


    cmd_AS5600_INIT_help = "Report on the state of as5600"
    def cmd_AS5600_INIT(self, gcmd):
        self.cur_zpos = self.read_register_D16('REG_ZPOS')
        self.cur_mpos = self.read_register_D16('REG_MPOS')
        msg = "as5600:"
        msg = msg + "ZPOS=" + hex(self.cur_zpos) + ","
        msg = msg + "MPOS=" + hex(self.cur_mpos) + ","
        gcmd.respond_info(self.name + ":" + msg)

    def cmd_ANGLE_LAST_VAL(self, gcmd):
        self.check_magnet_status()
        self.last_angle = self.read_register_D16('REG_RAW_ANGLE')
        hex_str = hex(self.last_angle)
        msg = "%s:REG_RAW_ANGLE=%s:%d" % (self.magnet_status,hex_str,self.last_angle)
        gcmd.respond_info(msg)
        #logging.info("\n %s\n" ,msg)
        pass
 
    def read_register(self, reg_name, read_len):
        # read a single register
        regs = [self.chip_registers[reg_name]]
        params = self.i2c.i2c_read(regs, read_len)
        return bytearray(params['response'])

    def write_register(self, reg_name, data):
        if type(data) is not list:
            data = [data]
        reg = self.chip_registers[reg_name]
        data.insert(0, reg)
        self.i2c.i2c_write(data)

    def write_register_D16(self, reg_name, data_16):
        data = [ (data_16 >> 8) & 0xff, data_16 & 0xff ]
        self.write_register(reg_name,data)


    def read_register_D8(self, reg_name):
        params = self.read_register(self, reg_name, 1)
        return params[0]

    def read_register_D16(self, reg_name):
        params = self.read_register(self, reg_name, 1)
        return ((params[0] << 8) | params[1])

    def check_magnet_status(self):        
        self.magnet_status = 'nom'
        self.magnet_flag = 0
        val = self.read_register_D8('REG_STATUS')
        if val &  STATUS_MD :
            self.magnet_flag = 1
            self.magnet_status = 'yesm'    
            if val &  STATUS_ML :
                self.magnet_status = 'weak'  
            if val &  STATUS_MH :   
                self.magnet_status = 'strong' 
      
    def get_status(self, eventtime=None):
        return {
                'mstatus': self.magnet_status,
                'val': self.last_angle                
                }


def load_config_prefix(config):
    return Angledcmove(config)
    
