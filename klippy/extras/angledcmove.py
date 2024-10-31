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

#cw_pin:
#ccw_pin:
#counterclockwise

CCW_EN = 1
CW_EN  = 1
CCW_DIS = 0
CW_DIS  = 0


INTER_CMD_TIME = 0.101
DC_MIN_TIME = 0.001
STOP_WAIT_TIME = 0.101
#10ms
#RUN_UNIT_SEC = 0.010
#INTER_WAIT_TIME = 0.100
#8ms
RUN_UNIT_SEC = 0.008
#10ms
INTER_WAIT_TIME = 0.010


class AngDCMotor:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.reactor = self.printer.get_reactor()

        ppins = self.printer.lookup_object('pins')

        #self.cw_last_value = CW_DIS
        self.cw_last_value = 0
        self.cw_pin = None
        cw_pin = config.get('cw_pin', None)
        if cw_pin is not None:
            self.cw_pin = ppins.setup_pin('digital_out', cw_pin)
            self.cw_pin.setup_max_duration(0.)
            self.cw_pin.setup_start_value(self.cw_last_value, 0)

        #self.ccw_last_value = CCW_DIS
        self.ccw_last_value =  0
        self.ccw_pin = None
        ccw_pin = config.get('ccw_pin', None)
        if ccw_pin is not None:
            self.ccw_pin = ppins.setup_pin('digital_out', ccw_pin)
            self.ccw_pin.setup_max_duration(0.)
            self.ccw_pin.setup_start_value(self.ccw_last_value, 0)

        self.mcu = self.ccw_pin.get_mcu()
        cwpin_mcu = self.cw_pin.get_mcu()
        if self.mcu is not cwpin_mcu:
            raise config.error("cw_pin ccw_pin must be on same mcu")
        self.last_run_time = 0

    def cw_move_time_old(self,tm_sec):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.dwell(INTER_CMD_TIME)
        print_time = toolhead.get_last_move_time()
        print_time_old =  self.reactor.monotonic()
        #print_time = max(self.last_run_time + DC_MIN_TIME, print_time)
        self.ccw_pin.set_digital(print_time, 0)
        self.cw_pin.set_digital(print_time,  1)
        #logging.info("\n n:%s old:%s \n" ,print_time,print_time_old)
        toolhead.dwell(tm_sec)
        print_time = toolhead.get_last_move_time()
        #print_time = print_time+tm_sec
        self.ccw_pin.set_digital(print_time, 0)
        self.cw_pin.set_digital(print_time,  0)

        toolhead.dwell(STOP_WAIT_TIME)
        print_time = toolhead.get_last_move_time()

        #print_time = print_time+STOP_WAIT_TIME
        self.ccw_pin.set_digital(print_time, 0)
        self.cw_pin.set_digital(print_time,  0)
        self.last_run_time = print_time

        toolhead.wait_moves()

    def cw_move_time(self,tm_sec, waittmsec = INTER_WAIT_TIME):
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        print_time = print_time + waittmsec
        self.ccw_pin.set_digital(print_time, 0)
        self.cw_pin.set_digital(print_time,  1)
        #logging.info("\n n:%s old:%s \n" ,print_time,print_time_old)
        #toolhead.dwell(tm_sec)
        #print_time = toolhead.get_last_move_time()
        print_time = print_time+tm_sec
        self.ccw_pin.set_digital(print_time, 1)
        self.cw_pin.set_digital(print_time,  1)
        #toolhead.dwell(STOP_WAIT_TIME)
        #print_time = toolhead.get_last_move_time()
        #print_time = print_time+STOP_WAIT_TIME
        print_time = print_time + waittmsec
        self.ccw_pin.set_digital(print_time, 0)
        self.cw_pin.set_digital(print_time,  0)
        self.last_run_time = print_time
        toolhead.dwell(0.001)
        toolhead.wait_moves()
        


    def ccw_move_time(self,tm_sec, waittmsec = INTER_WAIT_TIME):
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        print_time = print_time + waittmsec
        self.ccw_pin.set_digital(print_time, 1)
        self.cw_pin.set_digital(print_time,  0)
        print_time = print_time+tm_sec
        self.ccw_pin.set_digital(print_time, 1)
        self.cw_pin.set_digital(print_time,  1)
        #print_time = print_time+STOP_WAIT_TIME
        print_time = print_time + waittmsec        
        self.ccw_pin.set_digital(print_time, 0)
        self.cw_pin.set_digital(print_time,  0)   
        self.last_run_time = print_time
        toolhead.dwell(0.001)
        toolhead.wait_moves()
        #self.reactor.pause(self.reactor.monotonic() + 0.0001)  


    def dc_set_pin(self, print_time, valuea, valueb):
        self.cw_pin.set_digital(print_time,  valuea)
        self.ccw_pin.set_digital(print_time, valueb)


class Angledcmove:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split(' ')[-1]
        self.reactor = self.printer.get_reactor()
        self.dcmotor = AngDCMotor(config)
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=AS5600_CHIP_ADDR, default_speed=100000)
        self.mcu = self.i2c.get_mcu()
        self.chip_registers = AS5600_REGS
        self.last_angle = 0
        self.magnet_status = 'nom'
        self.magnet_flag = 0
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_mux_command("ANGLE_AS_F", "ANGID", self.name,
                                        self.cmd_AS5600_INIT,
                                        desc=self.cmd_AS5600_INIT_help)

        self.gcode.register_command("ANGLE_LAST_VAL", self.cmd_ANGLE_LAST_VAL)  

        self.gcode.register_command("DCM_MOVE", self.cmd_DCM_MOVE)  
        self.gcode.register_command("DCM_PINOUT", self.cmd_DCM_PINOUT)          


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

    def cmd_DCM_MOVE(self, gcmd):

        mvdir = gcmd.get_int('D',1, minval=0, maxval=2)
        mvtmsec = gcmd.get_float('S', 0.01, minval=0.01, maxval=10)
        waittmsec = gcmd.get_float('W', 0.01, minval=0.01, maxval=10)
        msg = "dcm_move:"
        if mvdir > 0:
            msg = msg + "cw,"
            self.dcmotor.cw_move_time(mvtmsec, waittmsec)
        else:
            msg = msg + "ccw,"
            self.dcmotor.ccw_move_time(mvtmsec, waittmsec)
        msg = msg + "tm=%s wtm=%s " % (mvtmsec,waittmsec)
        #sum = 0
        #self.dcmotor.cw_move_time(RUN_UNIT_SEC)
        #self.dcmotor.ccw_move_time(RUN_UNIT_SEC)
        #for i in range(2):
            #self.dcmotor.cw_move_time(RUN_UNIT_SEC)
            #self.dcmotor.ccw_move_time(RUN_UNIT_SEC)
            #self.reactor.pause(self.reactor.monotonic() +RUN_UNIT_SEC)  
            #self.reactor.pause(self.reactor.monotonic() +RUN_UNIT_SEC) 
            #sum = sum + i
        #msg = "test=%d" % (sum) 
        gcmd.respond_info(msg)    
        pass

    def cmd_DCM_PINOUT(self, gcmd):
        valuea = gcmd.get_int('A',1, minval=0, maxval=2)
        valueb = gcmd.get_int('B',1, minval=0, maxval=2)
        #run slow
        toolhead = self.printer.lookup_object('toolhead')
        #toolhead.register_lookahead_callback(
            #lambda print_time: self.dcmotor._set_pin(print_time, value))
        print_time = toolhead.get_last_move_time()  
        self.dcmotor.dc_set_pin(print_time, valuea, valueb)
        toolhead.dwell(0.001)  # Minimal dwell  
        toolhead.wait_moves()          
        #print_time =  self.reactor.monotonic()+5
        #self.dcmotor._set_pin(print_time, value)    
        #msg = "output=%s" % (print_time) 
        msg = "A=%d B=%d" % (valuea, valueb) 
        gcmd.respond_info(msg)   


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
        params = self.read_register(reg_name, 1)
        return params[0]

    def read_register_D16(self, reg_name):
        params = self.read_register(reg_name, 2)
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
    
