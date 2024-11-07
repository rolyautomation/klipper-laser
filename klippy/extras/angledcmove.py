# Support angle dc move
#
# Copyright (C) 2024-2028  jinqiang <jinqiang@ecomedge.io>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import bus

REPORT_TIME = .8
AS5600_CHIP_ADDR = 0x36



ANGLE_IN_MIN = 0
ANGLE_IN_MAX = 4095
ANGLE_MOD_VAL = 4096
ANGLE_HF_VAL  = 2048
DIFF_VAL = 2
#TTURN_TM_SEC = 2.8
#TTURN_TM_SEC = 2.73
TTURN_TM_SEC = 2.65
MIN_RUN_TMUNIT = 0.005
FIND_MAX_TIMES  = 100


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


'''
AS5600_REGS = {
    'REG_ZMCO': 0x00, 'REG_ZPOS': 0x01, 'REG_MPOS': 0x03, 'REG_MANG': 0x05,
    'REG_CONF': 0x07, 'REG_RAW_ANGLE': 0x0C, 'REG_ANGLE': 0x0E,
    'REG_STATUS': 0x0B, 'REG_AGC': 0x1A, 'REG_MAGNITUDE': 0x1B, 'REG_BURN': 0xFF
}
'''
AS5600_REGS = {
    'REG_ZMCO': 0x00, 'REG_ZPOS': 0x01, 'REGL_ZPOS': 0x02, 'REG_MPOS': 0x03, 'REGL_MPOS': 0x04,
    'REG_MANG': 0x05, 'REGL_MANG': 0x06, 'REG_CONF': 0x07,  'REGL_CONF': 0x08, 
    'REG_RAW_ANGLE': 0x0C, 'REGL_RAW_ANGLE': 0x0D, 'REG_ANGLE': 0x0E, 'REGL_ANGLE': 0x0F,
    'REG_STATUS': 0x0B, 'REG_AGC': 0x1A, 'REG_MAGNITUDE': 0x1B, 'REGL_MAGNITUDE': 0x1C, 'REG_BURN': 0xFF
}

STATUS_MD = 1 << 5
STATUS_ML = 1 << 4
STATUS_MH = 1 << 3

#cw_pin:A
#ccw_pin:B
#INA=H，INB=L   CW
#INA=L，INB=H   CCW
#PWMA mode, pwm  normal
#PWMB mode, pwm  1 - pwmval

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
MAX_SCHEDULE_TIME = 5.0


M_RM_IDLE  = 3
M_RM_BREAK = 4
#M_RM_IDL_BRK = 5
#M_RM_BRK_IDL = 6
M_RM_AHBL = 5
M_RM_ALBH = 6

M_RM_BRK_RUN = 1
M_RM_IDL_RUN = 2

AB_PIN_CHG  =  1


#M_COTM_US =  1000000
#M_COTM_US =  1000
M_COTM_US =  500000



class MabControl_PIO:
    def __init__(self, config, ab_chg):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.reactor = self.printer.get_reactor()

        self.ab_chg = ab_chg
        ppins = self.printer.lookup_object('pins')

        cw_pin_params = ppins.lookup_pin(config.get('cw_pin'))
        ccw_pin_params = ppins.lookup_pin(config.get('ccw_pin'))     

        mcu = cw_pin_params['chip']
        if mcu is not ccw_pin_params['chip']:
            raise config.error("Mab_pio pins must be on same mcu")  

        self._mcu = mcu
        self._cw_pin = cw_pin_params['pin']    
        self._ccw_pin = ccw_pin_params['pin']  

        self._oid = self._mcu.create_oid()
        self._mcu.register_config_callback(self._build_config)

        self._last_clock = 0  

    def _build_config(self):

        self._mcu.add_config_cmd("config_ab_dcmotor oid=%d ab_min_pin=%s wkm=%u"
                                        % (self._oid, self._cw_pin, self.ab_chg))  

        self._cmd_queue = self._mcu.alloc_command_queue()

        self._pio_instr_cmds_cmd = self._mcu.lookup_command(
            "rinstr_ab_dcmotor oid=%c rtm=%u wtm=%u dir=%c wmod=%c", cq=self._cmd_queue)                                                                                                    
                                         
    def get_mcu(self):
        return self._mcu

    def pio_instr_send(self, print_time, runtm, waittm, vdir, vwmod):
        clock = self._mcu.print_time_to_clock(print_time)
        self._pio_instr_cmds_cmd.send([self._oid, runtm, waittm, vdir, vwmod],
                           minclock=self._last_clock, reqclock=clock)
        self._last_clock = clock   

    def pio_instr_cw(self, print_time, runtm, pmode=0):
        waittm = 1
        vdir = 1
        runtmus = int(runtm*M_COTM_US+0.5)
        vwmod = M_RM_IDL_RUN
        if pmode > 0 :
            vwmod = M_RM_BRK_RUN
        self.pio_instr_send(print_time, runtmus, waittm, vdir, vwmod)    

    def pio_instr_ccw(self, print_time, runtm, pmode=0):
        waittm = 1
        vdir = 0
        runtmus = int(runtm*M_COTM_US+0.5)
        vwmod = M_RM_IDL_RUN
        if pmode > 0 :
            vwmod = M_RM_BRK_RUN
        self.pio_instr_send(print_time, runtmus, waittm, vdir, vwmod)  

    def pio_instr_idle(self, print_time, runtmus=1):
        waittm = 1
        vdir = 0
        vwmod = M_RM_IDLE
        self.pio_instr_send(print_time, runtmus, waittm, vdir, vwmod)  

    def pio_instr_break(self, print_time, runtmus=1):
        waittm = 1
        vdir = 0
        vwmod = M_RM_BREAK
        self.pio_instr_send(print_time, runtmus, waittm, vdir, vwmod)  

    def pio_instr_xmod(self, print_time, xmod=M_RM_IDLE, runtmus=1):
        waittm = 1
        vdir = 0
        vwmod = xmod
        self.pio_instr_send(print_time, runtmus, waittm, vdir, vwmod) 


class AngDCMotor:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.reactor = self.printer.get_reactor()

        ppins = self.printer.lookup_object('pins')
        self.cw_last_value = 0
        self.ccw_last_value =  0

        self.is_piom = config.getboolean('pio_mode', False)
        ab_chg  = config.getint('ab_chg', 0, minval=0)
        self.pio_rmodeb = config.getint('prmode', 0, minval=0)

        if self.is_piom:
            #self.pio_rmodeb = config.getint('prmode', 0, minval=0)
            self.pio_dcm = MabControl_PIO(config, ab_chg)
            return  

        self.is_pwm = config.getboolean('pwm', False)
        if self.is_pwm:
            self.cw_pin = ppins.setup_pin('pwm', config.get('cw_pin'))
            cycle_time = config.getfloat('cycle_time', 0.100, above=0.,
                                         maxval=MAX_SCHEDULE_TIME)
            hardware_pwm = config.getboolean('hardware_pwm', False)
            self.cw_pin.setup_cycle_time(cycle_time, hardware_pwm)
            self.scale = config.getfloat('scale', 1., above=0.)

            self.ccw_pin = ppins.setup_pin('pwm', config.get('ccw_pin'))
            self.ccw_pin.setup_cycle_time(cycle_time, hardware_pwm)
            self.pwmmodeb = config.getboolean('pwm_mb', False)
        
        else:    
            #self.cw_last_value = CW_DIS
            self.scale = 1.
            self.cw_pin = None
            cw_pin = config.get('cw_pin', None)
            if cw_pin is not None:
                self.cw_pin = ppins.setup_pin('digital_out', cw_pin)

            #self.ccw_last_value = CCW_DIS
            self.ccw_pin = None
            ccw_pin = config.get('ccw_pin', None)
            if ccw_pin is not None:
                self.ccw_pin = ppins.setup_pin('digital_out', ccw_pin)

            #self.mcu = self.ccw_pin.get_mcu()
            #cwpin_mcu = self.cw_pin.get_mcu()
            #if self.mcu is not cwpin_mcu:
                #raise config.error("cw_pin ccw_pin must be on same mcu")
        if self.cw_pin is not None:                
            self.cw_pin.setup_max_duration(0.)
            self.cw_pin.setup_start_value(self.cw_last_value, 0) 
        if self.ccw_pin is not None:               
            self.ccw_pin.setup_max_duration(0.)
            self.ccw_pin.setup_start_value(self.ccw_last_value, 0)                      
        self.last_run_time = 0

    def set_digital_host(self, pinc, print_time, value):  
        if self.is_pwm:
            pinc.set_pwm(print_time, value)
        else:
            pinc.set_digital(print_time, value)         

    def cw_move_time_pio(self,tm_sec, waittmsec = INTER_WAIT_TIME):
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        print_time = print_time + waittmsec
        if (self.pio_rmodeb > 0):
            self.pio_dcm.pio_instr_xmod(print_time, M_RM_BREAK) 
            print_time = print_time + waittmsec
            self.pio_dcm.pio_instr_cw(print_time, tm_sec, 1)
            print_time = print_time+tm_sec + waittmsec
            self.pio_dcm.pio_instr_xmod(print_time, M_RM_IDLE)
        else:
            self.pio_dcm.pio_instr_xmod(print_time, M_RM_IDLE) 
            print_time = print_time + waittmsec
            self.pio_dcm.pio_instr_cw(print_time, tm_sec, 0)                
            print_time = print_time+tm_sec + waittmsec
            self.pio_dcm.pio_instr_xmod(print_time, M_RM_IDLE)

        self.last_run_time = print_time
        toolhead.dwell(0.001)
        toolhead.wait_moves()            


    def cw_move_time(self,tm_sec, waittmsec = INTER_WAIT_TIME, pval=1):
        if self.is_piom:
            self.cw_move_time_pio(tm_sec,waittmsec)
            return 
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        print_time = print_time + waittmsec
        if self.is_pwm:
            if self.pwmmodeb:
                self.set_digital_host(self.ccw_pin, print_time, 1)
                self.set_digital_host(self.cw_pin, print_time, 1)              
                print_time = print_time + waittmsec
                #PWMB MODE 1-pval
                self.set_digital_host(self.ccw_pin, print_time, pval)
                self.set_digital_host(self.cw_pin, print_time, 1)  
            else:
                #PWMA MODE
                self.set_digital_host(self.ccw_pin, print_time, 0)
                self.set_digital_host(self.cw_pin, print_time, pval)                 


        else:
            self.set_digital_host(self.ccw_pin, print_time, 0)
            self.set_digital_host(self.cw_pin, print_time, 1)                            
        #logging.info("\n n:%s old:%s \n" ,print_time,print_time_old)
        print_time = print_time+tm_sec
        self.set_digital_host(self.ccw_pin, print_time, 1)
        self.set_digital_host(self.cw_pin, print_time, 1)         
        #print_time = print_time+STOP_WAIT_TIME
        print_time = print_time + waittmsec
        self.set_digital_host(self.ccw_pin, print_time, 0)
        self.set_digital_host(self.cw_pin, print_time, 0)        
        self.last_run_time = print_time
        toolhead.dwell(0.001)
        toolhead.wait_moves()

    def ccw_move_time_pio(self,tm_sec, waittmsec = INTER_WAIT_TIME):  
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        print_time = print_time + waittmsec
        if (self.pio_rmodeb > 0):
            self.pio_dcm.pio_instr_xmod(print_time, M_RM_BREAK) 
            print_time = print_time + waittmsec
            self.pio_dcm.pio_instr_ccw(print_time, tm_sec, 1)
            print_time = print_time+tm_sec + waittmsec
            self.pio_dcm.pio_instr_xmod(print_time, M_RM_IDLE)
        else:
            self.pio_dcm.pio_instr_xmod(print_time, M_RM_IDLE) 
            print_time = print_time + waittmsec
            self.pio_dcm.pio_instr_ccw(print_time, tm_sec, 0)                
            print_time = print_time+tm_sec + waittmsec
            self.pio_dcm.pio_instr_xmod(print_time, M_RM_IDLE)

        self.last_run_time = print_time
        toolhead.dwell(0.001)
        toolhead.wait_moves()           


    def ccw_move_time(self,tm_sec, waittmsec = INTER_WAIT_TIME, pval=1):
        if self.is_piom:
            self.ccw_move_time_pio(tm_sec,waittmsec)
            return         
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        print_time = print_time + waittmsec
        if self.is_pwm:
            if self.pwmmodeb:
                self.set_digital_host(self.ccw_pin, print_time, 1)
                self.set_digital_host(self.cw_pin, print_time, 1)             
                print_time = print_time + waittmsec
                #PWMB MODE 1-pval
                self.set_digital_host(self.ccw_pin, print_time, 1)
                self.set_digital_host(self.cw_pin, print_time, pval)
            else:
                #PWMA MODE
                self.set_digital_host(self.ccw_pin, print_time, pval)
                self.set_digital_host(self.cw_pin, print_time, 0)                 
                           
        else:    
            self.set_digital_host(self.ccw_pin, print_time, 1)
            self.set_digital_host(self.cw_pin, print_time, 0)             
        print_time = print_time+tm_sec
        self.set_digital_host(self.ccw_pin, print_time, 1)
        self.set_digital_host(self.cw_pin, print_time, 1)         
        #print_time = print_time+STOP_WAIT_TIME
        print_time = print_time + waittmsec   
        self.set_digital_host(self.ccw_pin, print_time, 0)
        self.set_digital_host(self.cw_pin, print_time, 0)               
        self.last_run_time = print_time
        toolhead.dwell(0.001)
        toolhead.wait_moves()


    def dc_set_pin(self, print_time, valuea, valueb):
        if self.is_piom:
            if  valuea and valueb:
                self.pio_dcm.pio_instr_xmod(print_time, M_RM_BREAK) 
                #logging.info("\n M_RM_BREAK \n")
            elif  not valuea and not valueb:          
                self.pio_dcm.pio_instr_xmod(print_time, M_RM_IDLE) 
                #logging.info("\n M_RM_IDLE \n")
            elif  valuea and not valueb:
                self.pio_dcm.pio_instr_xmod(print_time, M_RM_AHBL)
                #logging.info("\n M_RM_AHBL \n")                
            else: 
                self.pio_dcm.pio_instr_xmod(print_time, M_RM_ALBH)
                #logging.info("\n M_RM_ALBH \n")                 
            return  
        if self.is_pwm:
            self.cw_pin.set_pwm(print_time,  valuea)
            self.ccw_pin.set_pwm(print_time, valueb)            
        else: 
            if valuea:
                valuea = 1
            if valueb:
                valueb = 1  
            self.cw_pin.set_digital(print_time,  valuea)
            self.ccw_pin.set_digital(print_time, valueb)

class Angledcmove:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split(' ')[-1]
        self.reactor = self.printer.get_reactor()
        self.save_vars = None
        
        self.dcmotor = AngDCMotor(config)
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=AS5600_CHIP_ADDR, default_speed=100000)
        self.mcu = self.i2c.get_mcu()

        self.taturn_sec = TTURN_TM_SEC

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
        self.run_stopflag = 0
        self.sample_timer = self.reactor.register_timer(self._sample_as5600) 


        self.gcode.register_command("AS_DEBUG_READ", self.cmd_AS5600_DEBUG_READ)  
        self.gcode.register_command("AS_DEBUG_WRITE", self.cmd_AS5600_DEBUG_WRITE) 
        self.gcode.register_command("SAVE_POS_AS", self.cmd_SAVE_POS_AS)  
        self.gcode.register_command("LOOK_POS_AS", self.cmd_LOOK_POS_AS) 
        self.gcode.register_command("FIND_POS_BYAS", self.cmd_FIND_POS_BYAS)    
        self.gcode.register_command("SWINGARM_BYAS", self.cmd_SWINGARM_BYAS)  

        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect_init)        
    def _handle_connect_init(self):
        self.save_vars = self.printer.lookup_object('save_variables')
        self.load_value_pos()   

    def cmd_FIND_POS_BYAS(self, gcmd):   
        mpos = gcmd.get_int('M',1, minval=0, maxval=4095)  
        retst = self.find_angle_pos(mpos) 
        msg = "%s=%d" % ("result:",retst)
        gcmd.respond_info(msg)  

    def cmd_SWINGARM_BYAS(self, gcmd): 
        if (self.poshead == self.posuse):
            msg = "not do Position verification"
        else:
            pos = gcmd.get_int('P',1, minval=0, maxval=2)
            destpos = self.poshead
            if pos > 0 :
                destpos = self.posuse
            msg = "%s=%d" % ('destp',destpos)  
            retst = self.find_angle_pos(destpos) 
            msg = msg + "%s=%d" % ("result:",retst)
        gcmd.respond_info(msg)

    def cmd_LOOK_POS_AS(self, gcmd):                 
        msg = "%s=%d" % ('poshead',self.poshead)
        msg = msg + ",%s=%d" % ('posuse',self.posuse)
        gcmd.respond_info(msg)

    def load_value_pos(self):
        self.poshead = self.save_vars.allVariables.get('poshead', 0)        
        self.posuse = self.save_vars.allVariables.get('posuse', 0)  

    def cmd_SAVE_POS_AS(self, gcmd):
        pos = gcmd.get_int('P',1, minval=0, maxval=2)
        cur_angle = self.read_register_D16('REG_RAW_ANGLE')
        pos_str = 'poshead'
        if pos > 0 :
            pos_str = 'posuse'
        self.save_vars.cmd_PROG_VARIABLE(pos_str, repr(cur_angle))
        self.load_value_pos()
        msg = "%s=%d" % (pos_str,cur_angle)
        gcmd.respond_info(msg)

    cmd_AS5600_DEBUG_READ_help = "Query register (for debugging)"
    def cmd_AS5600_DEBUG_READ(self, gcmd):
        regv = gcmd.get("REG", minval=0, maxval=255, parser=lambda x: int(x, 0))
        lregn = [k for k, v in self.chip_registers.items() if v == regv]
        regn = lregn[0] if len(lregn) > 0 else None
        if regn is None: 
            gcmd.respond_info("AS5600 REG[0x%x] address error" % (regv))
            return
        reg_address =  self.chip_registers[regn]
        if reg_address != regv :
            gcmd.respond_info("AS5600 REG[0x%x] != %s" % (regv, regn))
            return
        val = self.read_register_D8(regn)
        gcmd.respond_info("AS5600 REG[0x%x] = 0x%x" % (regv, val))


    cmd_AS5600_DEBUG_WRITE_help = "Set register (for debugging)"
    def cmd_AS5600_DEBUG_WRITE(self, gcmd):
        regv = gcmd.get("REG", minval=0, maxval=255, parser=lambda x: int(x, 0))
        lregn = [k for k, v in self.chip_registers.items() if v == regv]
        regn = lregn[0] if len(lregn) > 0 else None
        if regn is None: 
            gcmd.respond_info("AS5600 REG[0x%x] address error" % (regv))
            return
        reg_address =  self.chip_registers[regn]
        if reg_address != regv :
            gcmd.respond_info("AS5600 REG[0x%x] != %s" % (regv, regn))
            return

        val = gcmd.get("VAL", minval=0, maxval=255, parser=lambda x: int(x, 0))
        self.write_register(regn,val)
        gcmd.respond_info("AS5600 WREG[0x%x] = 0x%x" % (regv, val))  

        #self.chip.set_reg(reg, val)  
                                
    def _sample_as5600(self, eventtime):
        self.check_magnet_status()
        last_raw_angle = self.read_register_D16('REG_RAW_ANGLE')
        last_filter_angle = self.read_register_D16('REG_ANGLE')
        hex_str1 = hex(last_raw_angle)
        hex_str2 = hex(last_filter_angle)            
        msg = "%s:REG_RAW_ANGLE=%s:%d" % (self.magnet_status,hex_str,self.last_angle)
        return self.reactor.NEVER  


    cmd_AS5600_INIT_help = "Report on the state of as5600"
    def cmd_AS5600_INIT(self, gcmd):

        self.write_register_D16('REG_ZPOS', 3)
        self.write_register_D16('REG_MPOS', 0xff)
        #self.write_register_D16('REG_MANG', 4095)

        self.cur_zpos = self.read_register_D16('REG_ZPOS')
        self.cur_mpos = self.read_register_D16('REG_MPOS')
        self.cur_conf = self.read_register_D16('REG_CONF') 
        self.cur_magn = self.read_register_D16('REG_MAGNITUDE') 
        self.cur_vmang = self.read_register_D16('REG_MANG')

        msg = "as5600:"
        msg = msg + "ZPOS=" + hex(self.cur_zpos) + ","
        msg = msg + "MPOS=" + hex(self.cur_mpos) + ","
        msg = msg + "CONF=" + hex(self.cur_conf) + ","   
        msg = msg + "MAGN=" + hex(self.cur_magn) + ","  
        msg = msg + "MANG=" + hex(self.cur_vmang) + "," 
        gcmd.respond_info(self.name + ":" + msg)

    def cmd_ANGLE_LAST_VAL(self, gcmd):
        self.check_magnet_status()
        self.last_angle = self.read_register_D16('REG_RAW_ANGLE')
        last_filter_angle = self.read_register_D16('REG_ANGLE')
        hex_str = hex(self.last_angle)
        hex_str2 = hex(last_filter_angle) 
        msg = "%s:REG_RAW_ANGLE=%s:%d" % (self.magnet_status,hex_str,self.last_angle)
        msg = msg + "RAW_ANGLE=%s:%d" % (hex_str2,last_filter_angle)
        gcmd.respond_info(msg)
        #logging.info("\n %s\n" ,msg)

    def read_cur_angle_value(self):  
        #cur_value = self.read_register_D16('REG_RAW_ANGLE')
        cur_value = self.read_register_D16('REG_ANGLE')
        return  cur_value  


    def decide_val_tolerance(self, curangle, destangle):
        accept_toler = 0
        diffval = abs(curangle - destangle)
        if diffval < DIFF_VAL:
            accept_toler = 1
        diffval_1 =  ANGLE_MOD_VAL - diffval
        if diffval_1 < DIFF_VAL:
            accept_toler = 1
        return accept_toler
        
    def angel_transfrom_rtm(self, angle_value):   
        ret_sec = angle_value/ANGLE_MOD_VAL
        ret_sec = self.taturn_sec * ret_sec
        if ret_sec < MIN_RUN_TMUNIT:
            ret_sec = MIN_RUN_TMUNIT            
        return ret_sec

    def wait_motionless(self, tmval = 0.001):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.dwell(tmval) 
        toolhead.wait_moves()  

    def find_angle_pos(self, destangle): 
        successf = 0
        if destangle < 0  or  destangle > ANGLE_IN_MAX :
            successf = 1
            return successf   
        find_times = 0     
        #cur_angle_value = self.read_cur_angle_value()
        #ret = self.decide_val_tolerance(cur_angle_value, destangle)
        #if (ret > 0)
            #return successf
        while True:
            cur_angle_value = self.read_cur_angle_value()
            logging.info("cur_angle=%d\n" ,cur_angle_value)
            ret = self.decide_val_tolerance(cur_angle_value, destangle)
            if (ret > 0):
                break  
            if find_times > FIND_MAX_TIMES:
                successf = 2
                break             
            diffv = destangle - cur_angle_value
            abs_diffv = abs(diffv)
            CW_DIR = 0
            runangle = 0
            if abs_diffv  <  ANGLE_HF_VAL:
                runangle = abs_diffv  
                if  diffv > 0:
                    CW_DIR = 1
                else:
                    CW_DIR = 0
            else:
                runangle = ANGLE_MOD_VAL-abs_diffv 
                if  diffv > 0:
                    CW_DIR = 0
                else:
                    CW_DIR = 1
            runtmsec = self.angel_transfrom_rtm(runangle)
            logging.info("%dinfo [%s:%s] dir=%d \n",find_times, runangle, runtmsec, CW_DIR)
            self.dcm_move_correct(runtmsec,CW_DIR)
            self.wait_motionless(0.5)
            find_times = find_times + 1
        logging.info("findtimes=%d\n", find_times)

        return successf



    def cmd_DCM_MOVE(self, gcmd):

        mvdir = gcmd.get_int('D',1, minval=0, maxval=2)
        #mvtmsec = gcmd.get_float('S', 0.01, minval=0.01, maxval=10)
        mvtmsec = gcmd.get_float('S', 0.01, minval=0., maxval=20)
        waittmsec = gcmd.get_float('W', 0.01, minval=0.01, maxval=10)
        pvalue = gcmd.get_float('P', 1, minval=0., maxval=1.0)        
        msg = "dcm_move:"
        if mvdir > 0:
            msg = msg + "cw,"
            self.dcmotor.cw_move_time(mvtmsec, waittmsec, pvalue)
        else:
            msg = msg + "ccw,"
            self.dcmotor.ccw_move_time(mvtmsec, waittmsec, pvalue)
        msg = msg + "tm=%s wtm=%s pv=%s" % (mvtmsec, waittmsec, pvalue)
        gcmd.respond_info(msg)    

    def dcm_move_correct(self, mvtmsec=0.01, mvdir=0):  
        waittmsec = 0.01
        pvalue = 0.5
        if mvdir > 0:
            self.dcmotor.cw_move_time(mvtmsec, waittmsec, pvalue)
        else:
            self.dcmotor.ccw_move_time(mvtmsec, waittmsec, pvalue)    


    def cmd_DCM_PINOUT(self, gcmd):
        #valuea = gcmd.get_int('A',1, minval=0, maxval=2)
        #valueb = gcmd.get_int('B',1, minval=0, maxval=2)
        valuea = gcmd.get_float('A', 1, minval=0., maxval=1.0) 
        valueb = gcmd.get_float('B', 1, minval=0., maxval=1.0)
        #run slow
        toolhead = self.printer.lookup_object('toolhead')
        #toolhead.register_lookahead_callback(
            #lambda print_time: self.dcmotor._set_pin(print_time, value))
        print_time = toolhead.get_last_move_time()  
        self.dcmotor.dc_set_pin(print_time, valuea, valueb)
        toolhead.dwell(0.001)  # Minimal dwell  
        toolhead.wait_moves()          
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
    
