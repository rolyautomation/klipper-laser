# Support angle dc move
#
# Copyright (C) 2024-2028  jinqiang <jinqiang@ecomedge.io>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import bus
import time
import threading


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
TTURN_TM_SEC_D = 1.65
MIN_RUN_TMUNIT = 0.005
FIND_MAX_TIMES  = 100
FIND_MAX_TIMES_TST  = 1000
APPROACH_TARGET = 0.50
#very important
DC_MOVE_PWM_MINV  = 0.8


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


#4096
class MFindPID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target_val=0.0
        self.actual_val=0.0
        self.err = 0.0
        self.err_last = 0.0
        self.err_next = 0.0 
        self.integral = 0.0       
    def cacl_next_positon_incmode(self, diffval):
        self.err=diffval
        increment_val = self.Kp*(self.err - self.err_next) + self.Ki*self.err + self.Kd*(self.err - 2 * self.err_next + self.err_last)
        self.actual_val += increment_val
        self.err_last = self.err_next
        self.err_next = self.err
        return self.actual_val
    def cacl_next_positon_nmode(self, diffval):
        self.err=diffval
        self.integral+=self.err
        self.actual_val=self.Kp*self.err+self.Ki*self.integral+self.Kd*(self.err-self.err_last)
        self.err_last=self.err
        return self.actual_val

    def restart_init(self):
        self.target_val=0.0
        self.actual_val=0.0
        self.err = 0.0
        self.err_last = 0.0
        self.err_next = 0.0 
        self.integral = 0.0 


MAX_INTEGRAL = 100  
MIN_INTEGRAL = -100 
INTEGER_LIMIT =  20

class ControlPID_MG:
    def __init__(self, Kp, Ki, Kd, Kpup, tstlog_en, max_dcspeed):
        self.max_dcspeed = max_dcspeed
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd   
        self.Kpup = Kpup
        self.tstlog_en = tstlog_en
        self.prev_angle_time = 0.
        self.target_val=0.0
        self.actual_val=0.0
        self.err = 0.0
        self.err_last = 0.0
        self.err_next = 0.0 
        self.integral = 0.0  
        self.kp_sel  = 0


    def angle_update(self, read_time, angle, target_angle):
        time_diff = read_time - self.prev_angle_time
        # Calculate change of temperature
        # Calculate accumulated temperature "error"
        angle_err = target_angle - angle
        self.err= angle_err
        if abs(self.err) < INTEGER_LIMIT:
            self.integral+=self.err
        else:
            self.integral = 0 
        self.integral = max(MIN_INTEGRAL, min(MAX_INTEGRAL, self.integral))
        #if target_angle >= 2000:
        if self.kp_sel > 0:
            #factorP = self.Kp*self.err*1.8
            factorP = self.Kpup*self.err
        else:
            factorP = self.Kp*self.err
        factorI = self.Ki*self.integral
        factorD = self.Kd*(self.err-self.err_last)
        self.actual_val= factorP + factorI + factorD
        if self.tstlog_en:
            logging.info("Factor P=%s I=%s D=%s\n", factorP, factorI, factorD)
        self.err_last=self.err
        return self.actual_val   


    def set_kp_sel(self, kp_sel):        
        self.kp_sel = kp_sel

    def restart_init(self):
        self.target_val=0.0
        self.actual_val=0.0
        self.err = 0.0
        self.err_last = 0.0
        self.err_next = 0.0 
        self.integral = 0.0         
        self.kp_sel  = 0



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
        #self.toolhead = self.printer.lookup_object('toolhead')
        #self.toolhead = None
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


    def cw_move_time_v00(self,tm_sec, waittmsec = INTER_WAIT_TIME, pval=1):
        if self.is_piom:
            self.cw_move_time_pio(tm_sec,waittmsec)
            return 
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        start_time = print_time
        logging.info("s1:%s" ,print_time)
        print_time = print_time + waittmsec
        logging.info("s2:%s" ,print_time)
        if self.is_pwm:
            if self.pwmmodeb:
                self.set_digital_host(self.ccw_pin, print_time, 1)
                self.set_digital_host(self.cw_pin, print_time, 1)              
                print_time = print_time + waittmsec
                logging.info("s31:%s" ,print_time)
                #PWMB MODE 1-pval
                self.set_digital_host(self.ccw_pin, print_time, pval)
                self.set_digital_host(self.cw_pin, print_time, 1)  
            else:
                #PWMA MODE
                logging.info("s3:%s" ,print_time)
                self.set_digital_host(self.ccw_pin, print_time, 0)
                self.set_digital_host(self.cw_pin, print_time, pval)                 


        else:
            self.set_digital_host(self.ccw_pin, print_time, 0)
            self.set_digital_host(self.cw_pin, print_time, 1)                            
        #logging.info("\n n:%s old:%s \n" ,print_time,print_time_old)
        print_time = print_time+tm_sec
        logging.info("s4:%s" ,print_time)
        self.set_digital_host(self.ccw_pin, print_time, 1)
        self.set_digital_host(self.cw_pin, print_time, 1)         
        #print_time = print_time+STOP_WAIT_TIME
        print_time = print_time + waittmsec
        logging.info("s5:%s" ,print_time)
        self.set_digital_host(self.ccw_pin, print_time, 0)
        self.set_digital_host(self.cw_pin, print_time, 0)        
        self.last_run_time = print_time
        end_time = print_time + waittmsec
        #toolhead.dwell(0.001)
        delay_time = end_time-start_time
        logging.info("s7:%s" ,delay_time)
        toolhead.dwell(2*delay_time)
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


    def ccw_move_time_v00(self,tm_sec, waittmsec = INTER_WAIT_TIME, pval=1):
        if self.is_piom:
            self.ccw_move_time_pio(tm_sec,waittmsec)
            return   
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.wait_moves()
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



    def cw_move_time(self,tm_sec, waittmsec = INTER_WAIT_TIME, pval=1):
        if self.is_piom:
            self.cw_move_time_pio(tm_sec,waittmsec)
            return 
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        start_time = print_time
        #logging.info("s1:%s" ,print_time)
        print_time = print_time + waittmsec
        #logging.info("s2:%s" ,print_time)
        if self.is_pwm:
            if self.pwmmodeb:
                #logging.info("s31:%s" ,print_time)
                #PWMB MODE 1-pval
                self.set_digital_host(self.ccw_pin, print_time, pval)
                self.set_digital_host(self.cw_pin, print_time, 1)  
            else:
                #PWMA MODE
                #logging.info("s3:%s" ,print_time)
                self.set_digital_host(self.ccw_pin, print_time, 0)
                self.set_digital_host(self.cw_pin, print_time, pval)                 
        else:
            self.set_digital_host(self.ccw_pin, print_time, 0)
            self.set_digital_host(self.cw_pin, print_time, 1)                            
        #logging.info("\n n:%s old:%s \n" ,print_time,print_time_old)
        print_time = print_time+tm_sec
        #logging.info("s5:%s" ,print_time)
        self.set_digital_host(self.ccw_pin, print_time, 0)
        self.set_digital_host(self.cw_pin, print_time, 0)        
        end_time = print_time + waittmsec
        self.last_run_time = print_time        
        delay_time = end_time-start_time
        #logging.info("s7:%s" ,delay_time)
        toolhead.dwell(delay_time)
        toolhead.wait_moves()
        


    def ccw_move_time(self,tm_sec, waittmsec = INTER_WAIT_TIME, pval=1):
        if self.is_piom:
            self.ccw_move_time_pio(tm_sec,waittmsec)
            return   
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.wait_moves()
        print_time = toolhead.get_last_move_time()
        start_time = print_time
        print_time = print_time + waittmsec
        if self.is_pwm:
            if self.pwmmodeb:
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
        self.set_digital_host(self.ccw_pin, print_time, 0)
        self.set_digital_host(self.cw_pin, print_time, 0)  
        end_time = print_time + waittmsec
        self.last_run_time = print_time  
        delay_time = end_time-start_time                    
        toolhead.dwell(delay_time)
        toolhead.wait_moves()  

    def is_move_complete(self):
        toolhead = self.printer.lookup_object('toolhead')
        current_time = toolhead.get_last_move_time()
        last_time = self.last_run_time
        return current_time >= last_time

                

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
           


    def dc_set_pin_speedmode(self, print_time, valuea, valueb):
        retmode = 0
        if self.is_piom:
            return retmode
        if self.is_pwm:
            self.cw_pin.set_pwm(print_time,  valuea)
            self.ccw_pin.set_pwm(print_time, valueb) 
            retmode = 1
            return retmode           
        else: 
            return retmode

            
class Angledcmove:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split(' ')[-1]
        self.reactor = self.printer.get_reactor()
        self.save_vars = None

        #self.toolhead = self.printer.lookup_object('toolhead')
        self.toolhead = None
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
        self.gcode.register_command("LOAD_ANGLE_CONF", self.cmd_LOAD_ANGLE_CONF)   
        self.gcode.register_command("READ_ANGLE_CONF", self.cmd_AS5600_INIT)                 

        self.gcode.register_command("DCM_MOVE", self.cmd_DCM_MOVE)  
        self.gcode.register_command("DCM_PINOUT", self.cmd_DCM_PINOUT) 
        self.gcode.register_command("DCM_PINOUT_PID", self.cmd_DCM_PINOUT_PID)         
        self.run_stopflag = 0
        self.sample_timer = self.reactor.register_timer(self._sample_as5600) 


        self.gcode.register_command("AS_DEBUG_READ", self.cmd_AS5600_DEBUG_READ)  
        self.gcode.register_command("AS_DEBUG_WRITE", self.cmd_AS5600_DEBUG_WRITE) 
        self.gcode.register_command("SAVE_POS_AS", self.cmd_SAVE_POS_AS) 
        self.gcode.register_command("UPDATE_POS_AS", self.cmd_UPDATE_POS_AS)           
        self.gcode.register_command("LOOK_POS_AS", self.cmd_LOOK_POS_AS) 
        self.gcode.register_command("FIND_POS_BYAS", self.cmd_FIND_POS_BYAS)   
        self.gcode.register_command("FIND_POS_BYAS_SA", self.cmd_FIND_POS_BYAS_SA) 
        self.gcode.register_command("FIND_POS_BYAS_EN", self.cmd_FIND_POS_BYAS_EN) 
        self.gcode.register_command("FIND_POS_BYAS_ST", self.cmd_FIND_POS_BYAS_ST)          
        self.gcode.register_command("SWINGARM_BYAS", self.cmd_SWINGARM_BYAS)  
        self.gcode.register_command("SEL_ALG_FIND", self.cmd_SEL_ALG_FIND)
        self.gcode.register_command("SWINGARM_EN_SW", self.cmd_SWINGARM_EN_SW)        

        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect_init)    


        self.swingarm_en = config.getint('swingarm_en', 1)   
        self.kp = config.getfloat('pid_kp', 0.006)
        self.ki = config.getfloat('pid_ki', 0.0)
        self.kd = config.getfloat('pid_kd', 0.01)
        self.kpup = config.getfloat('pid_kp_up', self.kp)      
        self.toleranceval = config.getint('tolerance', DIFF_VAL)
        self.tstlog_en = config.getint('tstlog_en', 0)              
        self.previous_error = 0
        self.integral = 0
        #self.last_time = time.time()
        self.last_time = 0

        self.sel_alg = 0
        #self.sel_alg = 1
        self.previous_diff = 0
        self.previous_rtm = 0
        #self.normpid = MFindPID(0.31,0.07,0.3)
        #self.normpid = MFindPID(0.28,0.07,0.03)
        self.normpid = MFindPID(0.25,0.07,0.03)
        self.incpid = MFindPID(0.21,0.80,0.01)
        #self.pidreptime = config.getint('pidreptimef',0.02,minval=0.01)
        self.pidreptime = config.getfloat('pidreptimef', 0.02, above=0.01, maxval=2)
        self.samptimer_fpos = self.reactor.register_timer(self.samp_fpos_fun) 
        self.sample_angle_value = 0 
        #self.lock = threading.Lock()  
        self.last_angle_val = 0. 
        self.last_angle_time = 0.

        #self.pid_normal = ControlPID_MG(0.006,0.0,0.01,0.45)   
        self.pid_normal = ControlPID_MG(self.kp,self.ki,self.kd, self.kpup, self.tstlog_en, 0.45)     
        self.force_exit = 0
        self.targer_angle = 0
        self.find_times = 0
        self.pwm_dcwork_flag = 0
        self.pwm_delay = config.getfloat('pwm_delay', 0.005)
        self.stop_delay = config.getfloat('stop_delay', 0.001) 

        #self.min_pwm_plus = 0.10
        #self.max_pwm_plus = 0.40  
        self.min_pwm_plus = config.getfloat('min_pwm_val', 0.1)  
        self.max_pwm_plus = config.getfloat('max_pwm_val', 0.4)       
        self.findpid_work = 0 
        self.finetuningmode = 0  
        self.pwm_chg_status = 0
        self.gcmd = None
        self.dc_abm_running = False
        self.last_valuea = None
        self.last_valueb = None
        self.epsilon = 0.003
        #self.toleranceval = 15
        logging.info("kp=%s ki=%s kd=%s kpup=%s pidreptime=%s ", self.kp, self.ki, self.kd, self.kpup, self.pidreptime)  
        logging.info("toleranceval=%d swingarm_en=%d", self.toleranceval,self.swingarm_en) 
        if self.tstlog_en:
            logging.info("minpwm=%s maxpwm=%s pwm_delay=%s stop_delay=%s \n", self.min_pwm_plus, self.max_pwm_plus, self.pwm_delay, self.stop_delay) 
            
    def cmd_SEL_ALG_FIND(self, gcmd):
        selalgv = gcmd.get_int('S',0, minval=0, maxval=10) 
        if selalgv == 1:
            self.sel_alg = 1
        elif selalgv == 0:
            self.sel_alg = 0 
        msg = "sel alg=%s " % (self.sel_alg, )            
        gcmd.respond_info(msg)  


    def cmd_SWINGARM_EN_SW(self, gcmd):
        swen = gcmd.get_int('S',1, minval=0, maxval=10) 
        if swen == 1:
            self.swingarm_en = swen 
        elif swen == 0:
            self.swingarm_en = swen 
        msg = "swing arm en=%s " % (self.swingarm_en, )            
        gcmd.respond_info(msg)  


    def _handle_connect_init(self):
        self.save_vars = self.printer.lookup_object('save_variables')
        self.load_value_pos()  
        if self.posoffset > 0:
            self.update_pos_offset()
        else:
            logging.info("no set pos offset=%s", self.posoffset)         


    def update_pos_offset(self):   
        #if self.posoffset > 0:
        self.write_register_D16('REG_ZPOS', self.posoffset)  
        logging.info("set pos offset=%s ", self.posoffset)  
        #else:
        #logging.info("no set pos offset=%s", self.posoffset)  


    def cmd_FIND_POS_BYAS(self, gcmd):   
        mpos = gcmd.get_int('M',1, minval=0, maxval=4095)  
        retst = self.find_angle_pos(mpos) 
        msg = "%s=%d" % ("result:",retst)
        gcmd.respond_info(msg)  

    def cmd_FIND_POS_BYAS_SA(self, gcmd):  
        self.gcmd =  gcmd
        mpos = gcmd.get_int('M',1, minval=0, maxval=4095)  
        sel = gcmd.get_int('S',0, minval=0, maxval=1) 
        if self.findpid_work == 0:
            self.set_targer_angle(mpos)
            self.pid_normal.restart_init()
            self.pid_normal.set_kp_sel(sel)
            self.reactor.update_timer(self.samptimer_fpos, self.reactor.NOW)
            msghead = 'start find'
        else:
            msghead =  'busy,find'
        #retst = self.find_angle_pos(mpos) 
        msg = "%s=%d" % (msghead,self.targer_angle)
        gcmd.respond_info(msg) 

    def cmd_FIND_POS_BYAS_EN(self, gcmd):   
        #mpos = gcmd.get_int('M',1, minval=0, maxval=4095)  
        #retst = self.find_angle_pos(mpos) 
        if self.findpid_work > 0:
            self.force_exit = 1
            msghead =  'stop find ='   
        else:
            msghead =  'idle ='   
        msg = "%s=%d" % (msghead,self.targer_angle)
        #msg = "%s=%d" % ("result:",retst)
        gcmd.respond_info(msg) 

    def cmd_FIND_POS_BYAS_ST(self, gcmd):   
        msg = "status:wokr=%d,last=%d,targer=%d" % (self.findpid_work,self.last_angle_val, self.targer_angle)
        gcmd.respond_info(msg) 


    def cmd_SWINGARM_BYAS(self, gcmd): 
        self.gcmd =  gcmd
        if self.swingarm_en == 0:
            logging.info("SWINGARM_BYAS in testmode")  
            return
        if (self.poshead == self.posuse):
            msg = "not do Position verification"
        else:
            pos = gcmd.get_int('P',1, minval=0, maxval=2)
            destpos = self.poshead
            kp_sel = 1
            if pos > 0 :
                destpos = self.posuse
                kp_sel = 0   
            msg = "%s=%d" % ('destp',destpos)  
            if self.findpid_work == 0:
                self.set_targer_angle(destpos)
                self.pid_normal.restart_init()
                self.pid_normal.set_kp_sel(kp_sel)
                self.reactor.update_timer(self.samptimer_fpos, self.reactor.NOW)
                msghead = 'start find ' + "sel=%d" % (kp_sel, )
            else:
                msghead =  'busy,find '            
            #retst = self.find_angle_pos(destpos) 
            msg = msghead + msg + "%s=%d" % ("result:",0)
        gcmd.respond_info(msg)
        
    def cmd_LOOK_POS_AS(self, gcmd):                 
        msg = "%s=%d" % ('poshead',self.poshead)
        msg = msg + ",%s=%d" % ('posuse',self.posuse)
        msg = msg + ",%s=%d" % ('posoffset',self.posoffset)
        gcmd.respond_info(msg)

    def load_value_pos(self):
        self.poshead = self.save_vars.allVariables.get('poshead', 0)        
        self.posuse = self.save_vars.allVariables.get('posuse', 0)  
        self.posoffset = self.save_vars.allVariables.get('posoffset', 0) 

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
        

    def cmd_UPDATE_POS_AS(self, gcmd):
        updateoffsetf = 0
        pos = gcmd.get_int('P',1, minval=0, maxval=2)
        #cur_angle = self.read_register_D16('REG_RAW_ANGLE')
        cur_angle = gcmd.get_int("VAL", 0, minval=0, maxval=4095)  
        pos_str = 'poshead'
        #if pos > 0 :
        if pos ==  1 :
            pos_str = 'posuse'
        elif pos ==  2 :  
            pos_str = 'posoffset'
            updateoffsetf = 1
        self.save_vars.cmd_PROG_VARIABLE(pos_str, repr(cur_angle))
        self.load_value_pos()
        if updateoffsetf > 0:
            self.update_pos_offset()
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

        #self.write_register_D16('REG_ZPOS', 3)
        #self.write_register_D16('REG_MPOS', 0xff)
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


    def cmd_LOAD_ANGLE_CONF(self, gcmd): 
        regind = gcmd.get_int("RIND", 0, minval=0, maxval=3)
        val = gcmd.get_int("VAL", 0, minval=0, maxval=4095)   
        rundo = 0 
        if regind == 1:
            self.write_register_D16('REG_ZPOS', val)   
            rundo = 1  
        elif regind == 2:
            self.write_register_D16('REG_MPOS', val) 
            rundo = 1                 
        elif regind == 3:
            self.write_register_D16('REG_MANG', val) 
            rundo = 1                 
        if rundo > 0:
            gcmd.respond_info("sucess: %s %d" % (regind, val))
        else:
            gcmd.respond_info("error: %s %d" % (regind, val)) 

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

    def stop_dc_move_pwm_v00(self, read_time):
        self.findpid_work = 0
        if self.pwm_dcwork_flag > 0:
            if self.toolhead is None:
                self.toolhead = self.printer.lookup_object('toolhead')    
            self.pwm_dcwork_flag = 0
            self.find_times = 0
            #fast stop 
            valuea = 0
            valueb = 0
            pwm_time = read_time + self.pwm_delay
            print_time_move = self.toolhead.get_last_move_time()  
            print_time = min(pwm_time, print_time_move)            
            self.dcmotor.dc_set_pin_speedmode(print_time, valuea, valueb)
            self.pwm_chg_status = 0 


    def stop_dc_move_pwm(self, read_time):
        self.findpid_work = 0
        if self.pwm_dcwork_flag > 0:
            if self.toolhead is None:
                self.toolhead = self.printer.lookup_object('toolhead')    
            self.pwm_dcwork_flag = 0
            self.find_times = 0
            #fast stop 
            valuea = 0
            valueb = 0
            #pwm_time = read_time + self.pwm_delay
            #print_time_move = self.toolhead.get_last_move_time()  
            #print_time = min(pwm_time, print_time_move)  
            print_time =  read_time+self.stop_delay         
            self.dcmotor.dc_set_pin_speedmode(print_time, valuea, valueb) 
            self.finetuningmode = 0
            self.pwm_chg_status = 0

    def set_targer_angle(self, targer_angle):
        self.targer_angle = targer_angle
        self.finetuningmode = 0
        self.last_valuea = None
        self.last_valueb = None        

    def samp_fpos_fun(self, eventtime):
        if self.toolhead is None:
            self.toolhead = self.printer.lookup_object('toolhead') 
        self.toolhead.wait_moves()    
        print_time_move = self.toolhead.get_last_move_time()    

        self.sample_angle_value = self.read_cur_angle_value()
        self.findpid_work = 1
        measured_time = self.reactor.monotonic()
        print_time = self.i2c.get_mcu().estimated_print_time(measured_time)
        #buffer_time = 0.005  # 5毫秒的缓冲
        #if print_time < print_time_move + buffer_time:
        #    print_time = print_time_move + buffer_time   
             
        if self.force_exit == 0:
            #retst = self._callback(print_time, self.sample_angle_value)
            retst = self.angle_dadjust_callback(print_time, self.sample_angle_value)
            if retst == 0:
                # find contiune
                return measured_time + self.pidreptime 
            else:
                # find success
                # stop pwm
                self.stop_dc_move_pwm(print_time)
                logging.info("finish to stop=%s\n",retst)  
                msg = "finish find:%s=%d" % ("result:",retst)
                self.gcmd.respond_info(msg)      
                return self.reactor.NEVER                    
        else:
            #stop find pos
            #stop pwm
            self.force_exit = 0
            msg = "froce exit find:%s=%d" % ("result:",10)
            self.gcmd.respond_info(msg)             
            self.stop_dc_move_pwm(print_time)
            return self.reactor.NEVER    

    #def setup_callback(self, cb):
    #    self._callback = cb
    
    def set_pwm_co(self, read_time, co_value):
        # co_value to pwm
        # adj_co = co_value/ANGLE_MOD_VAL
        adj_co = co_value
        adj_val = max(self.min_pwm_plus, min(self.max_pwm_plus, abs(adj_co)))
        valuea = valueb = 0.
        if co_value < 0:
            valuea = adj_val
            valueb = 0            
            #valuea = 0
            #valueb = adj_val
        elif co_value > 0:
            valuea = 0
            valueb = adj_val            
            #valuea = adj_val
            #valueb = 0
        else:
            pass
        if self.finetuningmode > 0:
            if self.pwm_chg_status > 0:
                valuea = 0    
                valueb = 0
                #self.pwm_chg_status = 0
                self.pwm_chg_status += 1
                if self.pwm_chg_status > 2 :
                    self.pwm_chg_status = 0    
            else:
                self.pwm_chg_status = 1   

        if self.toolhead is None:
            self.toolhead = self.printer.lookup_object('toolhead')             
        pwm_time = read_time + self.pwm_delay
        print_time_move = self.toolhead.get_last_move_time()  
        print_time = min(pwm_time, print_time_move)  
        valuea = round(valuea, 3)
        valueb = round(valueb, 3)

        values_unchanged = (self.last_valuea is not None and self.last_valueb is not None and
                   abs(self.last_valuea - valuea) < self.epsilon and
                   abs(self.last_valueb - valueb) < self.epsilon)
        if not values_unchanged:
            if self.tstlog_en:
                logging.info("time=%s [%s:%s] \n",print_time, valuea, valueb)         
            self.dcmotor.dc_set_pin_speedmode(print_time, valuea, valueb)
            self.last_valuea = valuea
            self.last_valueb = valueb    
                           
        #if self.tstlog_en:
        #    logging.info("time=%s [%s:%s] \n",print_time, valuea, valueb)         
        #self.dcmotor.dc_set_pin_speedmode(print_time, valuea, valueb)

    def angle_dadjust_callback(self, read_time, angle_val):
        retst = 0
        time_diff = read_time - self.last_angle_time
        self.last_angle_val = angle_val
        self.last_angle_time = read_time 
        retst = self.decide_val_tolerance(angle_val, self.targer_angle)
        if retst > 0 :
        # if retst > 0 and self.pwm_chg_status == 0 and self.finetuningmode > 0:
            if self.tstlog_en:
                logging.info("(%d)info last angle = %s\n", self.find_times, angle_val)
            return  retst
        #if self.find_times > FIND_MAX_TIMES:
        if self.find_times > FIND_MAX_TIMES_TST:
            retst = 2
            return  retst 
        self.pwm_dcwork_flag = 1  
        self.find_times +=1  
        co = self.pid_normal.angle_update(read_time,angle_val,self.targer_angle)  
        if self.tstlog_en:
            logging.info("(%d)info [%s:%s] pidco=%s \n",self.find_times, angle_val, self.targer_angle, co) 
        #if abs(co) < APPROACH_TARGET:
            #retst = 3
            #return  retst 
            #pass
            #self.finetuningmode = 1                       
        self.set_pwm_co(read_time, co)
        retst = 0
        return retst

    def decide_val_tolerance(self, curangle, destangle):
        accept_toler = 0
        diffval = abs(curangle - destangle)
        if diffval < self.toleranceval:
            accept_toler = 1
        diffval_1 =  ANGLE_MOD_VAL - diffval
        if diffval_1 < self.toleranceval:
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
        retst = 0
        if self.sel_alg == 0:
            retst = self.find_angle_pos_v01(destangle)
        else:            
            retst = self.find_angle_pos_pidalg(destangle)
        return  retst            

    def find_angle_pos_v01(self, destangle): 
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



    def control_pid(self, error):
        current_time = time.time()
        delta_time = current_time - self.last_time
        derivative = (error - self.previous_error) / delta_time
        self.integral += error * delta_time

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        self.last_time = current_time
        logging.info("control_pid =[%s %s %s]\n", output,self.last_time,self.previous_error)
        return output

    def angel_transfrom_rtm_pid(self, angle_value):   
        #ret_sec = angle_value/ANGLE_MOD_VAL
        #ret_sec = self.taturn_sec * ret_sec
        ret_sec = angle_value/self.previous_diff
        ret_sec = self.previous_rtm * ret_sec
        if ret_sec  >=  MIN_RUN_TMUNIT:
            self.previous_diff = angle_value
            self.previous_rtm = ret_sec
        if ret_sec < MIN_RUN_TMUNIT:
            ret_sec = MIN_RUN_TMUNIT            
        return ret_sec


    def find_angle_pos_pidalg_v00(self, destangle): 
        successf = 0
        if destangle < 0  or  destangle > ANGLE_IN_MAX :
            successf = 1
            return successf   
        find_times = 0     

        self.previous_error = 0
        self.integral = 0        
        self.last_time = time.time()

        self.previous_diff = ANGLE_MOD_VAL
        #self.previous_rtm = TTURN_TM_SEC
        self.previous_rtm = TTURN_TM_SEC_D
        
        while True:
            cur_angle_value = self.read_cur_angle_value()
            logging.info("pid cur_angle=%d\n" ,cur_angle_value)
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

        
            #runtmsec = self.angel_transfrom_rtm(runangle)
            runtmsec_pid = self.angel_transfrom_rtm_pid(runangle)
            runtmsec = self.control_pid(runtmsec_pid)
            logging.info("%d pid info [%s:%s %s] dir=%d \n",find_times, runangle, runtmsec, runtmsec_pid, CW_DIR)
            runtmsec = runtmsec_pid
            self.dcm_move_correct(runtmsec,CW_DIR)
            self.wait_motionless(0.5)
            find_times = find_times + 1
        logging.info("pid findtimes=%d\n", find_times)

        return successf 

    def find_angle_pos_pidalg(self, destangle): 
        successf = 0
        if destangle < 0  or  destangle > ANGLE_IN_MAX :
            successf = 1
            return successf   
        find_times = 0 
        #mode_pid = 0
        mode_pid = 0
        self.normpid.restart_init() 
        self.incpid.restart_init() 
        if mode_pid > 0:
            logging.info("incpid mode\n") 

        while True:
            cur_angle_value = self.read_cur_angle_value()
            logging.info("pid cur_angle=%d\n" ,cur_angle_value)
            ret = self.decide_val_tolerance(cur_angle_value, destangle)
            if (ret > 0):
                break  
            if find_times > FIND_MAX_TIMES:
                successf = 2
                break             
            diffv_pid = destangle - cur_angle_value
            #isright
            if mode_pid > 0:
                diffv = self.incpid.cacl_next_positon_incmode(diffv_pid)  
            else:
                diffv = self.normpid.cacl_next_positon_nmode(diffv_pid)                   
            logging.info("incpid next =%s, %s\n" ,diffv_pid, diffv)  
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
        st = self.dcmotor.is_move_complete() 
        logging.info("cmd_DCM_MOVE time:%s\n",st)        
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
        if self.findpid_work > 0: 
            logging.info("find busy:%s\n","dcm_move_correct")
            return
        st = self.dcmotor.is_move_complete()  
        logging.info("dcmotor cmd time:%s\n",st)          
        if not self.dc_abm_running:  
            self.dc_abm_running = True  
            waittmsec = 0.01
            #any mode work
            #pvalue = 0.5
            #pvalue = 0.8
            pvalue =  DC_MOVE_PWM_MINV
            #note mbmode relation pvalue = 1
            logging.info("run:%s %s\n","dcm_move_correct",mvdir)
            logging.info("run:%s %s %s\n",mvtmsec, waittmsec, pvalue)
            if mvdir > 0:
                self.dcmotor.cw_move_time(mvtmsec, waittmsec, pvalue)
            else:
                self.dcmotor.ccw_move_time(mvtmsec, waittmsec, pvalue) 
            self.dc_abm_running = False  
        else:
            logging.info("press too fast,please slow")                          


    def dcm_move_cmd(self, cmdval=1, mvdir=0): 
        if self.findpid_work > 0: 
            logging.info("find busy:%s\n","dcm_move_cmd")            
            return 
        logging.info("run:%s %s,%s\n","dcm_move_cmd",mvdir,cmdval)                   
        toolhead = self.printer.lookup_object('toolhead') 
        print_time = toolhead.get_last_move_time()  
        valuea = 0
        valueb = 0        
        if cmdval == 1:
            if mvdir > 0:
                #valuea = 0.6
                valuea = 1
                valueb = 0
                self.dcmotor.dc_set_pin(print_time, valuea, valueb)
            else:
                valuea = 0
                #valueb = 0.6  
                valueb = 1              
                self.dcmotor.dc_set_pin(print_time, valuea, valueb) 
            #new add by 0115    
            toolhead.dwell(0.001)
            toolhead.wait_moves()                 

        else:
            self.dcmotor.dc_set_pin(print_time, valuea, valueb)
            #new add by 0115    
            toolhead.dwell(0.001)
            toolhead.wait_moves()             


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


    def cmd_DCM_PINOUT_PID(self, gcmd):
        #valuea = gcmd.get_int('A',1, minval=0, maxval=2)
        #valueb = gcmd.get_int('B',1, minval=0, maxval=2)
        valuea = gcmd.get_float('A', 1, minval=0., maxval=1.0) 
        valueb = gcmd.get_float('B', 1, minval=0., maxval=1.0)
        #run slow
        toolhead = self.printer.lookup_object('toolhead')
        #toolhead.register_lookahead_callback(
            #lambda print_time: self.dcmotor._set_pin(print_time, value))
        print_time = toolhead.get_last_move_time()  
        self.dcmotor.dc_set_pin_speedmode(print_time, valuea, valueb)
        toolhead.dwell(0.001)  # Minimal dwell  
        toolhead.wait_moves()          
        msg = "A_PID=%s B_PID=%s" % (valuea, valueb) 
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
    
