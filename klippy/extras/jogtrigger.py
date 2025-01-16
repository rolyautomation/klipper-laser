# jogtrigger support
#
# Copyright (C) 2018-2024  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import RPi.GPIO as GPIO


# joggingrun  "endstop" wrapper
class JoggTrigger:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()

        self.gpio_trigpin  = config.getint('trig_pin', 23, minval=0) 
        self.last_outval   = 0  
        self.trig_sw  = config.getint('trig_sw', 1, minval=0) 
        self.chktrigger_timer = self.reactor.register_timer(self.chktrigger_fun)   

        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('TST_JOGTRIGGER', self.cmd_ST_JOGTRIGGER)  

        self.gpio_trigpin_init() 
        self.waitflagjog = 0
        self.chk_interval = 0.002
        self.sumtime = 0 

        self.printer.register_event_handler("homing:homing_move_begin",
                                            self.handle_homing_move_begin)
        self.printer.register_event_handler("homing:homing_move_end",
                                            self.handle_homing_move_end)
        self.printer.register_event_handler("homing:homing_moving",
                                            self.handle_homing_moving) 

        self.printer.register_event_handler("jogging:trigger_to_stop",
                                            self.handle_trigger_to_stop)  

        #by user:self.printer.send_event("jogging:trigger_to_stop", 1)                                                                                                                                  

    def handle_homing_move_begin(self, hmove):
        self.waitflagjog = 0
        self.sumtime = 0
        if self.last_outval < 1:
            self.gpio_trigpin_high_out()            

    def handle_homing_move_end(self, hmove):
        self.waitflagjog = 0        
        if self.last_outval < 1:
            self.gpio_trigpin_high_out()  

    def handle_homing_moving(self, hmove):
        self.waitflagjog = 1 

    def chktrigger_fun(self, eventtime):
        if self.waitflagjog == 0:
            self.sumtime += self.chk_interval
            if self.sumtime > 0.2:
                return self.reactor.NEVER    
            return eventtime + self.chk_interval
        else:
            self.waitflagjog = 0
            if self.last_outval > 0:
                self.gpio_trigpin_low_out() 
            return self.reactor.NEVER 

    def handle_trigger_to_stop(self, num):
        logging.info("rpi trigger_to_stop=%s\n",num) 
        self.reactor.update_timer(self.chktrigger_timer, self.reactor.NOW)

    def gpio_trigpin_init(self):
        if self.trig_sw > 0:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.gpio_trigpin, GPIO.OUT)  
            GPIO.output(self.gpio_trigpin, GPIO.HIGH)
            self.last_outval = 1                       

    def gpio_trigpin_high_out(self):
        if self.trig_sw > 0:        
            GPIO.output(self.gpio_trigpin, GPIO.HIGH)
            self.last_outval = 1  

    def gpio_trigpin_low_out(self):  
        if self.trig_sw > 0:              
            GPIO.output(self.gpio_trigpin, GPIO.LOW)
            self.last_outval = 0  
 
    def cmd_ST_JOGTRIGGER(self, gcmd):
        #pass  
        outvalue = 0                  
        if gcmd.get('H', None) is not None:
            outvalue = 1
        if gcmd.get('L', None) is not None:
            outvalue = 2  
        if  outvalue == 1:
            self.gpio_trigpin_high_out()
        elif  outvalue == 2:
            self.gpio_trigpin_low_out()             
        msg = "hgpio=%s,outval=%s" % (self.gpio_trigpin, self.last_outval) 
        gcmd.respond_info(msg)


def load_config_prefix(config):
    return JoggTrigger(config)  

     
