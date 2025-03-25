# Support fast digital out that are controlled 
# fastdigitalout
# Copyright (C) 2024-2028  jinqiang <jinqiang@ecomedge.io>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#from . import fan
import logging


class FastDigitalOut:

    def __init__(self, config):
   
        self.printer = printer = config.get_printer()
        self.fastout_name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        ppins = printer.lookup_object('pins')
        ctrl_pin_params = ppins.lookup_pin(config.get('ctrl_pin'))
        self._mcu = ctrl_pin_params['chip']
        self._ctrl_pin = ctrl_pin_params['pin']   
        self._oid = self._mcu.create_oid()
        self._mcu.register_config_callback(self._build_config)
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command('TST_FASTDOUT',
                                self.cmd_TST_FASTDOUT)

        self.chktrigger_timer = self.reactor.register_timer(self.chktrigger_fun) 
        self.last_outval = 1
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
    def _build_config(self):
        self.cmd_queue = self._mcu.alloc_command_queue()
        self._mcu.add_config_cmd("config_digital_out oid=%d pin=%s"
                                " value=%d default_value=%d max_duration=%d" % (
                                    self._oid, self._ctrl_pin, 1, 1, 0))   
        cmd_queue = self.cmd_queue
        #self._set_cmd = self._mcu.lookup_command(
        #"queue_digital_out oid=%c clock=%u on_ticks=%u", cq=cmd_queue) 

        self.update_pin_cmd = self._mcu.lookup_command(
            "update_digital_out oid=%c value=%c", cq=self.cmd_queue)  


    def get_mcu(self):
        return self._mcu

    #def update_digital_out_noclock(self, value):
        #clock = self._mcu.get_clock()
        #self.set_cmd.send([self._oid, clock, (not not value)])        
        #self.update_pin_cmd.send([self.oid, not not value])   

    def update_digital_out_noclock(self, value):
        self.last_outval = value      
        self.update_pin_cmd.send([self._oid, not not value]) 

    def gpio_trigpin_high_out(self):
        self.update_digital_out_noclock(1)

    def gpio_trigpin_low_out(self):  
        self.update_digital_out_noclock(0)


    def handle_homing_move_begin(self, hmove):
        self.waitflagjog = 0
        self.sumtime = 0
        if self.last_outval < 1:
            self.gpio_trigpin_high_out()            

    def handle_homing_move_end(self, hmove):
        self.waitflagjog = 0        
        if self.last_outval < 1:
            self.gpio_trigpin_high_out()  
            #pass
            

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
        logging.info("mcu trigger_to_stop=%s\n",num) 
        self.reactor.update_timer(self.chktrigger_timer, self.reactor.NOW)
        #self.gpio_trigpin_low_out() 

    '''
    def update_digital_out(self, value, minclock=0, reqclock=0):
        if self.update_pin_cmd is None:
            # Send setup message via mcu initialization
            self.mcu.add_config_cmd("update_digital_out oid=%c value=%c"
                                    % (self.oid, not not value))
            return
        self.update_pin_cmd.send([self.oid, not not value],
                                 minclock=minclock, reqclock=reqclock)
    '''
    def cmd_TST_FASTDOUT(self, gcmd):
        outvalue = gcmd.get_int('S',0, minval=0, maxval=2)                
        if  outvalue == 1:
            self.update_digital_out_noclock(1)
        elif  outvalue == 0:
            self.update_digital_out_noclock(0)             
        msg = "hgpio=%s,outval=%s,curioval=%s" % (self._ctrl_pin, outvalue, self.last_outval) 
        gcmd.respond_info(msg)



def load_config_prefix(config):
    return FastDigitalOut(config)


    

