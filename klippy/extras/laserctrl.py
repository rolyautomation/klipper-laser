# laser controller interface
#
# Copyright (C) 2025-2028  jinqiang <jinqiang@ecomedge.io>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

LASER_T_PWM_BLUE = 0
LASER_T_OPTICAL_RED = 1

class LaserCtrlInterface:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split(' ')[-1]

        self.cur_sellaser = 0
        self.opticalfiber_existf = 0
       
        self.gcode = self.printer.lookup_object('gcode')

        self.gcode.register_command("QUERY_LASER", self.cmd_QUERY_LASER, desc=self.cmd_QUERY_LASER_help)  
        self.gcode.register_command("TEST_LASER", self.cmd_TEST_LASER)
                                              
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect_laserctrl)     

    def _handle_connect_laserctrl(self):
        pass                                                   

    def cmd_TEST_LASER(self, gcmd):
        optical_existf  =  gcmd.get_int('E', 0, minval=0, maxval=1)        
        sel_val  =  gcmd.get_int('S', 0, minval=0, maxval=1) 
        if optical_existf == 0:
            sel_val = 0
        self.opticalfiber_existf = optical_existf
        self.cur_sellaser = sel_val
        gcmd.respond_info("support optical fiber:" + str(self.opticalfiber_existf) + ", current selected laser:" + str(self.cur_sellaser))   

    cmd_QUERY_LASER_help = "Report on the state of laser controller"
    def cmd_QUERY_LASER(self, gcmd):
        gcmd.respond_info("support optical fiber:" + str(self.opticalfiber_existf) + ", current selected laser:" + str(self.cur_sellaser))  

    def get_status(self, eventtime=None):
        laser_status = {}
        laser_status['laser_opticalf'] = self.opticalfiber_existf
        laser_status['laser_cursel'] = self.cur_sellaser
        return dict(laser_status)

def load_config_prefix(config):
    return LaserCtrlInterface(config)
