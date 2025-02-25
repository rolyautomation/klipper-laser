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
        #self.name = config.get_name().split(' ')[-1]

        self.cur_sellaser = 0
        self.opticalfiber_existf = 0
        self.diode_high = 0
        self.galvo_enabled = 0
 

        self.gcode = self.printer.lookup_object('gcode')

        self.gcode.register_command("QUERY_LASER", self.cmd_QUERY_LASER, desc=self.cmd_QUERY_LASER_help)  
        self.gcode.register_command("TEST_LASER", self.cmd_TEST_LASER)
        self.gcode.register_command('CHG_LASER_TYPE', self.cmd_CHG_LASER_TYPE,
                               desc=self.cmd_CHG_LASER_TYPE_help)  

        self.chg_run_cmd = False                                       
                                              
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect_laserctrl)     

    def _handle_connect_laserctrl(self):
        extruderpwm1 = self.printer.lookup_object('extruderpwm1',None)
        if extruderpwm1 is not None:
            self.opticalfiber_existf = 1    
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


    cmd_CHG_LASER_TYPE_help = "change laser type  blue or red  "
    def cmd_CHG_LASER_TYPE(self, gcmd): 
        mtype = gcmd.get_int('S',0, minval=0, maxval=1)
        if self.opticalfiber_existf == 0 and mtype > 0:
            gcmd.respond_info("optical fiber not exist, can not change laser type")
            return
        if  self.cur_sellaser == mtype:
            gcmd.respond_info("current laser type already:" + str(mtype))
            return

        if mtype > 0:
            if not self.chg_run_cmd:
                self.chg_run_cmd = True
                self.gcode.run_script_from_command(
                    "ACTIVATE_LASER LASER=extruderpwm1\n"
                    "M118 change laser to optical fiber\n"
                    )
                self.chg_run_cmd = False            
        else:
            if not self.chg_run_cmd:
                self.chg_run_cmd = True
                self.gcode.run_script_from_command(
                    "ACTIVATE_LASER LASER=extruderpwm\n"
                    "M118 change laser to blue\n"
                    )
                self.chg_run_cmd = False            
        self.cur_sellaser = mtype
        gcmd.respond_info("current laser type changed to:" + str(mtype))


    def get_diode_high_status(self,eventtime):
        diode_obj = self.printer.lookup_object('output_pin laserpoweren_level')
        if diode_obj is not None:
            diode_high_val = diode_obj.get_status(eventtime)['value']
            self.diode_high = 1 if diode_high_val > 0 else 0
    
    def get_status(self, eventtime=None):
        self.get_diode_high_status(eventtime)
        laser_status = {}
        laser_status['fiber_available'] = self.opticalfiber_existf
        laser_status['fiber_selected'] = self.cur_sellaser
        laser_status['diode_high'] = self.diode_high   
        laser_status['galvo_enabled'] = self.galvo_enabled              
        return dict(laser_status)
        
        
def load_config(config):
    return LaserCtrlInterface(config)

#def load_config_prefix(config):
#   return LaserCtrlInterface(config)
