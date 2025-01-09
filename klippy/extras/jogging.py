# BLTouch support
#
# Copyright (C) 2018-2024  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import probe


# joggingrun  "endstop" wrapper
class Joggingrun:
    def __init__(self, config):
        self.printer = config.get_printer()
        # Create an "endstop" object to handle the jog pin
        ppins = self.printer.lookup_object('pins')
        self.mcu_endstop = ppins.setup_pin('endstop', config.get('jog_pin'))
        # Wrappers
        self.get_mcu = self.mcu_endstop.get_mcu
        self.add_stepper = self.mcu_endstop.add_stepper
        self.get_steppers = self.mcu_endstop.get_steppers
        self.home_start = self.mcu_endstop.home_start
        self.home_wait = self.mcu_endstop.home_wait
        self.query_endstop = self.mcu_endstop.query_endstop   

        gcode = self.printer.lookup_object('gcode')
        # QUERY_JOGPIN command
        self.last_state = False
        gcode.register_command('QUERY_JOGPIN', self.cmd_QUERY_JOGPIN,
                               desc=self.cmd_QUERY_JOGPIN_help)        

        self.position_endstop = 0

        gcode.register_command('M286', self.cmd_M286_JOG_XYZ)  
        gcode.register_command('M287', self.cmd_M287_JOG_TRIGG)  
        # Common probe implementation helpers
        #self.cmd_helper = probe.ProbeCommandHelper(
            #config, self, self.mcu_endstop.query_endstop)
        #self.probe_session = probe.ProbeSessionHelper(config, self)
        self.printer.register_event_handler('klippy:mcu_identify',
                                            self._handle_mcu_identify)

    def _handle_mcu_identify(self):
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        for stepper in kin.get_steppers():
            if stepper.is_active_axis('z'):
                #self.mcu_probe.add_stepper(stepper)
                self.add_stepper(stepper)
            if stepper.is_active_axis('x'):
                #self.mcu_probe.add_stepper(stepper)
                self.add_stepper(stepper)
            if stepper.is_active_axis('y'):
                #self.mcu_probe.add_stepper(stepper)
                self.add_stepper(stepper)      
                                         

    def get_position_endstop(self):
        return self.position_endstop


    cmd_QUERY_JOGPIN_help = "Return the status of the z-probe"
    def cmd_QUERY_JOGPIN(self, gcmd):
        if self.query_endstop is None:
            raise gcmd.error("Jog does not support QUERY_JOGPIN")
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        res = self.query_endstop(print_time)
        self.last_state = res
        gcmd.respond_info("probe: %s" % (["open", "TRIGGERED"][not not res],))  

    def probing_move(self, pos, speed):
        phoming = self.printer.lookup_object('homing')
        return phoming.probing_move(self, pos, speed)

    def cmd_M286_JOG_XYZ(self, gcmd):
        axes = []
        for pos, axis in enumerate('XYZ'):
            if gcmd.get(axis, None) is not None:
                axes.append(pos)
        if not axes:
            axes = [2]  
        dist = 0.0
        endval = 0
        drip_speed = 20.0
        params = gcmd.get_command_parameters()
        if 'E' in params:
            endval = int(params['E'])
        if 'D' in params:
            dist = float(params['D']) 
        if 'F' in params:
            drip_speed = float(params['F'])
            if drip_speed <= 0.: 
                drip_speed = 20.0
        dripparam = [dist,endval,drip_speed]
        logging.info("iaxes=%s param=%s\n",axes,dripparam) 

        axis = axes[0]
        godist = dripparam[0]
        if dripparam[1] < 2:
            if  dripparam[1] == 1:
                #godist = position_max
                godist = 99
            else:
                #godist = position_min 
                godist = 8 


        toolhead = self.printer.lookup_object('toolhead')
        #curtime = self.printer.get_reactor().monotonic()
        #if 'z' not in toolhead.get_status(curtime)['homed_axes']:
        #    raise self.printer.command_error("Must home before probe")
        pos = toolhead.get_position()
        pos[axis] = godist
        speedv = dripparam[2]
        logging.info("inparam=%s,%s\n",pos,speedv) 
        retv = self.probing_move(pos,speedv)
        logging.info("probing_move=%s\n",retv) 
        #recpointnull = {}
        #jogrun_state = Homing(self.printer,recpointnull)
        #jogrun_state.set_axes(axes)
        #jogrun_state.set_dripparam(dripparam)
        #kin = self.printer.lookup_object('toolhead').get_kinematics()
        #logging.info("stop kin jogrun_drip=%s\n",axes) 
        #kin.jogrun_drip(jogrun_state) 
        #logging.info("kin jogrun_drip end\n")
    def cmd_M287_JOG_TRIGG(self, gcmd):
        pass                    
 

 
def load_config_prefix(config):
    return Joggingrun(config)  

     
