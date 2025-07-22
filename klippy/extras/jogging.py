# jogging support
#
# Copyright (C) 2018-2024  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
#from . import probe
#import RPi.GPIO as GPIO
SPEED_MIN_LIMIT = 5
SPEED_MAX_LIMIT = 300

# joggingrun  "endstop" wrapper
class Joggingrun:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
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
        self.epsilon = 0.01  

        gcode = self.printer.lookup_object('gcode')
        # QUERY_JOGPIN command
        self.last_state = False
        gcode.register_command('QUERY_JOGPIN', self.cmd_QUERY_JOGPIN,
                               desc=self.cmd_QUERY_JOGPIN_help)        

        self.position_endstop = 0
         
        self.min_speed  = config.getint('min_speed', SPEED_MIN_LIMIT, minval=1)  
        self.max_speed  = config.getint('max_speed', SPEED_MAX_LIMIT, minval=5) 
        self.xmin_axes  = config.getint('xmin_axes', 0, minval=0)   
        self.ymin_axes  = config.getint('ymin_axes', 0, minval=0)   
        self.zmin_axes  = config.getint('zmin_axes', 0, minval=0)   

        wh = self.printer.lookup_object('webhooks')
        wh.register_endpoint("gcode/jogcancel", self._handle_jogcancel)  
        self.jogworkmode = 0 
        self.gcode = self.printer.lookup_object('gcode')
        wh.register_endpoint("gcode/jogmove", self._handle_jogmove)  

        
        self.jog_starttimer = self.reactor.register_timer(self.jog_start_fun)   
        self.jog_runcmdstr = None

        self.machinepos = None 
        gcode.register_command('M286', self.cmd_M286_JOG_XYZ)  
        gcode.register_command('LOOKMACHRANGE', self.cmd_LOOKMACHRANGE)  
        # Common probe implementation helpers
        #self.cmd_helper = probe.ProbeCommandHelper(
            #config, self, self.mcu_endstop.query_endstop)
        #self.probe_session = probe.ProbeSessionHelper(config, self)
        self.printer.register_event_handler('klippy:mcu_identify',
                                            self._handle_mcu_identify)

        self.printer.register_event_handler("homing:homing_move_end",
                                            self.handle_homing_move_end)

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


    def filter_Z_ZAcomd(self, gcode_line):
        has_z = 'Z' in gcode_line
        has_a = 'A' in gcode_line
        if  has_z and has_a:
            #parts = gcode_line.split()
            #has_z = any('Z' in part for part in parts)
            #has_a = any('A' in part for part in parts)
            parts = gcode_line.split()
            new_parts = []
            for part in parts:
                if part.startswith('Z'):
                    continue
                new_parts.append(part)
            return ' '.join(new_parts)
        return gcode_line  

    #Continuous jog    
    def is_continue_jog(self, gcode_line):
        if 'G90' not in gcode_line:
            return False
        has_x = 'X' in gcode_line
        has_y = 'Y' in gcode_line
        if (has_x and not has_y) or (has_y and not has_x):
            return True
        return False   


    def parse_jog_command(self, commandstr): 
        iret = 0  
        iststr = 'ok'
        # Convert commandstr to uppercase
        logging.info("inputjog=%s",commandstr) 
        commandstr = commandstr.upper()
        if commandstr.startswith('$J='):
            gcodestr = commandstr[3:].strip()
            gcodestr = self.filter_Z_ZAcomd(gcodestr)
            cjogf = self.is_continue_jog(gcodestr)
            parts = gcodestr.split()
            result = []
            coords = []
            for part in parts:
                if part.startswith('G'):
                    result.append(part)
                else:
                    coords.append(part)
            iret = 1        
            if coords : 
                ginstr = ' '.join(coords)  
                if not cjogf:
                    result.append(f"G0 {ginstr}")
                else:
                    result.append(f"M286 {ginstr}")
                    iret = 2

            if result :  
                iststr = '\n'.join(result)
 
        return iret, iststr


    def _handle_jogmove(self, web_request):
        istcode,irunstr = self.parse_jog_command(web_request.get_str('script'))
        if istcode > 0:
            try:
                if istcode == 1:
                #if istcode > 0:
                    self.gcode.run_script(irunstr) 
                else:
                    self.jog_runcmdstr = irunstr
                    self.reactor.update_timer(self.jog_starttimer, self.reactor.NOW)
                    logging.info("jog_starttimer") 
            except Exception as e:
                self.gcode.respond_info("info: %s" % str(e))


    def jog_start_fun(self, eventtime):                
        if self.jog_runcmdstr is not None:
            try:
                logging.info("jog_start_fun start") 
                self.gcode.run_script(self.jog_runcmdstr) 
                logging.info("jog_start_fun end") 
            except Exception as e:
                self.gcode.respond_info("info: %s" % str(e))
            self.jog_runcmdstr = None
        return self.reactor.NEVER



    def _handle_jogcancel(self, web_request):
        if self.jogworkmode > 0:                
            self.printer.send_event("jogging:trigger_to_stop", 0)    



    def handle_homing_move_end(self, hmove):
        self.jogworkmode = 0     
         
   
    def get_position_endstop(self):
        return self.position_endstop

    def check_is_move(self, curpos, gopos, mdir):
        #logging.info("check_is_move")    
        ismove = True
        if mdir == 0 and curpos <= gopos:
            ismove = False
        elif mdir == 1 and curpos >= gopos:
            ismove = False
        if abs(curpos - gopos) < self.epsilon:
            ismove = False
        return ismove

    def speedunit_min_sec(self, axis, speed_min):
        speed_factor = 1. / 60.
        speed_sec =  speed_min   
        if axis < 2:
            speed_sec = speed_min * speed_factor
        return speed_sec    

        
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
        #return phoming.probing_move(self, pos, speed)
        return phoming.probing_move_jog(self, pos, speed)
        


    def cmd_M286_JOG_XYZ(self, gcmd):
        axes = []
        for pos, axis in enumerate('XYZ'):
            if gcmd.get(axis, None) is not None:
                axes.append(pos)
        if not axes:
            axes = [2]  
        dist = 0.0
        endval = 0

        #drip_speed = 20.0
        drip_speed = self.min_speed
        params = gcmd.get_command_parameters()

        if 'X' in params:
            logging.info("INX=%s",params['X']) 
            x_str = params['X']
            if not x_str:  # Check if empty string
                xval = 0.0  # Default to 0 if empty
            else:
                xval = float(x_str)
            endval = 0
            if xval > 1:
                endval = 1       
        if 'Y' in params:
            logging.info("INY=%s",params['Y'])             
            #yval = int(params['Y']) 
            y_str = params['Y']
            if not y_str:  # Check if empty string
                yval = 0.0  # Default to 0 if empty
            else:
                yval = float(y_str)
            endval = 0
            if yval > 1:
                endval = 1 
 
        if 'E' in params:
            endval = int(params['E'])
        if 'D' in params:
            dist = float(params['D']) 
        if 'F' in params:
            drip_speed = float(params['F'])
            drip_speed = self.speedunit_min_sec(axes[0], drip_speed)
            #if drip_speed <= 0.: 
            #    drip_speed = 20.0
            if drip_speed < self.min_speed: 
                drip_speed = self.min_speed  
            if drip_speed > self.max_speed: 
                drip_speed = self.max_speed                             
        dripparam = [dist,endval,drip_speed]
        logging.info("iaxes=%s param=%s\n",axes,dripparam) 
        if self.machinepos is None:
            kin = self.printer.lookup_object('toolhead').get_kinematics()
            self.machinepos = kin.get_machine_pos()
            self.machinepos[0][0] = self.xmin_axes
            self.machinepos[1][0] = self.ymin_axes
            self.machinepos[2][0] = self.zmin_axes   

        axis = axes[0]
        godist = dripparam[0]
        if dripparam[1] < 2:
            if  dripparam[1] == 1:
                #godist = position_max
                godist = self.machinepos[axis][1]
            else:
                #godist = position_min 
                godist = self.machinepos[axis][0] 

        toolhead = self.printer.lookup_object('toolhead')
        #curtime = self.printer.get_reactor().monotonic()
        #if 'z' not in toolhead.get_status(curtime)['homed_axes']:
        #    raise self.printer.command_error("Must home before probe")
        pos = toolhead.get_position()
        ismove = self.check_is_move(pos[axis], godist, dripparam[1])
        if ismove:
            pos[axis] = godist
            speedv = dripparam[2]
            logging.info("inparam=%s,%s\n",pos,speedv) 
            if self.jogworkmode == 0:
                self.jogworkmode = 1
                retv = self.probing_move(pos,speedv)
                self.jogworkmode = 0
                logging.info("probing_move=%s\n",retv) 
            else:
                logging.info("probing_move busy\n")
        else:
            logging.info("current position is here,no need move\n")    

    def cmd_LOOKMACHRANGE(self, gcmd): 
        if self.machinepos is None:
            kin = self.printer.lookup_object('toolhead').get_kinematics()
            self.machinepos = kin.get_machine_pos()
        msgstr = "machine range:" + str(self.machinepos)
        gcmd.respond_info(msgstr)   

def load_config_prefix(config):
    return Joggingrun(config)  
         
