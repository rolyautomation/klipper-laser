# G-Code G1 movement commands (and associated coordinate manipulation)
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging


# Enum
#Close
PWM_MODE_IDLE = 0
#Continuous
PWM_MODE_M3   = 1
#Dynamic
PWM_MODE_M4   = 2
#close nouse
PWM_MODE_M5   = 3



class GCodeMove:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        printer.register_event_handler("klippy:ready", self._handle_ready)
        printer.register_event_handler("klippy:shutdown", self._handle_shutdown)
        printer.register_event_handler("toolhead:set_position",
                                       self.reset_last_position)
        printer.register_event_handler("toolhead:manual_move",
                                       self.reset_last_position)
        printer.register_event_handler("gcode:command_error",
                                       self.reset_last_position)
        printer.register_event_handler("extruder:activate_extruder",
                                       self._handle_activate_extruder)
        printer.register_event_handler("homing:home_rails_end",
                                       self._handle_home_rails_end)
        self.is_printer_ready = False
        # Register g-code commands
        gcode = printer.lookup_object('gcode')
        self.gcode_fs = gcode
        handlers = [
            'G1', 'G20', 'G21',
            'M82', 'M83', 'G90', 'G91', 'G92', 'M220', 'M221',
            'SET_GCODE_OFFSET', 'SAVE_GCODE_STATE', 'RESTORE_GCODE_STATE',
        ]
        for cmd in handlers:
            func = getattr(self, 'cmd_' + cmd)
            desc = getattr(self, 'cmd_' + cmd + '_help', None)
            gcode.register_command(cmd, func, False, desc)
        #gcode.register_command('G0', self.cmd_G1)
        gcode.register_command('M114', self.cmd_M114, True)
        gcode.register_command('GET_POSITION', self.cmd_GET_POSITION, True,
                               desc=self.cmd_GET_POSITION_help)
        self.Coord = gcode.Coord
        # G-Code coordinate manipulation
        self.absolute_coord = self.absolute_extrude = True
        # Use relative distances 
        self.absolute_extrude = False
        self.base_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.last_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.homing_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.speed = 25.
        self.speed_factor = 1. / 60.
        self.extrude_factor = 1.
        #M3 M4 M5
        self.pwm_work_mode = 0
        self.pwm_work_curpower = 0
        self.pwm_work_mode_use = 0
        self.pwm_work_curpower_use = 0  
        self.pwm_work_ponoff_use = 0
        gcode.register_command('G0', self.cmd_G0_LASER)   
        #gcode.register_command('M3', self.cmd_M3_LASER)  
        #gcode.register_command('M4', self.cmd_M4_LASER)  
        #gcode.register_command('M5', self.cmd_M5_LASER)             
        gcode.register_command('M301', self.cmd_M3_LASER)  
        gcode.register_command('M302', self.cmd_M4_LASER)  
        gcode.register_command('M303', self.cmd_M5_LASER)  

        gcode.register_command('M222', self.cmd_M222_LASER_POWER)  
        gcode.register_command('M223', self.cmd_M223_LASER_SPEED)
        gcode.register_command('M224', self.cmd_M224_LOOK)
        self.laser_power_factor = 100.
        self.laser_speed_factor = 100.  
        self.laser_adj_min = 10.
        self.laser_adj_max = 200.
        self.laser_adj_ref = 100.
        self.adj_speed_mmsec  =  0
        self.adj_power_pwm  =  0


        # G-Code state
        self.galvo_coord_confactor = None
        self.saved_states = {}
        self.move_transform = self.move_with_transform = None
        self.predict_move_distance_with_transform  = None
        self.position_with_transform = (lambda: [0., 0., 0., 0., 0.0, 0.0, 0.0])
    def _handle_ready(self):
        self.is_printer_ready = True
        if self.move_transform is None:
            toolhead = self.printer.lookup_object('toolhead')
            self.move_with_transform = toolhead.move
            self.position_with_transform = toolhead.get_position
            self.predict_move_distance_with_transform = toolhead.predict_move_distance
        self.reset_last_position()
    def _handle_shutdown(self):
        if not self.is_printer_ready:
            return
        self.is_printer_ready = False
        logging.info("gcode state: absolute_coord=%s absolute_extrude=%s"
                     " base_position=%s last_position=%s homing_position=%s"
                     " speed_factor=%s extrude_factor=%s speed=%s",
                     self.absolute_coord, self.absolute_extrude,
                     self.base_position, self.last_position,
                     self.homing_position, self.speed_factor,
                     self.extrude_factor, self.speed)
    def _handle_activate_extruder(self):
        self.reset_last_position()
        self.extrude_factor = 1.
        self.base_position[3+3] = self.last_position[3+3]
    def _handle_home_rails_end(self, homing_state, rails):
        self.reset_last_position()
        for axis in homing_state.get_axes():
            self.base_position[axis] = self.homing_position[axis]
    def set_move_transform(self, transform, force=False):
        if self.move_transform is not None and not force:
            raise self.printer.config_error(
                "G-Code move transform already specified")
        old_transform = self.move_transform
        if old_transform is None:
            old_transform = self.printer.lookup_object('toolhead', None)
        self.move_transform = transform
        self.move_with_transform = transform.move
        self.position_with_transform = transform.get_position
        return old_transform
    def _get_gcode_position(self):
        p = [lp - bp for lp, bp in zip(self.last_position, self.base_position)]
        p[3+3] /= self.extrude_factor
        return p
    def _get_gcode_speed(self):
        return self.speed / self.speed_factor
    def _get_gcode_speed_override(self):
        return self.speed_factor * 60.
    def get_status(self, eventtime=None):
        move_position = self._get_gcode_position()
        return {
            'speed_factor': self._get_gcode_speed_override(),
            'speed': self._get_gcode_speed(),
            'extrude_factor': self.extrude_factor,
            'absolute_coordinates': self.absolute_coord,
            'absolute_extrude': self.absolute_extrude,
            'homing_origin': self.Coord(*self.homing_position),
            'position': self.Coord(*self.last_position),
            'gcode_position': self.Coord(*move_position),
        }
    def reset_last_position(self):
        if self.is_printer_ready:
            self.last_position = self.position_with_transform()

    def cmd_M222_LASER_POWER(self, gcmd):  
        value = gcmd.get_int('S', 100) 
        adj_value = self.laser_power_factor 
        if value < 100 and value > -100:
            adj_value += value
        else:
            adj_value = 100.
        if adj_value < self.laser_adj_min:
            adj_value = self.laser_adj_min
        if adj_value > self.laser_adj_max:
            adj_value = self.laser_adj_max
        self.laser_power_factor = adj_value

    def cmd_M223_LASER_SPEED(self, gcmd):  
        value = gcmd.get_int('S', 100) 
        adj_value = self.laser_speed_factor 
        if value < 100 and value > -100:
            adj_value += value
        else:
            adj_value = 100.
        if adj_value < self.laser_adj_min:
            adj_value = self.laser_adj_min
        if adj_value > self.laser_adj_max:
            adj_value = self.laser_adj_max
        self.laser_speed_factor = adj_value



    def cmd_M224_LOOK(self, gcmd):
        value = gcmd.get_int('S', 1) 
        msg = "noinfo"
        if value == 1:
            msg = "sf:%s,pf:%s" % (self.laser_speed_factor,self.laser_power_factor)
        elif value == 2:
            msg = "adjvalue:[%s,%s]to[%s,%s]" % (self.speed, self.pwm_work_curpower_use,
                self.adj_speed_mmsec, self.adj_power_pwm)
        else:
            msg = "wmode:%s,ponoff:%s" % (self.pwm_work_mode_use,self.pwm_work_ponoff_use)
        gcmd.respond_info(msg)  


    def adj_power_factor(self, power_pwm): 
        adj_power_value =  power_pwm*self.laser_speed_factor/self.laser_adj_ref
        adj_power_value =  adj_power_value*self.laser_power_factor/self.laser_adj_ref
        if adj_power_value  > 255:
            adj_power_value = 255
        return adj_power_value  

    def adj_speed_factor(self, speed_mmsec): 
        adj_speed_value =  speed_mmsec*self.laser_speed_factor/self.laser_adj_ref
        return adj_speed_value

    def reint_speedpower_factor(self): 
        self.laser_power_factor = self.laser_adj_ref
        self.laser_speed_factor = self.laser_adj_ref

    def cmd_M3_LASER(self, gcmd):  
        params = gcmd.get_command_parameters()  
        if 'S' in params:             
            v = float(params['S'])
            if v <= 0.:
                v = 0.
            if v >= 1000.:
                v = 1000. 
            v = v/1000.0 * 255
            self.pwm_work_curpower = v
        self.pwm_work_mode =  PWM_MODE_M3

    def cmd_M4_LASER(self, gcmd):  

        params = gcmd.get_command_parameters()  
        if 'S' in params:             
            v = float(params['S'])
            if v <= 0.:
                v = 0.
            if v >= 1000.:
                v = 1000. 
            v = v/1000.0 * 255
            self.pwm_work_curpower = v
        self.pwm_work_mode =  PWM_MODE_M4

    def cmd_M5_LASER(self, gcmd):  
        self.pwm_work_curpower = 0
        self.pwm_work_mode =  PWM_MODE_IDLE        
        self.pwm_work_mode_use = PWM_MODE_IDLE
        self.pwm_work_ponoff_use = 0
        #add 250117
        #self.reint_speedpower_factor()
        # safe , close pwm by macro

    def cmd_G0_LASER(self, gcmd):
        #self.pwm_work_mode_use =  PWM_MODE_IDLE
        self.pwm_work_mode_use =  self.pwm_work_mode
        params = gcmd.get_command_parameters()  
        if 'S' in params:             
            v = float(params['S'])
            if v <= 0.:
                v = 0.
            if v >= 1000.:
                v = 1000. 
            v = v/1000.0 * 255
            self.pwm_work_curpower = v        
        self.pwm_work_curpower_use =  self.pwm_work_curpower
        #next G1 use change power
        #self.pwm_work_curpower_use =  0
        self.pwm_work_ponoff_use = 0
        self.cmd_G1_Prefun(gcmd)  


    def is_fire_gfs_command(self, params):
        bret =  False
        if any(axis in params for axis in ['X', 'Y', 'Z', 'A', 'B', 'C']):
            return bret
        iret = all(fs in params for fs in ['F', 'S'])
        if iret:
            bret =  True

            svarstr = params['S']
            ncmd = "M305"
            self.gcode_fs.run_script_from_command(f"M305 S{svarstr}")
            #gfs_gcmd = self.gcode_fs.create_gcode_command(ncmd, ncmd, {"S":svarstr})
            #if self.gcode_fs.cmd_M305 is not None:
            #    self.gcode_fs.cmd_M305(gfs_gcmd)
            logging.info("[%s F%s S%s]",ncmd, params['F'], svarstr)  
        return bret
        

    # G-Code movement commands
    def cmd_G1(self, gcmd):
        # Move
        self.pwm_work_mode_use =  self.pwm_work_mode
        params = gcmd.get_command_parameters() 

        bret = self.is_fire_gfs_command(params)
        if bret:
            return

        if 'S' in params:             
            v = float(params['S'])
            if v <= 0.:
                v = 0.
            if v >= 1000.:
                v = 1000. 
            v = v/1000.0 * 255
            self.pwm_work_curpower = v  
        self.pwm_work_curpower_use =  self.pwm_work_curpower 
        self.pwm_work_ponoff_use = 1                    
        self.cmd_G1_Prefun(gcmd)     

    # G-Code movement commands
    def cmd_G1_Prefun(self, gcmd):
        # Move

        if self.galvo_coord_confactor is None:
            toolhead = self.printer.lookup_object('toolhead', None)
            if toolhead is None:
                raise gcmd.error("Printer not ready in cmd G1")
            kin = toolhead.get_kinematics()
            self.galvo_coord_confactor = kin.get_galvo_coord_confactor()            

        params = gcmd.get_command_parameters()
        try:
            for pos, axis in enumerate('XYZ'):
                if axis in params:
                    v = float(params[axis])
                    if not self.absolute_coord:
                        # value relative to position of last move
                        self.last_position[pos] += v
                    else:
                        # value relative to base coordinate position
                        self.last_position[pos] = v + self.base_position[pos]

            for pos, axis in enumerate('ABC'):
                if axis in params:
                    v = float(params[axis])
                    cur_absolute_coord = self.absolute_coord
                    offset_absolute_pos = 0
                    if self.galvo_coord_confactor is not None and pos > 0 :
                        offset_absolute_pos = self.galvo_coord_confactor
                        #v = v +  self.galvo_coord_confactor
                        #cur_absolute_coord = True

                    #if not self.absolute_coord:
                    if not cur_absolute_coord:                    
                        # value relative to position of last move
                        self.last_position[pos+3] += v
                    else:
                        # value relative to base coordinate position
                        #self.last_position[pos+3] = v + self.base_position[pos+3]
                        self.last_position[pos+3] = v + self.base_position[pos+3] + offset_absolute_pos
                        

            if 'E' in params:
                v = float(params['E']) * self.extrude_factor
                if not self.absolute_coord or not self.absolute_extrude:
                    # value relative to position of last move
                    self.last_position[3+3] += v
                else:
                    # value relative to base coordinate position
                    self.last_position[3+3] = v + self.base_position[3+3]

                logging.info("\nG1: absolute_coord=%s absolute_extrude=%s"
                     " base_position=%s last_position=%s "
                     " speed_factor=%s extrude_factor=%s speed=%s\n",
                     self.absolute_coord, self.absolute_extrude,
                     self.base_position[6], self.last_position[6],
                     self.speed_factor, 
                     self.extrude_factor, self.speed)
                     
            if 'F' in params:
                gcode_speed = float(params['F'])
                if gcode_speed <= 0.:
                    raise gcmd.error("Invalid speed in '%s'"
                                     % (gcmd.get_commandline(),))
                self.speed = gcode_speed * self.speed_factor

            move_e_axis_d = 0
            if (self.pwm_work_mode_use > 0):
                if self.predict_move_distance_with_transform is not None:
                    move_e_axis_d = self.predict_move_distance_with_transform(self.last_position)
                    # value relative to position of last move
                    self.last_position[3+3] += move_e_axis_d


        except ValueError as e:
            raise gcmd.error("Unable to parse move '%s'"
                             % (gcmd.get_commandline(),))


        self.adj_power_pwm  =  self.adj_power_factor(self.pwm_work_curpower_use)
        self.adj_speed_mmsec  =  self.adj_speed_factor(self.speed)
        #logging.info("adjvalue: [%s,%s]to[%s,%s]\n",self.speed, self.pwm_work_curpower_use,
                #self.adj_speed_mmsec, self.adj_power_pwm)    

        #close#logging.info("\npwm: pwm_work_curpower_use=%s pwm_work_mode_use=%s move_e_axis_d=%s ponoff=%s\n",
                #self.pwm_work_curpower_use, self.pwm_work_mode_use, move_e_axis_d, self.pwm_work_ponoff_use)     
        #self.move_with_transform(self.last_position, self.speed)
        #self.move_with_transform(self.last_position, self.speed, self.pwm_work_mode_use, self.pwm_work_curpower_use, self.pwm_work_ponoff_use)
        self.move_with_transform(self.last_position, self.adj_speed_mmsec, self.pwm_work_mode_use, self.adj_power_pwm, self.pwm_work_ponoff_use)  
            
        
    # G-Code coordinate manipulation
    def cmd_G20(self, gcmd):
        # Set units to inches
        raise gcmd.error('Machine does not support G20 (inches) command')
    def cmd_G21(self, gcmd):
        # Set units to millimeters
        pass
    def cmd_M82(self, gcmd):
        # Use absolute distances for extrusion
        self.absolute_extrude = True
    def cmd_M83(self, gcmd):
        # Use relative distances for extrusion
        self.absolute_extrude = False
    def cmd_G90(self, gcmd):
        # Use absolute coordinates
        self.absolute_coord = True
    def cmd_G91(self, gcmd):
        # Use relative coordinates
        self.absolute_coord = False
    def cmd_G92(self, gcmd):
        # Set position
        offsets = [ gcmd.get_float(a, None) for a in 'XYZABCE' ]
        for i, offset in enumerate(offsets):
            if offset is not None:
                #if i == 3:
                if i == 6:
                    offset *= self.extrude_factor
                self.base_position[i] = self.last_position[i] - offset
        if offsets == [None, None, None, None, None, None, None]:
            self.base_position = list(self.last_position)
    def cmd_M114(self, gcmd):
        # Get Current Position
        p = self._get_gcode_position()
        #gcmd.respond_raw("X:%.3f Y:%.3f Z:%.3f E:%.3f" % tuple(p))
        gcmd.respond_raw("X:%.3f Y:%.3f Z:%.3f A:%.3f B:%.3f C:%.3f E:%.3f" % tuple(p))
    def cmd_M220(self, gcmd):
        # Set speed factor override percentage
        value = gcmd.get_float('S', 100., above=0.) / (60. * 100.)
        self.speed = self._get_gcode_speed() * value
        self.speed_factor = value
    def cmd_M221(self, gcmd):
        # Set extrude factor override percentage
        new_extrude_factor = gcmd.get_float('S', 100., above=0.) / 100.
        last_e_pos = self.last_position[3+3]
        e_value = (last_e_pos - self.base_position[3+3]) / self.extrude_factor
        self.base_position[3+3] = last_e_pos - e_value * new_extrude_factor
        self.extrude_factor = new_extrude_factor
    cmd_SET_GCODE_OFFSET_help = "Set a virtual offset to g-code positions"
    def cmd_SET_GCODE_OFFSET(self, gcmd):
        move_delta = [0., 0., 0., 0., 0., 0., 0.]
        for pos, axis in enumerate('XYZABCE'):
            offset = gcmd.get_float(axis, None)
            if offset is None:
                offset = gcmd.get_float(axis + '_ADJUST', None)
                if offset is None:
                    continue
                offset += self.homing_position[pos]
            delta = offset - self.homing_position[pos]
            move_delta[pos] = delta
            self.base_position[pos] += delta
            self.homing_position[pos] = offset
        # Move the toolhead the given offset if requested
        if gcmd.get_int('MOVE', 0):
            speed = gcmd.get_float('MOVE_SPEED', self.speed, above=0.)
            for pos, delta in enumerate(move_delta):
                self.last_position[pos] += delta
            self.move_with_transform(self.last_position, speed)
    cmd_SAVE_GCODE_STATE_help = "Save G-Code coordinate state"
    def cmd_SAVE_GCODE_STATE(self, gcmd):
        state_name = gcmd.get('NAME', 'default')
        self.saved_states[state_name] = {
            'absolute_coord': self.absolute_coord,
            'absolute_extrude': self.absolute_extrude,
            'base_position': list(self.base_position),
            'last_position': list(self.last_position),
            'homing_position': list(self.homing_position),
            'speed': self.speed, 'speed_factor': self.speed_factor,
            'extrude_factor': self.extrude_factor,
        }
    cmd_RESTORE_GCODE_STATE_help = "Restore a previously saved G-Code state"
    def cmd_RESTORE_GCODE_STATE(self, gcmd):
        state_name = gcmd.get('NAME', 'default')
        state = self.saved_states.get(state_name)
        if state is None:
            raise gcmd.error("Unknown g-code state: %s" % (state_name,))
        # Restore state
        self.absolute_coord = state['absolute_coord']
        self.absolute_extrude = state['absolute_extrude']
        self.base_position = list(state['base_position'])
        self.homing_position = list(state['homing_position'])
        self.speed = state['speed']
        self.speed_factor = state['speed_factor']
        self.extrude_factor = state['extrude_factor']
        # Restore the relative E position
        e_diff = self.last_position[3+3] - state['last_position'][3+3]
        self.base_position[3+3] += e_diff
        # Move the toolhead back if requested
        if gcmd.get_int('MOVE', 0):
            speed = gcmd.get_float('MOVE_SPEED', self.speed, above=0.)
            self.last_position[:6] = state['last_position'][:6]
            self.move_with_transform(self.last_position, speed)
    cmd_GET_POSITION_help = (
        "Return information on the current location of the toolhead")
    def cmd_GET_POSITION(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead', None)
        if toolhead is None:
            raise gcmd.error("Printer not ready")
        kin = toolhead.get_kinematics()
        steppers = kin.get_steppers()
        mcu_pos = " ".join(["%s:%d" % (s.get_name(), s.get_mcu_position())
                            for s in steppers])
        cinfo = [(s.get_name(), s.get_commanded_position()) for s in steppers]
        stepper_pos = " ".join(["%s:%.6f" % (a, v) for a, v in cinfo])
        kinfo = zip("XYZABC", kin.calc_position(dict(cinfo)))
        kin_pos = " ".join(["%s:%.6f" % (a, v) for a, v in kinfo])
        toolhead_pos = " ".join(["%s:%.6f" % (a, v) for a, v in zip(
            "XYZABCE", toolhead.get_position())])
        gcode_pos = " ".join(["%s:%.6f"  % (a, v)
                              for a, v in zip("XYZABCE", self.last_position)])
        base_pos = " ".join(["%s:%.6f"  % (a, v)
                             for a, v in zip("XYZABCE", self.base_position)])
        homing_pos = " ".join(["%s:%.6f"  % (a, v)
                               for a, v in zip("XYZABC", self.homing_position)])
        gcmd.respond_info("mcu: %s\n"
                          "stepper: %s\n"
                          "kinematic: %s\n"
                          "toolhead: %s\n"
                          "gcode: %s\n"
                          "gcode base: %s\n"
                          "gcode homing: %s"
                          % (mcu_pos, stepper_pos, kin_pos, toolhead_pos,
                             gcode_pos, base_pos, homing_pos))

def load_config(config):
    return GCodeMove(config)
