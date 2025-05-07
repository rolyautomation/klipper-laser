# Code for handling printer nozzle extruders
#
# Copyright (C) 2016-2022  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
import stepper, chelper
import array


class ExtruderStepper:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.pressure_advance = self.pressure_advance_smooth_time = 0.
        self.config_pa = config.getfloat('pressure_advance', 0., minval=0.)
        self.config_smooth_time = config.getfloat(
                'pressure_advance_smooth_time', 0.040, above=0., maxval=.200)
        # Setup stepper
        self.stepper = stepper.PrinterStepper(config)
        ffi_main, ffi_lib = chelper.get_ffi()
        self.sk_extruder = ffi_main.gc(ffi_lib.extruder_stepper_alloc(),
                                       ffi_lib.free)
        self.stepper.set_stepper_kinematics(self.sk_extruder)
        self.motion_queue = None
        # Register commands
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        gcode = self.printer.lookup_object('gcode')
        if self.name == 'extruder':
            gcode.register_mux_command("SET_PRESSURE_ADVANCE", "EXTRUDER", None,
                                       self.cmd_default_SET_PRESSURE_ADVANCE,
                                       desc=self.cmd_SET_PRESSURE_ADVANCE_help)
        gcode.register_mux_command("SET_PRESSURE_ADVANCE", "EXTRUDER",
                                   self.name, self.cmd_SET_PRESSURE_ADVANCE,
                                   desc=self.cmd_SET_PRESSURE_ADVANCE_help)
        gcode.register_mux_command("SET_EXTRUDER_ROTATION_DISTANCE", "EXTRUDER",
                                   self.name, self.cmd_SET_E_ROTATION_DISTANCE,
                                   desc=self.cmd_SET_E_ROTATION_DISTANCE_help)
        gcode.register_mux_command("SYNC_EXTRUDER_MOTION", "EXTRUDER",
                                   self.name, self.cmd_SYNC_EXTRUDER_MOTION,
                                   desc=self.cmd_SYNC_EXTRUDER_MOTION_help)
    def _handle_connect(self):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_step_generator(self.stepper.generate_steps)
        self._set_pressure_advance(self.config_pa, self.config_smooth_time)
    def get_status(self, eventtime):
        return {'pressure_advance': self.pressure_advance,
                'smooth_time': self.pressure_advance_smooth_time,
                'motion_queue': self.motion_queue}
    def find_past_position(self, print_time):
        mcu_pos = self.stepper.get_past_mcu_position(print_time)
        return self.stepper.mcu_to_commanded_position(mcu_pos)
    def sync_to_extruder(self, extruder_name):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        if not extruder_name:
            self.stepper.set_trapq(None)
            self.motion_queue = None
            return
        extruder = self.printer.lookup_object(extruder_name, None)
        if extruder is None or not isinstance(extruder, PrinterExtruder):
            raise self.printer.command_error("'%s' is not a valid extruder."
                                             % (extruder_name,))
        self.stepper.set_position([extruder.last_position, 0., 0.])
        self.stepper.set_trapq(extruder.get_trapq())
        self.motion_queue = extruder_name
    def _set_pressure_advance(self, pressure_advance, smooth_time):
        old_smooth_time = self.pressure_advance_smooth_time
        if not self.pressure_advance:
            old_smooth_time = 0.
        new_smooth_time = smooth_time
        if not pressure_advance:
            new_smooth_time = 0.
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.note_step_generation_scan_time(new_smooth_time * .5,
                                                old_delay=old_smooth_time * .5)
        ffi_main, ffi_lib = chelper.get_ffi()
        espa = ffi_lib.extruder_set_pressure_advance
        espa(self.sk_extruder, pressure_advance, new_smooth_time)
        self.pressure_advance = pressure_advance
        self.pressure_advance_smooth_time = smooth_time
    cmd_SET_PRESSURE_ADVANCE_help = "Set pressure advance parameters"
    def cmd_default_SET_PRESSURE_ADVANCE(self, gcmd):
        extruder = self.printer.lookup_object('toolhead').get_extruder()
        if extruder.extruder_stepper is None:
            raise gcmd.error("Active extruder does not have a stepper")
        strapq = extruder.extruder_stepper.stepper.get_trapq()
        if strapq is not extruder.get_trapq():
            raise gcmd.error("Unable to infer active extruder stepper")
        extruder.extruder_stepper.cmd_SET_PRESSURE_ADVANCE(gcmd)
    def cmd_SET_PRESSURE_ADVANCE(self, gcmd):
        pressure_advance = gcmd.get_float('ADVANCE', self.pressure_advance,
                                          minval=0.)
        smooth_time = gcmd.get_float('SMOOTH_TIME',
                                     self.pressure_advance_smooth_time,
                                     minval=0., maxval=.200)
        self._set_pressure_advance(pressure_advance, smooth_time)
        msg = ("pressure_advance: %.6f\n"
               "pressure_advance_smooth_time: %.6f"
               % (pressure_advance, smooth_time))
        self.printer.set_rollover_info(self.name, "%s: %s" % (self.name, msg))
        gcmd.respond_info(msg, log=False)
    cmd_SET_E_ROTATION_DISTANCE_help = "Set extruder rotation distance"
    def cmd_SET_E_ROTATION_DISTANCE(self, gcmd):
        rotation_dist = gcmd.get_float('DISTANCE', None)
        if rotation_dist is not None:
            if not rotation_dist:
                raise gcmd.error("Rotation distance can not be zero")
            invert_dir, orig_invert_dir = self.stepper.get_dir_inverted()
            next_invert_dir = orig_invert_dir
            if rotation_dist < 0.:
                next_invert_dir = not orig_invert_dir
                rotation_dist = -rotation_dist
            toolhead = self.printer.lookup_object('toolhead')
            toolhead.flush_step_generation()
            self.stepper.set_rotation_distance(rotation_dist)
            self.stepper.set_dir_inverted(next_invert_dir)
        else:
            rotation_dist, spr = self.stepper.get_rotation_distance()
        invert_dir, orig_invert_dir = self.stepper.get_dir_inverted()
        if invert_dir != orig_invert_dir:
            rotation_dist = -rotation_dist
        gcmd.respond_info("Extruder '%s' rotation distance set to %0.6f"
                          % (self.name, rotation_dist))
    cmd_SYNC_EXTRUDER_MOTION_help = "Set extruder stepper motion queue"
    def cmd_SYNC_EXTRUDER_MOTION(self, gcmd):
        ename = gcmd.get('MOTION_QUEUE')
        self.sync_to_extruder(ename)
        gcmd.respond_info("Extruder '%s' now syncing with '%s'"
                          % (self.name, ename))

# Tracking for hotend heater, extrusion motion queue, and extruder stepper
class PrinterExtruder:
    def __init__(self, config, extruder_num):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.last_position = 0.
        # Setup hotend heater
        pheaters = self.printer.load_object(config, 'heaters')
        gcode_id = 'T%d' % (extruder_num,)
        self.heater = pheaters.setup_heater(config, gcode_id)
        # Setup kinematic checks
        self.nozzle_diameter = config.getfloat('nozzle_diameter', above=0.)
        filament_diameter = config.getfloat(
            'filament_diameter', minval=self.nozzle_diameter)
        self.filament_area = math.pi * (filament_diameter * .5)**2
        def_max_cross_section = 4. * self.nozzle_diameter**2
        def_max_extrude_ratio = def_max_cross_section / self.filament_area
        max_cross_section = config.getfloat(
            'max_extrude_cross_section', def_max_cross_section, above=0.)
        self.max_extrude_ratio = max_cross_section / self.filament_area
        logging.info("Extruder max_extrude_ratio=%.6f", self.max_extrude_ratio)
        toolhead = self.printer.lookup_object('toolhead')
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_e_velocity = config.getfloat(
            'max_extrude_only_velocity', max_velocity * def_max_extrude_ratio
            , above=0.)
        self.max_e_accel = config.getfloat(
            'max_extrude_only_accel', max_accel * def_max_extrude_ratio
            , above=0.)
        self.max_e_dist = config.getfloat(
            'max_extrude_only_distance', 50., minval=0.)
        self.instant_corner_v = config.getfloat(
            'instantaneous_corner_velocity', 1., minval=0.)
        # Setup extruder trapq (trapezoidal motion queue)
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapq_append = ffi_lib.trapq_append
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        # Setup extruder stepper
        self.extruder_stepper = None
        if (config.get('step_pin', None) is not None
            or config.get('dir_pin', None) is not None
            or config.get('rotation_distance', None) is not None):
            self.extruder_stepper = ExtruderStepper(config)
            self.extruder_stepper.stepper.set_trapq(self.trapq)
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        if self.name == 'extruder':
            toolhead.set_extruder(self, 0.)
            gcode.register_command("M104", self.cmd_M104)
            gcode.register_command("M109", self.cmd_M109)
        gcode.register_mux_command("ACTIVATE_EXTRUDER", "EXTRUDER",
                                   self.name, self.cmd_ACTIVATE_EXTRUDER,
                                   desc=self.cmd_ACTIVATE_EXTRUDER_help)
    def update_move_time(self, flush_time, clear_history_time):
        self.trapq_finalize_moves(self.trapq, flush_time, clear_history_time)
    def get_status(self, eventtime):
        sts = self.heater.get_status(eventtime)
        sts['can_extrude'] = self.heater.can_extrude
        if self.extruder_stepper is not None:
            sts.update(self.extruder_stepper.get_status(eventtime))
        return sts
    def get_name(self):
        return self.name
    def get_heater(self):
        return self.heater
    def get_trapq(self):
        return self.trapq
    def stats(self, eventtime):
        return self.heater.stats(eventtime)
    def check_move(self, move):
        axis_r = move.axes_r[3]
        if not self.heater.can_extrude:
            raise self.printer.command_error(
                "Extrude below minimum temp\n"
                "See the 'min_extrude_temp' config option for details")
        if (not move.axes_d[0] and not move.axes_d[1]) or axis_r < 0.:
            # Extrude only move (or retraction move) - limit accel and velocity
            if abs(move.axes_d[3]) > self.max_e_dist:
                raise self.printer.command_error(
                    "Extrude only move too long (%.3fmm vs %.3fmm)\n"
                    "See the 'max_extrude_only_distance' config"
                    " option for details" % (move.axes_d[3], self.max_e_dist))
            inv_extrude_r = 1. / abs(axis_r)
            move.limit_speed(self.max_e_velocity * inv_extrude_r,
                             self.max_e_accel * inv_extrude_r)
        elif axis_r > self.max_extrude_ratio:
            if move.axes_d[3] <= self.nozzle_diameter * self.max_extrude_ratio:
                # Permit extrusion if amount extruded is tiny
                return
            area = axis_r * self.filament_area
            logging.debug("Overextrude: %s vs %s (area=%.3f dist=%.3f)",
                          axis_r, self.max_extrude_ratio, area, move.move_d)
            raise self.printer.command_error(
                "Move exceeds maximum extrusion (%.3fmm^2 vs %.3fmm^2)\n"
                "See the 'max_extrude_cross_section' config option for details"
                % (area, self.max_extrude_ratio * self.filament_area))
    def calc_junction(self, prev_move, move):
        diff_r = move.axes_r[3] - prev_move.axes_r[3]
        if diff_r:
            return (self.instant_corner_v / abs(diff_r))**2
        return move.max_cruise_v2
    def move(self, print_time, move):
        axis_r = move.axes_r[3]
        accel = move.accel * axis_r
        start_v = move.start_v * axis_r
        cruise_v = move.cruise_v * axis_r
        can_pressure_advance = False
        if axis_r > 0. and (move.axes_d[0] or move.axes_d[1]):
            can_pressure_advance = True
        # Queue movement (x is extruder movement, y is pressure advance flag)
        self.trapq_append(self.trapq, print_time,
                          move.accel_t, move.cruise_t, move.decel_t,
                          move.start_pos[3], 0., 0.,
                          0., 0., 0.,
                          1., can_pressure_advance, 0.,
                          0., 0., 0.,
                          start_v, cruise_v, accel, 0)
        self.last_position = move.end_pos[3]
    def find_past_position(self, print_time):
        if self.extruder_stepper is None:
            return 0.
        return self.extruder_stepper.find_past_position(print_time)
    def cmd_M104(self, gcmd, wait=False):
        # Set Extruder Temperature
        temp = gcmd.get_float('S', 0.)
        index = gcmd.get_int('T', None, minval=0)
        if index is not None:
            section = 'extruder'
            if index:
                section = 'extruder%d' % (index,)
            extruder = self.printer.lookup_object(section, None)
            if extruder is None:
                if temp <= 0.:
                    return
                raise gcmd.error("Extruder not configured")
        else:
            extruder = self.printer.lookup_object('toolhead').get_extruder()
        pheaters = self.printer.lookup_object('heaters')
        pheaters.set_temperature(extruder.get_heater(), temp, wait)
    def cmd_M109(self, gcmd):
        # Set Extruder Temperature and Wait
        self.cmd_M104(gcmd, wait=True)
    cmd_ACTIVATE_EXTRUDER_help = "Change the active extruder"
    def cmd_ACTIVATE_EXTRUDER(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        if toolhead.get_extruder() is self:
            gcmd.respond_info("Extruder %s already active" % (self.name,))
            return
        gcmd.respond_info("Activating extruder %s" % (self.name,))
        toolhead.flush_step_generation()
        toolhead.set_extruder(self, self.last_position)
        self.printer.send_event("extruder:activate_extruder")

# Dummy extruder class used when a printer has no extruder at all
class DummyExtruder:
    def __init__(self, printer):
        self.printer = printer
    def update_move_time(self, flush_time, clear_history_time):
        pass
    def check_move(self, move):
        raise move.move_error("Extrude when no extruder present")
    def find_past_position(self, print_time):
        return 0.
    def calc_junction(self, prev_move, move):
        return move.max_cruise_v2
    def get_name(self):
        return ""
    def get_heater(self):
        raise self.printer.command_error("Extruder not configured")
    def get_trapq(self):
        raise self.printer.command_error("Extruder not configured")


'''
def add_printer_objects(config):
    printer = config.get_printer()
    for i in range(99):
        section = 'extruder'
        if i:
            section = 'extruder%d' % (i,)
        if not config.has_section(section):
            break
        pe = PrinterExtruder(config.getsection(section), i)
        printer.add_object(section, pe)
''' 

MIN_PWM_ZERO_VAL  = 0.1


class ExtruderStepperPWM:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.pressure_advance = self.pressure_advance_smooth_time = 0.
        self.config_pa = self.config_smooth_time = 0.
        #self.config_pa = config.getfloat('pressure_advance', 0., minval=0.)
        #self.config_smooth_time = config.getfloat(
        #        'pressure_advance_smooth_time', 0.040, above=0., maxval=.200)
        # Setup stepper
        self.stepper = stepper.PrinterStepper(config)
        ffi_main, ffi_lib = chelper.get_ffi()
        self.sk_extruder = ffi_main.gc(ffi_lib.extruder_stepper_alloc(),
                                       ffi_lib.free)
        self.stepper.set_stepper_kinematics(self.sk_extruder)
        self.motion_queue = None
        # Register commands
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)

        self._step_dist_tick = None                                            

        '''                                    
        gcode = self.printer.lookup_object('gcode')
        if self.name == 'extruder':
            gcode.register_mux_command("SET_PRESSURE_ADVANCE", "EXTRUDER", None,
                                       self.cmd_default_SET_PRESSURE_ADVANCE,
                                       desc=self.cmd_SET_PRESSURE_ADVANCE_help)
        gcode.register_mux_command("SET_PRESSURE_ADVANCE", "EXTRUDER",
                                   self.name, self.cmd_SET_PRESSURE_ADVANCE,
                                   desc=self.cmd_SET_PRESSURE_ADVANCE_help)
        gcode.register_mux_command("SET_EXTRUDER_ROTATION_DISTANCE", "EXTRUDER",
                                   self.name, self.cmd_SET_E_ROTATION_DISTANCE,
                                   desc=self.cmd_SET_E_ROTATION_DISTANCE_help)
        gcode.register_mux_command("SYNC_EXTRUDER_MOTION", "EXTRUDER",
                                   self.name, self.cmd_SYNC_EXTRUDER_MOTION,
                                   desc=self.cmd_SYNC_EXTRUDER_MOTION_help)
        '''

    def _handle_connect(self):
        logging.info("_handle_connect  extruderpwm")
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_step_generator(self.stepper.generate_steps)
        self._set_pressure_advance(self.config_pa, self.config_smooth_time)
    def get_status(self, eventtime):
        return {'pressure_advance': self.pressure_advance,
                'smooth_time': self.pressure_advance_smooth_time,
                'motion_queue': self.motion_queue}
    def find_past_position(self, print_time):
        mcu_pos = self.stepper.get_past_mcu_position(print_time)
        return self.stepper.mcu_to_commanded_position(mcu_pos)
    def sync_to_extruder(self, extruder_name):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        if not extruder_name:
            self.stepper.set_trapq(None)
            self.motion_queue = None
            return
        extruder = self.printer.lookup_object(extruder_name, None)
        if extruder is None or not isinstance(extruder, PrinterExtruder):
            raise self.printer.command_error("'%s' is not a valid extruder."
                                             % (extruder_name,))
        self.stepper.set_position([extruder.last_position, 0., 0.])
        self.stepper.set_trapq(extruder.get_trapq())
        self.motion_queue = extruder_name

    def set_extruder_stepper_pwm(self, pwm_oid,mcu_id,laser_type=0):
        if (self.stepper is not None):
            extstepper_oid = self.stepper.get_oid()
            extstepper_mcu = self.stepper.get_mcu()
            extstepper_name = self.stepper.get_name()
            if(extstepper_mcu is mcu_id ):
                self.stepper.bind_stepper_pwm(pwm_oid,laser_type)
                logging.info("set_extruder_stepper_pwm:%s %i", extstepper_name,extstepper_oid)
            else:
                #logging.info("set_extruder_stepper_pwm:%s %i fail: not same mcu", extstepper_name,extstepper_oid) 
                raise self.printer.command_error("'%s' is not same mcu,please check."
                                             % (extstepper_name,))  



    def configminpower_stepper_pwm(self, min_power=0):
        if (self.stepper is not None):
            self.stepper.setminpower_stepper_pwm(min_power)


    def set_pauseresumeflag_pwm(self, pwm_prf=0):
        if (self.stepper is not None):
            self.stepper.pauseresumep_stepper_pwm(pwm_prf)

    #def update_mcu_clock_freq(self):
        # Fetch the latest clock frequency from the MCU
    #    extstepper_mcu = self.stepper.get_mcu()
    #    self.mcu_clock_freq_value = extstepper_mcu.get_constant_float('CLOCK_FREQ')

    def get_step_dist_val(self):
        if self._step_dist_tick is None:
            self._step_dist_tick  =  self.stepper.get_step_dist()
        return self._step_dist_tick

    def cacl_step_dist_tick(self,curspeed=1):
        if self._step_dist_tick is None:
            self._step_dist_tick = self.stepper.get_step_dist()
        _step_dist_time = self._step_dist_tick/curspeed
        extstepper_mcu = self.stepper.get_mcu()
        _mcu_tick_clock = extstepper_mcu.seconds_to_clock(1.0)
        _mcu_tick_value = _step_dist_time*_mcu_tick_clock*2
        if (_mcu_tick_clock > 13000000):
            raise self.printer.command_error("'%d' clock freq is error,please check."
                                             % (_mcu_tick_clock,))  
        return(_mcu_tick_value)

    def _set_pressure_advance(self, pressure_advance, smooth_time):
        old_smooth_time = self.pressure_advance_smooth_time
        if not self.pressure_advance:
            old_smooth_time = 0.
        new_smooth_time = smooth_time
        if not pressure_advance:
            new_smooth_time = 0.
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.note_step_generation_scan_time(new_smooth_time * .5,
                                                old_delay=old_smooth_time * .5)
        ffi_main, ffi_lib = chelper.get_ffi()
        espa = ffi_lib.extruder_set_pressure_advance
        espa(self.sk_extruder, pressure_advance, new_smooth_time)
        self.pressure_advance = pressure_advance
        self.pressure_advance_smooth_time = smooth_time
    cmd_SET_PRESSURE_ADVANCE_help = "Set pressure advance parameters"
    def cmd_default_SET_PRESSURE_ADVANCE(self, gcmd):
        extruder = self.printer.lookup_object('toolhead').get_extruder()
        if extruder.extruder_stepper is None:
            raise gcmd.error("Active extruder does not have a stepper")
        strapq = extruder.extruder_stepper.stepper.get_trapq()
        if strapq is not extruder.get_trapq():
            raise gcmd.error("Unable to infer active extruder stepper")
        extruder.extruder_stepper.cmd_SET_PRESSURE_ADVANCE(gcmd)
    def cmd_SET_PRESSURE_ADVANCE(self, gcmd):
        pressure_advance = gcmd.get_float('ADVANCE', self.pressure_advance,
                                          minval=0.)
        smooth_time = gcmd.get_float('SMOOTH_TIME',
                                     self.pressure_advance_smooth_time,
                                     minval=0., maxval=.200)
        self._set_pressure_advance(pressure_advance, smooth_time)
        msg = ("pressure_advance: %.6f\n"
               "pressure_advance_smooth_time: %.6f"
               % (pressure_advance, smooth_time))
        self.printer.set_rollover_info(self.name, "%s: %s" % (self.name, msg))
        gcmd.respond_info(msg, log=False)
    cmd_SET_E_ROTATION_DISTANCE_help = "Set extruder rotation distance"
    def cmd_SET_E_ROTATION_DISTANCE(self, gcmd):
        rotation_dist = gcmd.get_float('DISTANCE', None)
        if rotation_dist is not None:
            if not rotation_dist:
                raise gcmd.error("Rotation distance can not be zero")
            invert_dir, orig_invert_dir = self.stepper.get_dir_inverted()
            next_invert_dir = orig_invert_dir
            if rotation_dist < 0.:
                next_invert_dir = not orig_invert_dir
                rotation_dist = -rotation_dist
            toolhead = self.printer.lookup_object('toolhead')
            toolhead.flush_step_generation()
            self.stepper.set_rotation_distance(rotation_dist)
            self.stepper.set_dir_inverted(next_invert_dir)
        else:
            rotation_dist, spr = self.stepper.get_rotation_distance()
        invert_dir, orig_invert_dir = self.stepper.get_dir_inverted()
        if invert_dir != orig_invert_dir:
            rotation_dist = -rotation_dist
        gcmd.respond_info("Extruder '%s' rotation distance set to %0.6f"
                          % (self.name, rotation_dist))
    cmd_SYNC_EXTRUDER_MOTION_help = "Set extruder stepper motion queue"
    def cmd_SYNC_EXTRUDER_MOTION(self, gcmd):
        ename = gcmd.get('MOTION_QUEUE')
        self.sync_to_extruder(ename)
        gcmd.respond_info("Extruder '%s' now syncing with '%s'"
                          % (self.name, ename))

# Tracking for extrusion motion queue, and extruder stepper ，remove hotend heater
class PrinterExtruderPWM:
    def __init__(self, config, extruder_num):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.last_position = 0.
        # Setup hotend heater
        #pheaters = self.printer.load_object(config, 'heaters')
        #gcode_id = 'T%d' % (extruder_num,)
        #self.heater = pheaters.setup_heater(config, gcode_id)
        # Setup kinematic checks
        #self.nozzle_diameter = config.getfloat('nozzle_diameter', above=0.)
        #filament_diameter = config.getfloat(
        #    'filament_diameter', minval=self.nozzle_diameter)
        #self.filament_area = math.pi * (filament_diameter * .5)**2
        #def_max_cross_section = 4. * self.nozzle_diameter**2
        #def_max_extrude_ratio = def_max_cross_section / self.filament_area
        #max_cross_section = config.getfloat(
        #    'max_extrude_cross_section', def_max_cross_section, above=0.)
        #self.max_extrude_ratio = max_cross_section / self.filament_area
        #logging.info("Extruder max_extrude_ratio=%.6f", self.max_extrude_ratio)
        #new add  240904

        def_max_extrude_ratio = 1
        toolhead = self.printer.lookup_object('toolhead')
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_e_velocity = config.getfloat(
            'max_extrude_only_velocity', max_velocity * def_max_extrude_ratio
            , above=0.)
        self.max_e_accel = config.getfloat(
            'max_extrude_only_accel', max_accel * def_max_extrude_ratio
            , above=0.)
        self.max_e_dist = config.getfloat(
            'max_extrude_only_distance', 50., minval=0.)
        self.instant_corner_v = config.getfloat(
            'instantaneous_corner_velocity', 1., minval=0.)

        laser_minpower = config.getfloat(
            'laser_minpower', 0., minval=0., maxval=250.)   
        if abs(laser_minpower) < 1e-3:
            self.lasermin_power = 0
        else:
            self.lasermin_power = int(laser_minpower + 0.5)  

        # self.diff_pwmval = config.getfloat(
        #     'diff_pwmval', 0., minval=0., maxval=255.) 
        
        self.pre_ptagcode = 0
        self.psynccode = 0
        self.ptagcode = 0
        self.ep_step_dist_tick = None
        self.powertable_max = 64
        self.c_array = array.array('B', [0] * self.powertable_max)  
        #self.c_array_NULL = array.array('B', [0] * 2)            

        logging.info("EP:%s =%.6f,%d",self.name, self.max_e_velocity, self.lasermin_power) 
        # Setup extruder trapq (trapezoidal motion queue)
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        #self.trapq_append = ffi_lib.trapq_append
        self.trapq_append_extend = ffi_lib.trapq_append_extend
        self.power_table_ptr = ffi_main.from_buffer(self.c_array)
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        # Setup extruder stepper
        self.extruder_stepper = None
        if (config.get('step_pin', None) is not None
            or config.get('dir_pin', None) is not None
            or config.get('rotation_distance', None) is not None):
            self.extruder_stepper = ExtruderStepperPWM(config)
            self.extruder_stepper.stepper.set_trapq(self.trapq)
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        #if self.name == 'extruder':
        if self.name == 'extruderpwm':
            toolhead.set_extruder(self, 0.)
            gcode.register_command("M104", self.cmd_M104)
            gcode.register_command("M109", self.cmd_M109)
            #gcode.register_command("TPSW", self.cmd_TPWMSW)
            gcode.register_mux_command("TPSW", "LASER", None,
                                       self.cmd_default_TPWMSW,
                                       desc=self.cmd_TPWMSW_help)    
            gcode.register_mux_command("RSYNCPOWER", "LASER", None,
                                       self.cmd_default_RSYNCPOWER,
                                       desc=self.cmd_RSYNCPOWER_help)                                         


        #TPSW_M
        gcode.register_mux_command("TPSW", "LASER",
                                   self.name, self.cmd_TPWMSW,
                                   desc=self.cmd_TPWMSW_help)           

        gcode.register_mux_command("ACTIVATE_EXTRUDER", "EXTRUDER",
                                   self.name, self.cmd_ACTIVATE_EXTRUDER,
                                   desc=self.cmd_ACTIVATE_EXTRUDER_help)

        gcode.register_mux_command("ACTIVATE_LASER", "LASER",
                                   self.name, self.cmd_ACTIVATE_LASER,
                                   desc=self.cmd_ACTIVATE_LASER_help)                                   

        self._extrdpwm_oid = None
        logging.info("PrinterExtruderPWM end") 
        self.optmode = 1
        self._restartcmd_sn = 1
        self._laser_type = 0
        self._pwm_prf = 0
        self.last_min_power = 0 
        #self.pre_pwmval = 0
        #RSYNCPOWER_M
        gcode.register_mux_command("RSYNCPOWER", "LASER",
                                   self.name, self.cmd_RSYNCPOWER,
                                   desc=self.cmd_RSYNCPOWER_help)  


       
    #self.cmd_RSYNCPOWER_help = "RSYNCPOWER LASER=RSYNCPOWER 1"
    cmd_RSYNCPOWER_help = "RSYNCPOWER LASER=RSYNCPOWER OR 1"
    def cmd_default_RSYNCPOWER(self, gcmd):
        extruder = self.printer.lookup_object('toolhead').get_extruder()
        if extruder.extruder_stepper is None:
            raise gcmd.error("Active laser does not have a stepper on rsyncpower")
        strapq = extruder.extruder_stepper.stepper.get_trapq()
        if strapq is not extruder.get_trapq():
            raise gcmd.error("Unable to infer active laser stepper on rsyncpower")
        extruder.cmd_RSYNCPOWER(gcmd)
        #logging.info("\nrsyncpower:%s\n", extruder.name) 
        pass


    def cmd_RSYNCPOWER(self, gcmd):
        syncpmode = gcmd.get_int('M',0, minval=0, maxval=10) 
        if syncpmode == 0:
            self.optmode = 0
            self._restartcmd_sn = 0
        elif syncpmode == 1:
            self.optmode = 1 
            self._restartcmd_sn = (self._restartcmd_sn % 120) + 1
        msg = "resp name=%s optmode=%s restartcmd_sn=%s " % (self.name, self.optmode, self._restartcmd_sn)            
        gcmd.respond_info(msg)  
        

    def update_move_time(self, flush_time, clear_history_time):
        #close#logging.info("update_move_time extruderpwm") 
        self.trapq_finalize_moves(self.trapq, flush_time, clear_history_time)
    def get_status(self, eventtime):
        #logging.info("get_status extruderpwm") 
        #sts = self.heater.get_status(eventtime)
        #sts['can_extrude'] = self.heater.can_extrude
        #if self.extruder_stepper is not None:
        #    sts.update(self.extruder_stepper.get_status(eventtime))
        sts = {
               'laser_type': self._laser_type,
               'laser_onf': self._pwm_prf,
               'laser_minp':  self.last_min_power,               
               'extpwm': 'no info'
               }
        #printer[printer.toolhead.extruder].laser_type       
        #sts['extpwm'] = 'no info'
        return sts
    #   return 0
    def set_extrdpwm_oid(self, pwm_oid=None, mcu_id=None,laser_type=0):
        if (self._extrdpwm_oid is None):
            if ( pwm_oid is not None):
                self.extruder_stepper.set_extruder_stepper_pwm(pwm_oid,mcu_id,laser_type)
                self._extrdpwm_oid = pwm_oid
                self._laser_type =  laser_type
                logging.info("rev bind pwm: %i,%s", pwm_oid,self._laser_type)
                self.configminpower_stepper_pwm()
                pass
        else:
            logging.info("laser type : %i", self._laser_type)
            pass

    def configminpower_stepper_pwm(self): 
        if self.lasermin_power > 0:
            self.extruder_stepper.configminpower_stepper_pwm(self.lasermin_power) 
            self.last_min_power = self.lasermin_power 

    def configminpower_stepper_pwm_cmd(self, min_power=0): 
        if self.last_min_power != min_power:
            self.extruder_stepper.configminpower_stepper_pwm(min_power)           
            self.last_min_power = min_power


    def set_pauseresume_pwm(self,pwm_prf=0):
        if (self._pwm_prf != pwm_prf):
            self.extruder_stepper.set_pauseresumeflag_pwm(pwm_prf)
            self._pwm_prf = pwm_prf

    def cacl_step_dist_tick(self,curspeed=1):
        plus_inter_tick = self.extruder_stepper.cacl_step_dist_tick(curspeed)
        return(plus_inter_tick)

    def cacl_distance_count(self, distmm):
        if self.ep_step_dist_tick is None:
            self.ep_step_dist_tick = self.extruder_stepper.get_step_dist_val()
        dist_count = int(distmm/self.ep_step_dist_tick)
        return(dist_count)        

    def set_restart_pwmdcmd(self, val=0):   
        self._restartcmd_sn = val

    def get_name(self):
        return self.name
    def get_heater(self):
        #return self.heater
        logging.info("get_heater extruderpwm") 
        return 0
    def get_trapq(self):
        return self.trapq
    def stats(self, eventtime):
        #return self.heater.stats(eventtime)
        #logging.info("stats extruderpwm") 
        return (False, "extpwm")        
    #def stats(self, eventtime):
        #return self.heater.stats(eventtime)
        #logging.info("stats extruderpwm") 
        #return False, "expwmstates=%d" % (1,)
        
    def check_move(self, move):
        axis_r = move.axes_r[3+3]
        '''
        if not self.heater.can_extrude:
            raise self.printer.command_error(
                "Extrude below minimum temp\n"
                "See the 'min_extrude_temp' config option for details")
        
        if (not move.axes_d[0] and not move.axes_d[1]) or axis_r < 0.:
            # Extrude only move (or retraction move) - limit accel and velocity
            if abs(move.axes_d[3]) > self.max_e_dist:
                raise self.printer.command_error(
                    "Extrude only move too long (%.3fmm vs %.3fmm)\n"
                    "See the 'max_extrude_only_distance' config"
                    " option for details" % (move.axes_d[3], self.max_e_dist))
            inv_extrude_r = 1. / abs(axis_r)
            move.limit_speed(self.max_e_velocity * inv_extrude_r,
                             self.max_e_accel * inv_extrude_r)
        elif axis_r > self.max_extrude_ratio:
            if move.axes_d[3] <= self.nozzle_diameter * self.max_extrude_ratio:
                # Permit extrusion if amount extruded is tiny
                return
            area = axis_r * self.filament_area
            logging.debug("Overextrude: %s vs %s (area=%.3f dist=%.3f)",
                          axis_r, self.max_extrude_ratio, area, move.move_d)
            raise self.printer.command_error(
                "Move exceeds maximum extrusion (%.3fmm^2 vs %.3fmm^2)\n"
                "See the 'max_extrude_cross_section' config option for details"
                % (area, self.max_extrude_ratio * self.filament_area))
        '''

    # Dummy function that always returns the max V-squared value
    def calc_junction(self, prev_move, move):
        diff_r = move.axes_r[3+3] - prev_move.axes_r[3+3]
        if diff_r:
            return (self.instant_corner_v / abs(diff_r))**2
        return move.max_cruise_v2
    
    def move(self, print_time, move):
        axis_r = move.axes_r[3+3]
        accel = move.accel * axis_r
        start_v = move.start_v * axis_r
        cruise_v = move.cruise_v * axis_r
        max_cruise_v =  move.max_cruise_v2 ** 0.5
        
        can_pressure_advance = False
        pwmmode = 1.0*(move.pwmmode or 0)
        pwmvalue = 1.0*(move.pwmvalue or 0)
        pwmsw = 1.0*(move.pwmsw or 0)
        power_type  = 0


        speed_pulse_ticks = self.cacl_step_dist_tick(max_cruise_v) or 1500
        # if (speed_pulse_ticks is None):
        #     speed_pulse_ticks = 1500         
               
        distance_count = 0
        len_powertable = len(move.power_table or [])
        if len_powertable > 0:
            if len_powertable % 2 != 0:
                len_powertable = 0
            elif len_powertable <= self.powertable_max:
                for i in range(len_powertable):
                    self.c_array[i] = min(move.power_table[i], 255)
                #self.c_array[:len_powertable] = move.power_table[:len_powertable]
                distmm = abs(move.axes_d[3+3])
                distance_count = self.cacl_distance_count(distmm)
                #logging.info("distance_count=%s, distance=%s, len_powertable=%s", distance_count, distmm, len_powertable)
                self.ptagcode = (self.ptagcode % 127) + 1
                power_type  = 1
                #logging.info("power_table_array=%s", self.c_array)
            else:
                len_powertable = 0
        current_ptagcode = self.ptagcode        

        #self.c_array
        #power_table_carray = self.c_array_NULL
        #power_table_ptr = ffi_main.from_buffer(power_table_carray)

        # if self.diff_pwmval > 0:
        #     if pwmvalue == 0:
        #         self.pre_pwmval = 0
        #     elif abs(pwmvalue - self.pre_pwmval) > self.diff_pwmval:
        #         self.pre_pwmval = pwmvalue
        #     else:
        #         pwmvalue = self.pre_pwmval
        if power_type == 0:
            if self.pre_ptagcode != self.ptagcode:
                self.pre_ptagcode = self.ptagcode
                self.psynccode = (self.psynccode % 127) + 1
        current_psynccode = self.psynccode
         

        self.trapq_append_extend(self.trapq, print_time,
                          move.accel_t, move.cruise_t, move.decel_t,
                          move.start_pos[3+3], 0., 0.,
                          0., 0., pwmsw,
                          1., can_pressure_advance, self._restartcmd_sn,
                          pwmmode, pwmvalue, speed_pulse_ticks,
                          start_v, cruise_v, accel, 1, 
                          self.power_table_ptr, len_powertable, distance_count, 
                          current_ptagcode, current_psynccode)
        
        self.last_position = move.end_pos[3+3]
        
    def find_past_position(self, print_time):
        if self.extruder_stepper is None:
            return 0.
        return self.extruder_stepper.find_past_position(print_time)

    cmd_TPWMSW_help = "TPWMSW ON PAUSERESUME"
    def cmd_default_TPWMSW(self, gcmd):
        extruder = self.printer.lookup_object('toolhead').get_extruder()
        if extruder.extruder_stepper is None:
            raise gcmd.error("Active laser does not have a stepper on tpwmsw")
        strapq = extruder.extruder_stepper.stepper.get_trapq()
        if strapq is not extruder.get_trapq():
            raise gcmd.error("Unable to infer active laser stepper on tpwmsw")
        extruder.cmd_TPWMSW(gcmd)
        #extruder.extruder_stepper.cmd_SET_PRESSURE_ADVANCE(gcmd)
        pass

    def cmd_TPWMSW(self, gcmd):
        pwm_prf = gcmd.get_int('S', 0, minval=0, maxval=1)
        self.set_pauseresume_pwm(pwm_prf)
        msg = "resp name=%s tpwmsw=%s " % (self.name, pwm_prf)            
        gcmd.respond_info(msg)  


    def cmd_M104(self, gcmd, wait=False):
        # Set Extruder Temperature
        temp = gcmd.get_float('S', 0.)
        index = gcmd.get_int('T', None, minval=0)
        if index is not None:
            section = 'extruder'
            if index:
                section = 'extruder%d' % (index,)
            extruder = self.printer.lookup_object(section, None)
            if extruder is None:
                if temp <= 0.:
                    return
                raise gcmd.error("Extruder not configured")
        else:
            extruder = self.printer.lookup_object('toolhead').get_extruder()
        #pheaters = self.printer.lookup_object('heaters')
        #pheaters.set_temperature(extruder.get_heater(), temp, wait)
    def cmd_M109(self, gcmd):
        # Set Extruder Temperature and Wait
        self.cmd_M104(gcmd, wait=True)
    cmd_ACTIVATE_EXTRUDER_help = "Change the active extruder"
    def cmd_ACTIVATE_EXTRUDER(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        if toolhead.get_extruder() is self:
            gcmd.respond_info("Extruder %s already active" % (self.name,))
            return
        gcmd.respond_info("Activating extruder %s" % (self.name,))
        toolhead.flush_step_generation()
        toolhead.set_extruder(self, self.last_position)
        self.printer.send_event("extruder:activate_extruder")

    cmd_ACTIVATE_LASER_help = "Change the active laser"
    def cmd_ACTIVATE_LASER(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        if toolhead.get_extruder() is self:
            gcmd.respond_info("LASER %s already active" % (self.name,))
            return
        gcmd.respond_info("Activating LASER %s" % (self.name,))
        toolhead.flush_step_generation()
        toolhead.set_extruder(self, self.last_position)
        self.printer.send_event("extruder:activate_extruder")        
        
def add_printer_objects(config):
    printer = config.get_printer()
    #logging.info("add_printer_objects") 
    for i in range(99):
        section = 'extruderpwm'
        if i:
            section = 'extruderpwm%d' % (i,)
        if not config.has_section(section):
            break
        pe = PrinterExtruderPWM(config.getsection(section), i)
        printer.add_object(section, pe)