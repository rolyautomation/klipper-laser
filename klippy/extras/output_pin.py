# PWM and digital output pin handling
#
# Copyright (C) 2017-2024  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

PIN_MIN_TIME = 0.100
RESEND_HOST_TIME = 0.300 + PIN_MIN_TIME
MAX_SCHEDULE_TIME = 5.0

class PrinterOutputPin:
    def __init__(self, config):
        self.printer = config.get_printer()
        ppins = self.printer.lookup_object('pins')
        # Determine pin type
        self.cycle_mattr = config.getint('cyclemf', 0, minval=0)
        self.is_pwm = config.getboolean('pwm', False)
        if self.is_pwm:
            self.mcu_pin = ppins.setup_pin('pwm', config.get('pin'))
            cycle_time = config.getfloat('cycle_time', 0.100, above=0.,
                                         maxval=MAX_SCHEDULE_TIME)
            self.last_cycle_time = self.default_cycle_time = cycle_time
            hardware_pwm = config.getboolean('hardware_pwm', False)
            self.mcu_pin.setup_cycle_time(cycle_time, hardware_pwm)
            self.scale = config.getfloat('scale', 1., above=0.)
        else:
            self.mcu_pin = ppins.setup_pin('digital_out', config.get('pin'))
            self.scale = 1.
        self.last_print_time = 0.
        # Support mcu checking for maximum duration
        self.reactor = self.printer.get_reactor()
        self.resend_timer = None
        self.resend_interval = 0.
        max_mcu_duration = config.getfloat('maximum_mcu_duration', 0.,
                                           minval=0.500,
                                           maxval=MAX_SCHEDULE_TIME)
        self.mcu_pin.setup_max_duration(max_mcu_duration)
        if max_mcu_duration:
            config.deprecate('maximum_mcu_duration')
            self.resend_interval = max_mcu_duration - RESEND_HOST_TIME
        # Determine start and shutdown values
        static_value = config.getfloat('static_value', None,
                                       minval=0., maxval=self.scale)
        if static_value is not None:
            config.deprecate('static_value')
            self.last_value = self.shutdown_value = static_value / self.scale
        else:
            self.last_value = config.getfloat(
                'value', 0., minval=0., maxval=self.scale) / self.scale
            self.shutdown_value = config.getfloat(
                'shutdown_value', 0., minval=0., maxval=self.scale) / self.scale
        self.mcu_pin.setup_start_value(self.last_value, self.shutdown_value)
        # Register commands
        pin_name = config.get_name().split()[1]

        self.pwm_oid = None
        self.mcu_pwm = None
        self._last_clock = self._cycle_ticks = 0  
        self._set_pwm_out_cycle = None  
        self.last_update_flag = 0

        self._epwm_name =  pin_name
        if self._epwm_name == 'extrdpwm':
            self._mcu = self.mcu_pin.get_mcu()
            self._mcu.register_config_callback(self._build_config)
            self.printer.register_event_handler("klippy:connect",
                                                self._handle_connect_bind)

            #extruderpwm = self.printer.lookup_object('extruderpwm')
            #pwm_oid =  self.mcu_pin.get_mcu_pwm_oid()
            #pwm_oid = 0
            #extruderpwm.set_extrdpwm_oid(pwm_oid)
            #logging.info("bind pwm: %s %i", self._epwm_name, pwm_oid) 


        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command("SET_PIN", "PIN", pin_name,
                                   self.cmd_SET_PIN,
                                   desc=self.cmd_SET_PIN_help)
        gcode.register_mux_command("SET_PIN_CYCLE", "PIN", pin_name,
                                   self.cmd_SET_PIN_CYCLE,
                                   desc=self.cmd_SET_PIN_CYCLE_help)   
      
    def _build_config(self): 
        if self.cycle_mattr > 0: 
            cmd_queue = self._mcu.alloc_command_queue()
            curtime = self._mcu.get_printer().get_reactor().monotonic()
            printtime = self._mcu.estimated_print_time(curtime)
            self._last_clock = self._mcu.print_time_to_clock(printtime + 0.200)        
            self._set_pwm_out_cycle = self._mcu.lookup_command(
            "set_pwm_out_cycle oid=%c cycle_ticks=%u value=%hu flag=%hu", cq=cmd_queue)
            self._pwm_max = self._mcu.get_constant_float("PWM_MAX")



    def _handle_connect_bind(self):
        if self._epwm_name == 'extrdpwm':
            extruderpwm = self.printer.lookup_object('extruderpwm')
            self.pwm_oid = pwm_oid =  self.mcu_pin.get_mcu_pwm_oid()
            self.mcu_pwm = mcu_pwm =  self.mcu_pin.get_mcu()
            #pwm_oid = 0
            laser_type = 0
            extruderpwm.set_extrdpwm_oid(pwm_oid,mcu_pwm,laser_type)
            #logging.info("bind pwm: %s %s %i", mcu_pwm, self._epwm_name, pwm_oid)   

    def _set_pin_cycle(self, print_time, value, cycle_time, update_flag):
        #if value == self.last_value and cycle_time == self.last_cycle_time:
        #if cycle_time == self.last_cycle_time:
        if value == self.last_value and cycle_time == self.last_cycle_time and update_flag == self.last_update_flag:
            return
        print_time = max(print_time, self.last_print_time + PIN_MIN_TIME)
        clock = self.mcu_pwm.print_time_to_clock(print_time)
        minclock = self._last_clock        
        #self.mcu_pin.set_pwm_cycle(print_time, value, cycle_time)
        # Send updated cycle_time if necessary
        cycle_ticks = self.mcu_pwm.seconds_to_clock(cycle_time)
        #if cycle_ticks != self._cycle_ticks:
        if cycle_ticks >= 1<<31:
            raise self.mcu_pwm.get_printer().command_error(
                "PWM cycle time too large")
        v = int(max(0., min(1., value)) * self._pwm_max + 0.5)        
        self._set_pwm_out_cycle.send([self.pwm_oid, cycle_ticks, v, update_flag],
                                    minclock=minclock, reqclock=clock)
        self._cycle_ticks = cycle_ticks

        self._last_clock = clock
        if update_flag > 0:
            self.last_value = value
        self.last_cycle_time = cycle_time
        self.last_print_time = print_time
        self.last_update_flag = update_flag   


    cmd_SET_PIN_CYCLE_help = "Set the value and cycle time of an output pin"
    def cmd_SET_PIN_CYCLE(self, gcmd):
        # Read requested value
        if  self.cycle_mattr == 0:
            gcmd.respond_info( "please set cyclemf is 1")
            return        
        if self._epwm_name == 'extrdpwm':
            v = int(max(0., min(1., self.last_value)) * self._pwm_max + 0.5)
            #logging.info("extrdpwm set:%s %s %s %s %s", self.last_cycle_time, self.last_value, self.last_update_flag, v, self._cycle_ticks) 
            update_flag  =  gcmd.get_int('FLAG', 0, minval=0, maxval=10)
            value = gcmd.get_float('VALUE', 0., minval=0., maxval=self.scale)
            value /= self.scale
            cycle_time = gcmd.get_float('CYCLE_TIME', self.default_cycle_time,
                                        above=0., maxval=MAX_SCHEDULE_TIME)
            # Obtain print_time and apply requested settings
            toolhead = self.printer.lookup_object('toolhead')
            toolhead.register_lookahead_callback(
                lambda print_time: self._set_pin_cycle(print_time, value, cycle_time, update_flag)) 
            #gcmd.respond_info( "SET_PIN_CYCLE pin=extrdpwm value=%s cycle_time=%s flag=%s" % (value, cycle_time, update_flag))
        else:
            gcmd.respond_info( "only support pin=extrdpwm on SET_PIN_CYCLE") 


    def get_status(self, eventtime):
        return {'value': self.last_value}
    def _set_pin(self, print_time, value, is_resend=False):
        if value == self.last_value and not is_resend:
            return
        print_time = max(print_time, self.last_print_time + PIN_MIN_TIME)
        if self.is_pwm:
            self.mcu_pin.set_pwm(print_time, value)
        else:
            self.mcu_pin.set_digital(print_time, value)
        self.last_value = value
        self.last_print_time = print_time
        if self.resend_interval and self.resend_timer is None:
            self.resend_timer = self.reactor.register_timer(
                self._resend_current_val, self.reactor.NOW)
    cmd_SET_PIN_help = "Set the value of an output pin"
    def cmd_SET_PIN(self, gcmd):
        # Read requested value
        value = gcmd.get_float('VALUE', minval=0., maxval=self.scale)
        value /= self.scale
        if not self.is_pwm and value not in [0., 1.]:
            raise gcmd.error("Invalid pin value")
        # Obtain print_time and apply requested settings
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_lookahead_callback(
            lambda print_time: self._set_pin(print_time, value))

    def _resend_current_val(self, eventtime):
        if self.last_value == self.shutdown_value:
            self.reactor.unregister_timer(self.resend_timer)
            self.resend_timer = None
            return self.reactor.NEVER

        systime = self.reactor.monotonic()
        print_time = self.mcu_pin.get_mcu().estimated_print_time(systime)
        time_diff = (self.last_print_time + self.resend_interval) - print_time
        if time_diff > 0.:
            # Reschedule for resend time
            return systime + time_diff
        self._set_pin(print_time + PIN_MIN_TIME, self.last_value, True)
        return systime + self.resend_interval

def load_config_prefix(config):
    return PrinterOutputPin(config)
