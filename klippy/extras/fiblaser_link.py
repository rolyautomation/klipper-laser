# Support fiber laser that are controlled 
#
# Copyright (C) 2024-2028  jinqiang <jinqiang@ecomedge.io>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging


PW_HCMD = 0x5A01
#test mode
#PW_HCMD = 0xA5A5

class FiberLaserLink:
    cmd_RCMD_FIBER_LASER_help = "run cmd mode pwms of a fiber laser"   
    cmd_QUERY_FIBER_LASER_help =  "Query flag of a fiber laser"   


    def __init__(self, config):
   
        self.printer = printer = config.get_printer()

        self.fiblaser_name = config.get_name().split()[-1]

        ppins = printer.lookup_object('pins')
        start_pin_params = ppins.lookup_pin(config.get('start_pin'))
        mcu = start_pin_params['chip']
        self._mcu = mcu
        self._start_pin = start_pin_params['pin']    


        self._sta_ee_em = config.getint('sta_ee_em', 5000, minval=5000, maxval=100000)
        self._sta_em_ee = config.getint('sta_em_ee', 1000, minval=1000, maxval=100000)   
        self._psyncpwm = config.getint('psyncpwm', 512, minval=2)  
        self._fiber_type = config.getint('type', 0, minval=0) 
        self.pulsewidth_en = config.getint('pulsewidth_en', 0, minval=0)          


        self._oid = self._mcu.create_oid()
        self._mcu.register_config_callback(self._build_config)

        gcode = self.printer.lookup_object("gcode")
        """
        gcode.register_mux_command("RCMD_FIBER_LASER", "FIBER",
                                   self.fiblaser_name,
                                   self.cmd_RCMD_FIBER_LASER,
                                   desc=self.cmd_RCMD_FIBER_LASER_help)          
        """
        gcode.register_command('RCMD_FIBER_LASER',
                                self.cmd_RCMD_FIBER_LASER,
                                desc=self.cmd_RCMD_FIBER_LASER_help)

        gcode.register_command('SET_PSYNC_PARAM',
                                self.cmd_SET_PSYNC_PARAM,
                                desc=self.cmd_SET_PSYNC_PARAM_help) 

        gcode.register_command('TEST_PSYNC_PARAM',
                                self.cmd_TEST_PSYNC_PARAM,
                                desc=self.cmd_TEST_PSYNC_PARAM_help)  

        gcode.register_command('SET_PULSEWIDTH',
                                self.cmd_SET_PULSEWIDTH,
                                desc=self.cmd_SET_PULSEWIDTH_help)

        gcode.register_command("QUERY_FIBER_LASER", self.cmd_QUERY_FIBER_LASER,
                               desc=self.cmd_QUERY_FIBER_LASER_help)
                                                          
 
        self._last_clock = 0
        self._lastprr_clock = 0
        self._lastpw_clock = 0        
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect_bind)



    def _handle_connect_bind(self):
            extruderpwm1 = self.printer.lookup_object('extruderpwm1')
            fiberlaser_oid =   self._oid
            mcu_fiberlaser =   self._mcu
            #pwm_oid = 0
            laser_type = 1
            extruderpwm1.set_extrdpwm_oid(fiberlaser_oid,mcu_fiberlaser,laser_type)


    def _build_config(self):
        # Setup config
        #self._mcu.add_config_cmd("config_stepper_fiber oid=%c start_pin=%c sta_ee_em=%u sta_em_ee=%u psyncpwm=%u type=%c"
        #                        % (self._oid, self._start_pin, self._sta_ee_em, self._sta_em_ee, self._psyncpwm, self._fiber_type))  
        self._mcu.add_config_cmd("config_stepper_fiber oid=%d start_pin=%s sta_ee_em=%u sta_em_ee=%u psyncpwm=%u type=%u"
                                 % (self._oid, self._start_pin, self._sta_ee_em, self._sta_em_ee, self._psyncpwm, self._fiber_type))                                                                     

        self._cmd_queue = self._mcu.alloc_command_queue()

        self._fiblaser_cmds_cmd = self._mcu.lookup_command(
            "queue_step_fiber oid=%c cmdmod=%c pwmv=%hu", cq=self._cmd_queue)  

        self._modify_psync_param_cmd = self._mcu.lookup_command(
            "modify_psync_param oid=%c psp=%u psd=%u", cq=self._cmd_queue)  

        self._modify_pulsewidth_param_cmd = self._mcu.lookup_command(
            "modify_pulsewidth_param oid=%c pch=%u pw=%u", cq=self._cmd_queue)              

        self._getfiberst_cmds_cmd = self._mcu.lookup_query_command(
            "stepper_get_fiber oid=%c",
            "stepper_fiber oid=%c flag=%i", oid=self._oid,
            cq=self._cmd_queue)

                                      

    def get_mcu(self):
        return self._mcu


    def cmd_RCMD_FIBER_LASER(self, gcmd):

        rcmd_mode  = gcmd.get_int('M', None, minval=0, maxval=100)
        rcmd_pwm  =  gcmd.get_int('S', None, minval=0, maxval=1000)
        v = float(rcmd_pwm)
        if v <= 0.:
            v = 0.
        if v >= 1000.:
            v = 1000. 
        v = v/1000.0 * 255   

        self._runmode = rcmd_mode
        self._pwm_sval = int(round(v)) 

        #time_clock = self._mcu.print_time_to_clock(1.0)   
        #logging.info("RCMD_FIBER_LASER M=%d,S=%d,tk=%d", self._runmode, self._pwm_sval, time_clock)
        
        logging.info("RCMD_FIBER_LASER M=%d,S=%d", self._runmode, self._pwm_sval)
        # Obtain print_time and apply requested settings
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_lookahead_callback(
            lambda print_time: self._run_cmd_send(print_time, self._runmode, self._pwm_sval))
        #self._run_cmd_send(, self._runmode, self._pwm_sval)

    #125M/3 = 41.6MHZ    41600/62 = 671   41600/27=1541
    #prr 27~62 khz, duty:0.1~0.9
    cmd_SET_PSYNC_PARAM_help =  "set psync param  PRR"   
    M_BASEHZ = 41600
    M_PRR_MINHZ = 1541
    M_PRR_MAXHZ = 671
    def cmd_SET_PSYNC_PARAM(self, gcmd):
        psyncp  = gcmd.get_int('P', 833, minval=1)
        #psyncduty  =  gcmd.get_int('L', 416, minval=1)
        psyncdutyf = gcmd.get_float('LF', 0.5, minval=0.1, maxval=0.9) 

        msgh = "PRR:"
        #if psyncp < self.M_PRR_MINHZ and psyncp > self.M_PRR_MAXHZ:
        if psyncp > 0:
           psyncduty = int(round(psyncp*psyncdutyf+0.5))
           toolhead = self.printer.lookup_object('toolhead')
           toolhead.register_lookahead_callback(
               lambda print_time: self._run_prr_send(print_time, psyncp, psyncduty))
           gcmd.respond_info(msgh + "SET_PSYNC_PARAM, P=%d,LF=%d" % (psyncp, psyncduty))

        else:
           gcmd.respond_info(msgh + "input psync period error, expect [%d~%d]" % (self.M_PRR_MINHZ, self.M_PRR_MAXHZ))        


    cmd_TEST_PSYNC_PARAM_help =  "test psync param  PRR"   
    def cmd_TEST_PSYNC_PARAM(self, gcmd):
        psyncp  = gcmd.get_int('P', 833, minval=1)
        #psyncduty  =  gcmd.get_int('L', 416, minval=1)
        psyncdutyf = gcmd.get_float('LF', 0.5, minval=0.0, maxval=1) 
        msgh = "TESTPRR:"
        if psyncp > 0:
           psyncduty = int(round(psyncp*psyncdutyf+0.5))
           toolhead = self.printer.lookup_object('toolhead')
           toolhead.register_lookahead_callback(
               lambda print_time: self._run_prr_send(print_time, psyncp, psyncduty))
           gcmd.respond_info(msgh + "TEST_PSYNC_PARAM, P=%d,LF=%d" % (psyncp, psyncduty))

        else:
           gcmd.respond_info(msgh + "input psync period error:[%d]" % (psyncp,))  


    cmd_SET_PULSEWIDTH_help =  "set pulsewidth param"   
    def cmd_SET_PULSEWIDTH(self, gcmd):
        pulsewidth  = gcmd.get_int('PW', 100, minval=1, maxval=1000)
        msgh = "PulseWidth:"
        if self.pulsewidth_en > 0:
            pwhcmd = PW_HCMD
            toolhead = self.printer.lookup_object('toolhead')
            toolhead.register_lookahead_callback(
                lambda print_time: self._run_pulsewidth_send(print_time, pwhcmd, pulsewidth))
            gcmd.respond_info(msgh + "SET_PULSEWIDTH, PW=%d" % (pulsewidth))

        else:
           gcmd.respond_info(msgh + "please config pulsewidth_en")  

    def _run_pulsewidth_send(self, print_time, pwhcmd,pulsewidth):
        clock = self._mcu.print_time_to_clock(print_time)
        self._modify_pulsewidth_param_cmd.send([self._oid, pwhcmd, pulsewidth],
                           minclock=self._lastpw_clock, reqclock=clock)
        self._lastpw_clock = clock 

    def _run_prr_send(self, print_time, psyncp, psyncduty):
        clock = self._mcu.print_time_to_clock(print_time)
        self._modify_psync_param_cmd.send([self._oid, psyncp, psyncduty],
                           minclock=self._lastprr_clock, reqclock=clock)
        self._lastprr_clock = clock 
        #logging.info("SET_PSYNC_PARAM, P=%d,LF=%d" % (psyncp, psyncduty))


    def _run_cmd_send(self, print_time, runmode, pwm_sval):
        clock = self._mcu.print_time_to_clock(print_time)
        self._fiblaser_cmds_cmd.send([self._oid, runmode, pwm_sval],
                           minclock=self._last_clock, reqclock=clock)
        self._last_clock = clock     


    def cmd_QUERY_FIBER_LASER(self, gcmd):
        self._flag_rd = 0
        self._get_fiberstatus_send()
        hex_flag = hex(self._flag_rd)
        msg = "FLAG=%s in FIBER" % (hex_flag)
        gcmd.respond_raw(msg)

    def _get_fiberstatus_send(self):
        params = self._getfiberst_cmds_cmd.send([self._oid])
        self._flag_rd = params['flag']


def load_config_prefix(config):
    return FiberLaserLink(config)


