# Support galvo that are controlled 
#
# Copyright (C) 2024-2028  jinqiang <jinqiang@ecomedge.io>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#from . import fan

class GalvoConfigCtrl:
    #cmd_SET_GALVO_POS_help = "Sets the pos of a galvo"
    cmd_SET_POS_GALVO_help = "Sets the pos x y of a galvo"   
    cmd_QUERY_POS_GALVO_help =  "Query the pos x y of a galvo"   
    cmd_QUERY_POS_GALVO_VIR_help =  "Query the pos x y of a galvo vir"  

    def __init__(self, config):
   
        self.printer = printer = config.get_printer()

        self.galvo_name = config.get_name().split()[-1]

        ppins = printer.lookup_object('pins')
        clock_pin_params = ppins.lookup_pin(config.get('clkb_pin'))
        data_pin_params = ppins.lookup_pin(config.get('xyb_pin'))

        mcu = data_pin_params['chip']
        if mcu is not clock_pin_params['chip']:
            raise config.error("Galvo pins must be on same mcu")

        self._mcu = mcu
        self._clock_pin = clock_pin_params['pin']    
        self._data_pin = data_pin_params['pin']


        self._x_pos = config.getint('x_pos', 0, minval=0, maxval=65535)
        self._y_pos = config.getint('y_pos', 0, minval=0, maxval=65535)   
        self._coord_factor = config.getint('coord_factor', 1, minval=1)   
        #self._mode = config.getint('mode', 0)   

        self._oid = self._mcu.create_oid()
        self._mcu.register_config_callback(self._build_config)

        gcode = self.printer.lookup_object("gcode")
        """
        gcode.register_mux_command("SET_POS_GALVO", "GALVO",
                                   self.galvo_name,
                                   self.cmd_SET_POS_GALVO,
                                   desc=self.cmd_SET_POS_GALVO_help)          
        """
        gcode.register_command('SET_POS_GALVO',
                                self.cmd_SET_POS_GALVO,
                                desc=self.cmd_SET_POS_GALVO_help)

        gcode.register_command("QUERY_POS_GALVO", self.cmd_QUERY_POS_GALVO,
                               desc=self.cmd_QUERY_POS_GALVO_help)

        gcode.register_command("QUERY_POS_GALVO_VIR", self.cmd_QUERY_POS_GALVO_VIR,
                               desc=self.cmd_QUERY_POS_GALVO_VIR_help)                               
                                                               
        self._last_clock = 0
        self._x_pos_rd = 0
        self._y_pos_rd = 0

        self._x_pos_rd_vir = 0
        self._y_pos_rd_vir = 0        

    def _build_config(self):
        # Setup config
        #self._mcu.add_config_cmd("config_xy2_stepper oid=%c clkb_pin=%c xb_pin=%c x_pos=%hu y_pos=%hu"
                                # % (self._oid, self._clock_pin, self._data_pin,self._x_pos,self._y_pos))   
        #self._mcu.add_config_cmd("config_xy2_stepper oid=%d clkb_pin=%s xb_pin=%s x_pos=%u y_pos=%u"
                                # % (self._oid, self._clock_pin, self._data_pin,self._x_pos,self._y_pos))  
        self._mcu.add_config_cmd("config_xy2_stepper oid=%d clkb_pin=%s xb_pin=%s x_pos=%u y_pos=%u coord_factor=%u"
                                 % (self._oid, self._clock_pin, self._data_pin,self._x_pos,self._y_pos,self._coord_factor))                                     

        self._cmd_queue = self._mcu.alloc_command_queue()

        self._setxy_cmds_cmd = self._mcu.lookup_command(
            "xy2_set_position oid=%c x_pos=%hu y_pos=%hu", cq=self._cmd_queue)  


        self._getxy_cmds_cmd = self._mcu.lookup_query_command(
            "xy2_stepper_get_position oid=%c",
            "xy2_stepper_position oid=%c xpos=%hu ypos=%hu", oid=self._oid,
            cq=self._cmd_queue)

        self._getxy_cmds_cmd_vir = self._mcu.lookup_query_command(
            "xy2_vir_get_position oid=%c",
            "xy2_vir_position oid=%c xvpos=%u yvpos=%u", oid=self._oid,
            cq=self._cmd_queue)            
                                          

    def get_mcu(self):
        return self._mcu



    def cmd_SET_POS_GALVO(self, gcmd):
        self._x_pos = gcmd.get_int('X', 0, minval=0, maxval=65535)
        self._y_pos = gcmd.get_int('Y', 0, minval=0, maxval=65535)
        #logging.info("SET_POS_GALVO X=%d,Y=%d", x, y)

        # Obtain print_time and apply requested settings
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_lookahead_callback(
            lambda print_time: self._set_xy_send(print_time, self._x_pos,self._y_pos))
        #self.set_xy_send(,self._x_pos,self._y_pos)
        

    def _set_xy_send(self, print_time, xp, yp):
        clock = self._mcu.print_time_to_clock(print_time)
        self._setxy_cmds_cmd.send([self._oid, xp, yp],
                           minclock=self._last_clock, reqclock=clock)
        self._last_clock = clock     

    def cmd_QUERY_POS_GALVO(self, gcmd):
        self._get_xy_send()
        msg = "X=%d Y=%d in GALVO" % (self._x_pos_rd, self._y_pos_rd)
        gcmd.respond_raw(msg)

    def _get_xy_send(self):
        params = self._getxy_cmds_cmd.send([self._oid])
        self._x_pos_rd = params['xpos']
        self._y_pos_rd = params['ypos']

    def cmd_QUERY_POS_GALVO_VIR(self, gcmd):
        self._get_xy_send_vir()
        msg = "vX=%d vY=%d in GALVO" % (self._x_pos_rd_vir, self._y_pos_rd_vir)
        gcmd.respond_raw(msg)

    def _get_xy_send_vir(self):
        params = self._getxy_cmds_cmd_vir.send([self._oid])
        self._x_pos_rd_vir = params['xvpos']
        self._y_pos_rd_vir = params['yvpos']        



def load_config_prefix(config):
    return GalvoConfigCtrl(config)


