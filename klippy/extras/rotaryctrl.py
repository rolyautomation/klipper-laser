# Rotary controller interface
#
# Copyright (C) 2025-2028  jinqiang <jinqiang@ecomedge.io>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

R0_IND = 0
R1_IND = 1
R2_IND = 2
R3_IND = 3
R0_DOACTION_YES = 0
R0_DOACTION_NO =  1
R1_DOACTION_YES = 2
R1_DOACTION_NO =  3
R2_DOACTION_YES = 4
R2_DOACTION_NO =  5
R3_DOACTION_YES = 6
R3_DOACTION_NO =  7


class RotaryCtrlInterface:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split(' ')[-1]
        self.last_rotarystate = [0, 0, 0, 0]

        #self.multimotor_existf = config.has_section('multimotor_axis')
        #if self.multimotor_existf :
        self.multimotor_axis_obj = None
        self.cur_selindex = -1

        #multimotor_axis_obj = self.printer.lookup_object('multimotor_axis')
        #    if multimotor_axis_obj is not None:
        #        z_min_st, z_max_st = multimotor_axis_obj.get_zlsw()   

        self.inside_handlegcode = False
        buttons = self.printer.load_object(config, "buttons")
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.r0_on_template = gcode_macro.load_template(config, 'r0_on_gcode', '')
        self.r0_off_template = gcode_macro.load_template(config,'r0_off_gcode', '')
        self.r1_on_template = gcode_macro.load_template(config, 'r1_on_gcode', '')
        self.r1_off_template = gcode_macro.load_template(config,'r1_off_gcode', '')
        self.r2_on_template = gcode_macro.load_template(config, 'r2_on_gcode', '')
        self.r2_off_template = gcode_macro.load_template(config,'r2_off_gcode', '')
        self.r3_on_template = gcode_macro.load_template(config, 'r3_on_gcode', '')
        self.r3_off_template = gcode_macro.load_template(config,'r3_off_gcode', '')
        self.abnormal_gcode = gcode_macro.load_template(config,'abnormal_gcode', '')
        
        self.gcode = self.printer.lookup_object('gcode')

        #self.gcode.register_mux_command("QUERY_ROTARY", "ROTARY", self.name,
        #                                self.cmd_QUERY_ROTARY,
        #                                desc=self.cmd_QUERY_ROTARY_help)
        self.gcode.register_command("QUERY_ROTARY", self.cmd_QUERY_ROTARY, desc=self.cmd_QUERY_ROTARY_help)  
        self.gcode.register_command("TEST_ROTARY", self.cmd_TEST_ROTARY)
                                              
        self.register_switchbutton(config, 'R0_pin', self.R0_callback)
        self.register_switchbutton(config, 'R1_pin', self.R1_callback)
        self.register_switchbutton(config, 'R2_pin', self.R2_callback)
        self.register_switchbutton(config, 'R3_pin', self.R3_callback)
        self.rotray_pkeystate = [0, 0, 0, 0]
        self.doaction_ins = 0

        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect_rotaryctrl)     


    def _handle_connect_rotaryctrl(self):
        self.multimotor_axis_obj = self.printer.lookup_object('multimotor_axis')
        self.update_rotary_selectindex()
        pass                                                   

    def update_rotary_selectindex(self):
        if self.multimotor_axis_obj is not None:
            self.cur_selindex = self.multimotor_axis_obj.get_curselindex()
        pass  

    def cmd_TEST_ROTARY(self, gcmd):
        inputnum_val  =  gcmd.get_int('N', 0, minval=0, maxval=3)
        set_val  =  gcmd.get_int('V', 0, minval=0, maxval=1)   
        self.last_rotarystate[inputnum_val] = set_val   
        gcmd.respond_info(self.name + ":[" + str(self.last_rotarystate[0]) + str(self.last_rotarystate[1]) + str(self.last_rotarystate[2]) + str(self.last_rotarystate[3]) + "]")  



    cmd_QUERY_ROTARY_help = "Report on the state of rotary controller"
    def cmd_QUERY_ROTARY(self, gcmd):
        gcmd.respond_info("rotary:[" + str(self.last_rotarystate[0]) + str(self.last_rotarystate[1]) + str(self.last_rotarystate[2]) + str(self.last_rotarystate[3]) + "]")  


    def key_event_handle(self, actionid, eventtime):
        if not self.inside_handlegcode :
            self.inside_handlegcode = True        
            if actionid == R0_DOACTION_YES:
                template = self.r0_on_template
            elif actionid == R0_DOACTION_NO:
                template = self.r0_off_template 
            elif actionid == R1_DOACTION_YES:
                template = self.r1_on_template
            elif actionid == R1_DOACTION_NO:
                template = self.r1_off_template
            elif actionid == R2_DOACTION_YES:
                template = self.r2_on_template
            elif actionid == R2_DOACTION_NO:
                template = self.r2_off_template
            elif actionid == R3_DOACTION_YES:
                template = self.r3_on_template
            elif actionid == R3_DOACTION_NO:
                template = self.r3_off_template
            else:
                template = self.abnormal_template
            try:
                self.gcode.run_script(template.render())
                #self.gcode.run_script_from_command(template.render())
            except:
                logging.exception("Script running error key event handle in rotary ctrl")
            self.inside_handlegcode = False            
        else:
            logging.exception("Script running repeat, check press speed")


    def R0_callback(self, eventtime, state):
        self.rotray_pkeystate[R0_IND] = state
        if state:
            self.last_rotarystate[R0_IND] = not self.last_rotarystate[R0_IND]
            actionid = self.R0_DOACTION_NO                
            if self.last_rotarystate[R0_IND]:
                actionid = self.R0_DOACTION_YES
            self.doaction_ins = actionid
            self.key_event_handle(actionid, eventtime)

       
    def R1_callback(self, eventtime, state):
        self.rotray_pkeystate[R1_IND] = state
        if state:
            self.last_rotarystate[R1_IND] = not self.last_rotarystate[R1_IND]
            actionid = self.R1_DOACTION_NO                
            if self.last_rotarystate[R1_IND]:
                actionid = self.R1_DOACTION_YES
            self.doaction_ins = actionid
            self.key_event_handle(actionid, eventtime)

    def R2_callback(self, eventtime, state):
        self.rotray_pkeystate[R2_IND] = state
        if state:
            self.last_rotarystate[R2_IND] = not self.last_rotarystate[R2_IND]
            actionid = self.R2_DOACTION_NO                
            if self.last_rotarystate[R2_IND]:
                actionid = self.R2_DOACTION_YES
            self.doaction_ins = actionid
            self.key_event_handle(actionid, eventtime)

        
    def R3_callback(self, eventtime, state):
        self.rotray_pkeystate[R3_IND] = state
        if state:
            self.last_rotarystate[R3_IND] = not self.last_rotarystate[R3_IND]
            actionid = self.R3_DOACTION_NO                
            if self.last_rotarystate[R3_IND]:
                actionid = self.R3_DOACTION_YES
            self.doaction_ins = actionid
            self.key_event_handle(actionid, eventtime)
            


    def register_switchbutton(self, config, name, callback, push_only=False):
        pin = config.get(name, None)
        if pin is None:
            return
        buttons = self.printer.lookup_object("buttons")
        if config.get('analog_range_' + name, None) is None:
            if push_only:
                buttons.register_button_push(pin, callback)
            else:
                buttons.register_buttons([pin], callback)
            return
        amin, amax = config.getfloatlist('analog_range_' + name, count=2)
        pullup = config.getfloat('analog_pullup_resistor', 4700., above=0.)
        if push_only:
            buttons.register_adc_button_push(pin, amin, amax, pullup, callback)
        else:
            buttons.register_adc_button(pin, amin, amax, pullup, callback)

               
    def get_status(self, eventtime=None):
        rotary_status = {}
        self.update_rotary_selectindex()
        rotary_status['rotary_status'] = [ self.last_rotarystate[0], self.last_rotarystate[1], self.last_rotarystate[2], self.last_rotarystate[3] ]
        rotary_status['rotary_cursel'] = self.cur_selindex
        return dict(rotary_status)


def load_config_prefix(config):
    return RotaryCtrlInterface(config)
