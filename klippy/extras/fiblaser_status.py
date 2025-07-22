# check fiber laser status 
#
# Copyright (C) 2025-2028  jinqiang <jinqiang@ecomedge.io>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

#pin13
A0_IND = 0
#pin20
A1_IND = 1
#pin11
A2_IND = 2
#pin12
A3_IND = 3
#GPIO16,!GPIO19,GPIO18,GPIO17

class FiberLaserStatusChk:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split(' ')[-1]

        self.error_num = 0
        self.last_prestate = [0, 0, 0, 0]
        self.last_state = [0, 0, 0, 0]
        self.inside_handlegcode = False
        buttons = self.printer.load_object(config, "buttons")
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.press_template = gcode_macro.load_template(config, 'press_gcode', '')
        self.release_template = gcode_macro.load_template(config,
                                                          'release_gcode', '')

        self.gcode = self.printer.lookup_object('gcode')

        self.gcode.register_command("QUERY_FIBLASER_STATUS", self.cmd_QUERY_FIBLASER_STATUS, 
                                        desc=self.cmd_QUERY_FIBLASER_STATUS_help) 

        #self.checkf_en = config.getint('checkf_en', 1) 

        self.register_switchbutton(config, 'a0_pin', self.a0_callback)
        self.register_switchbutton(config, 'a1_pin', self.a1_callback)
        self.register_switchbutton(config, 'a2_pin', self.a2_callback)
        self.register_switchbutton(config, 'a3_pin', self.a3_callback)    
        self.fiblaser_status_en = 0     
        self.gcode.register_command("OPEN_FIBLASER_STATUS_CHECK", self.cmd_OPEN_FIBLASER_STATUS_CHECK)  


    def cmd_OPEN_FIBLASER_STATUS_CHECK(self, gcmd):
        en_flag  =  gcmd.get_int('S', 0, minval=0, maxval=10)
        if (en_flag > 0):
            if  (en_flag == 1):
               self.fiblaser_status_en = 1
        else:
            self.fiblaser_status_en = 0 
        gcmd.respond_info( "fiblaser status check sw:" + str(self.fiblaser_status_en)) 

        


    def a0_callback(self, eventtime, state):
        self.last_state[A0_IND] = state
        self.fiberlaser_callback(eventtime)
        
    def a1_callback(self, eventtime, state):
        self.last_state[A1_IND] = state
        self.fiberlaser_callback(eventtime)

    def a2_callback(self, eventtime, state):
        self.last_state[A2_IND] = state
        self.fiberlaser_callback(eventtime)
        
    def a3_callback(self, eventtime, state):
        self.last_state[A3_IND] = state
        self.fiberlaser_callback(eventtime)

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


    def fiberlaser_callback(self, eventtime):
        state = 0
        if self.last_prestate[0] != self.last_state[0]:
            state = state + 1
        if self.last_prestate[1] != self.last_state[1]:
            state = state + 2
        if self.last_prestate[2] != self.last_state[2]:
            state = state + 4
        if self.last_prestate[3] != self.last_state[3]:
            state = state + 8

        error_num = 0    
        if self.last_state[0] :  
             error_num = 1
        if self.last_state[1] :  
             error_num = error_num + 2
        if self.last_state[2] :  
             error_num = error_num + 4
        if self.last_state[3] :  
             error_num = error_num + 8
        self.error_num = error_num    
        self.last_prestate = self.last_state.copy()   

        if self.fiblaser_status_en == 0:
            return   

        if not self.inside_handlegcode :
            self.inside_handlegcode = True
            if  state:
                template = self.press_template
                try:
                    self.gcode.run_script(template.render())
                except:
                    logging.exception("Script running error")
            self.inside_handlegcode = False
        else:
            logging.exception("Script running repeat, fiberlaser status")  
             

    cmd_QUERY_FIBLASER_STATUS_help = "Report on the state of fiber laser"
    def cmd_QUERY_FIBLASER_STATUS(self, gcmd):
        pin13_str = " pin13:"
        if self.last_state[A0_IND]:
           pin13_str = pin13_str + "H" 
        else:  
           pin13_str = pin13_str + "L"    
        pin20_str = " pin20:"
        if self.last_state[A1_IND]:
           pin20_str = pin20_str + "L"  
        else:  
           pin20_str = pin20_str + "H"   
        pin11_str = " pin11:"
        if self.last_state[A2_IND]:
           pin11_str = pin11_str + "H" 
        else:  
           pin11_str = pin11_str + "L"   
        pin12_str = " pin12:"
        if self.last_state[A3_IND]:
           pin12_str = pin12_str + "H"  
        else:  
           pin12_str = pin12_str + "L"   
        msg_st = "fali status:[" + str(self.error_num) + "]"
        if  not self.last_state[A0_IND]  and not self.last_state[A1_IND]:
            msg_st = "success status:[0]"
        gcmd.respond_info("st_" + msg_st +  pin13_str + pin20_str + pin11_str + pin12_str)    


    def get_status(self, eventtime=None):
        fiblaser_status = {}
        fiblaser_status['st_pin13'] = self.last_state[A0_IND]
        fiblaser_status['st_pin20'] = self.last_state[A1_IND]
        fiblaser_status['st_pin11'] = self.last_state[A2_IND]
        fiblaser_status['st_pin12'] = self.last_state[A3_IND]
        fiblaser_status['insst'] = self.error_num        
        return dict(fiblaser_status)


#def load_config_prefix(config):
def load_config(config):
    return FiberLaserStatusChk(config)
