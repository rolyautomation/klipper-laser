# Support limit switch check 
#
# Copyright (C) 2024-2028  jinqiang <jinqiang@ecomedge.io>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

X_MIN_IND = 0
X_MAX_IND = 1
Y_MIN_IND = 2
Y_MAX_IND = 3
Z_MIN_IND = 4
Z_MAX_IND = 5


class LimitSwitchCheck:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split(' ')[-1]
        #self.switch_enalbe = [1, 1, 1, 1, 1, 1]
        self.switch_enalbe = [0, 0, 0, 0, 0, 0]
        self.last_state = [0, 0, 0, 0, 0, 0]
        self.inside_handlegcode = False
        buttons = self.printer.load_object(config, "buttons")
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.press_template = gcode_macro.load_template(config, 'press_gcode', '')
        self.release_template = gcode_macro.load_template(config,
                                                          'release_gcode', '')
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_mux_command("QUERY_LSWT", "LSWT", self.name,
                                        self.cmd_QUERY_LSWT,
                                        desc=self.cmd_QUERY_LSWT_help)

        self.register_switchbutton(config, 'x_min_pin', self.x_min_callback)
        self.register_switchbutton(config, 'x_max_pin', self.x_max_callback)
        self.register_switchbutton(config, 'y_min_pin', self.y_min_callback)
        self.register_switchbutton(config, 'y_max_pin', self.y_max_callback)
        self.register_switchbutton(config, 'z_min_pin', self.z_min_callback)
        self.register_switchbutton(config, 'z_max_pin', self.z_max_callback)  

        '''
        self.printer.register_event_handler("homing:home_rails_begin",
                                            self._handle_home_rails_begin)
        self.printer.register_event_handler("homing:home_rails_end",
                                            self._handle_home_rails_end)
        '''                                    

        self.gcode.register_command("OPEN_LSWCHECK", self.cmd_OPEN_LSWCHECK)                                            

    def cmd_OPEN_LSWCHECK(self, gcmd):
        en_flag  =  gcmd.get_int('E', 1, minval=0, maxval=10)
        if (en_flag > 0):
            if  (en_flag > 3):
                self.switch_enalbe[X_MIN_IND] = 1
                self.switch_enalbe[Y_MIN_IND] = 1
                self.switch_enalbe[Z_MIN_IND] = 0 
                self.switch_enalbe[X_MAX_IND] = 1
                self.switch_enalbe[Y_MAX_IND] = 1
                self.switch_enalbe[Z_MAX_IND] = 0                 
            else:    
                self.switch_enalbe[X_MIN_IND] = 1
                self.switch_enalbe[Y_MIN_IND] = 1
                self.switch_enalbe[Z_MIN_IND] = 1 
                self.switch_enalbe[X_MAX_IND] = 1
                self.switch_enalbe[Y_MAX_IND] = 1
                self.switch_enalbe[Z_MAX_IND] = 1   
        else:
            self.switch_enalbe[X_MIN_IND] = 0
            self.switch_enalbe[Y_MIN_IND] = 0
            self.switch_enalbe[Z_MIN_IND] = 0 
            self.switch_enalbe[X_MAX_IND] = 0
            self.switch_enalbe[Y_MAX_IND] = 0
            self.switch_enalbe[Z_MAX_IND] = 0

    def get_zlsw(self):
        return self.last_state[Z_MIN_IND], self.last_state[Z_MAX_IND]
       

    '''                          
    def _handle_home_rails_begin(self, homing_state, rails):
        self.switch_enalbe[X_MIN_IND] = 0
        self.switch_enalbe[Y_MIN_IND] = 0
        self.switch_enalbe[Z_MIN_IND] = 0   

    def _handle_home_rails_end(self, homing_state, rails):
        self.switch_enalbe[X_MIN_IND] = 1
        self.switch_enalbe[Y_MIN_IND] = 1
        self.switch_enalbe[Z_MIN_IND] = 1       
    '''    

    def set_limitswitch_enable(self, ind, val):
        self.switch_enalbe[ind] = val

    def x_min_callback(self, eventtime, state):
        self.last_state[X_MIN_IND] = state
        self.lswt_callback(eventtime)
        
    def x_max_callback(self, eventtime, state):
        self.last_state[X_MAX_IND] = state
        self.lswt_callback(eventtime)

    def y_min_callback(self, eventtime, state):
        self.last_state[Y_MIN_IND] = state
        self.lswt_callback(eventtime)
        
    def y_max_callback(self, eventtime, state):
        self.last_state[Y_MAX_IND] = state
        self.lswt_callback(eventtime)

    def z_min_callback(self, eventtime, state):
        self.last_state[Z_MIN_IND] = state
        self.lswt_callback(eventtime)
        
    def z_max_callback(self, eventtime, state):
        self.last_state[Z_MAX_IND] = state
        self.lswt_callback(eventtime)

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

    def lswt_callback(self, eventtime):
        state = 0
        for index, value in enumerate(self.switch_enalbe):
            if value and self.last_state[index] :
                state = state + 1
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
            logging.exception("Script running repeat, limit switch fast")    


    cmd_QUERY_LSWT_help = "Report on the state of limit switch"
    def cmd_QUERY_LSWT(self, gcmd):
        resst = self.get_status()
        x_min_str = "[x_min="
        if resst['x_min']:
           x_min_str = x_min_str + "PRESSED]" 
        else:  
           x_min_str = x_min_str + "RELEASED]"   
        x_max_str = "[x_max="
        if resst['x_max']:
           x_max_str = x_max_str + "PRESSED]" 
        else:  
           x_max_str = x_max_str + "RELEASED]"  

        y_min_str = "[y_min="
        if resst['y_min']:
           y_min_str = y_min_str + "PRESSED]" 
        else:  
           y_min_str = y_min_str + "RELEASED]"   
        y_max_str = "[y_max="
        if resst['y_max']:
           y_max_str = y_max_str + "PRESSED]" 
        else:  
           y_max_str = y_max_str + "RELEASED]"            

        z_min_str = "[z_min="
        if resst['z_min']:
           z_min_str = z_min_str + "PRESSED]" 
        else:  
           z_min_str = z_min_str + "RELEASED]"   
        z_max_str = "[z_max="
        if resst['z_max']:
           z_max_str = z_max_str + "PRESSED]" 
        else:  
           z_max_str = z_max_str + "RELEASED]"     
        gcmd.respond_info(self.name + ": " + x_min_str + x_max_str + y_min_str + y_max_str + z_min_str + z_max_str)                

    def get_status(self, eventtime=None):
        lsw_status = {}
        lsw_status['x_min'] = self.last_state[X_MIN_IND]
        lsw_status['x_max'] = self.last_state[X_MAX_IND]
        lsw_status['y_min'] = self.last_state[Y_MIN_IND]
        lsw_status['y_max'] = self.last_state[Y_MAX_IND]
        lsw_status['z_min'] = self.last_state[Z_MIN_IND]
        lsw_status['z_max'] = self.last_state[Z_MAX_IND]        
        return dict(lsw_status)


def load_config_prefix(config):
    return LimitSwitchCheck(config)
