# Support z axis move by key  
#
# Copyright (C) 2024-2028  jinqiang <jinqiang@ecomedge.io>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

LONG_PRESS_DURATION = 0.400
#LONG_PRESS_DURATION = 1.800

class ZctrlPanel:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()        
        self.name = config.get_name().split(' ')[-1]
        self.up_state = 0
        self.down_state = 0
        self.inside_handlegcode = False
        buttons = self.printer.load_object(config, "buttons")
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.abnormal_template = gcode_macro.load_template(config, 'abnormal_gcode', '')

        self.uppress_template = gcode_macro.load_template(config, 'uppress_gcode', '')
        self.uprelease_template = gcode_macro.load_template(config,
                                                          'uprelease_gcode', '')
        self.uplngp_template = gcode_macro.load_template(config, 'uplngp_gcode', '')
        self.uplngr_template = gcode_macro.load_template(config,
                                                          'uplngr_gcode', '')    

        self.downpress_template = gcode_macro.load_template(config, 'downpress_gcode', '')
        self.downrelease_template = gcode_macro.load_template(config,
                                                          'downrelease_gcode', '')
        self.downlngp_template = gcode_macro.load_template(config, 'downlngp_gcode', '')
        self.downlngr_template = gcode_macro.load_template(config,
                                                          'downlngr_gcode', '')                                                               

        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_mux_command("QUERY_ZCTRL", "ZCTRL", self.name,
                                        self.cmd_QUERY_ZCTRL,
                                        desc=self.cmd_QUERY_ZCTRL_help)


        self.is_short_upclick = False
        self.is_long_upclick = False
        self.upclick_timer = self.reactor.register_timer(self.long_upclick_event)
        self.register_zbutton(config, 'up_pin', self.up_callback)

        self.is_short_downclick = False
        self.is_long_downclick = False        
        self.downclick_timer = self.reactor.register_timer(self.long_downclick_event)        
        self.register_zbutton(config, 'down_pin', self.down_callback)


    def key_event_handle(self, key, eventtime):

        if not self.inside_handlegcode :
            self.inside_handlegcode = True        
            template = self.abnormal_template
            if key == 'uppress':
                template = self.uppress_template
            elif key == 'uprelease':
                template = self.uprelease_template
            elif key == 'uplngp':
                template = self.uplngp_template
            elif key == 'uplngr':
                template = self.uplngr_template
            elif key == 'downpress':
                template = self.downpress_template
            elif key == 'downrelease':
                template = self.downrelease_template
            elif key == 'downlngp':
                template = self.downlngp_template 
            elif key == 'downlngr':
                template = self.downlngr_template
            try:
                self.gcode.run_script(template.render())
            except:
                logging.exception("Script running error key event handle")
            self.inside_handlegcode = False            
        else:
            logging.exception("Script running repeat, check zctrl")    

    def long_upclick_event(self, eventtime):
        self.is_short_upclick = False
        self.is_long_upclick = True
        self.key_event_handle('uplngp', eventtime)
        return self.reactor.NEVER

    def long_downclick_event(self, eventtime):
        self.is_short_downclick = False
        self.is_long_downclick = True
        self.key_event_handle('downlngp', eventtime)
        return self.reactor.NEVER        


    def up_callback(self, eventtime, state):
        self.up_state = state
        if state:
            self.is_short_upclick = True
            self.is_long_upclick = False
            self.reactor.update_timer(self.upclick_timer,
                                      eventtime + LONG_PRESS_DURATION)
        elif self.is_short_upclick:
            self.reactor.update_timer(self.upclick_timer, self.reactor.NEVER)
            self.key_event_handle('uppress', eventtime)
        #else:             
        elif self.is_long_upclick:    
             self.key_event_handle('uplngr', eventtime)

        
    def down_callback(self, eventtime, state):
        self.down_state = state
        if state:
            self.is_short_downclick = True
            self.is_long_downclick = False
            self.reactor.update_timer(self.downclick_timer,
                                      eventtime + LONG_PRESS_DURATION)
        elif self.is_short_downclick:
            self.reactor.update_timer(self.downclick_timer, self.reactor.NEVER)
            self.key_event_handle('downpress', eventtime)
        #else:    
        elif self.is_long_downclick:    
             self.key_event_handle('downlngr', eventtime)


    def register_zbutton(self, config, name, callback, push_only=False):
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


    cmd_QUERY_ZCTRL_help = "Report on the state of a zctrl"
    def cmd_QUERY_ZCTRL(self, gcmd):
        resst = self.get_status()
        gcmd.respond_info(self.name + ": U" + resst['stateu'] + ",D" + resst['stated'])

    def get_status(self, eventtime=None):
        if self.up_state and self.down_state:
            return {
                     'stateu': "PRESSED",
                     'stated': "PRESSED"
                    }
        elif self.up_state or self.down_state:  
            if self.up_state:
                return {
                     'stateu': "PRESSED",
                     'stated': "RELEASED"
                    }
            else:
                return {
                     'stateu': "RELEASED",
                     'stated': "PRESSED"
                    }

        return {
                  'stateu': "RELEASED",
                  'stated': "RELEASED"
               }
        

def load_config_prefix(config):
    return ZctrlPanel(config)
