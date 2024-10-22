# Support for executing gcode when a hardware button is pressed or released.
#
# Copyright (C) 2019 Alec Plumb <alec@etherwalker.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

class DoorAction:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split(' ')[-1]
        self.left_state = 0
        self.right_state = 0
        self.last_state = 0
        self.inside_handlegcode = False
        buttons = self.printer.load_object(config, "buttons")
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.opendoor_template = gcode_macro.load_template(config, 'opendoor_gcode', '')
        self.closedoor_template = gcode_macro.load_template(config,
                                                          'closedoor_gcode', '')
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_mux_command("QUERY_DOOR", "DOOR", self.name,
                                        self.cmd_QUERY_DOOR,
                                        desc=self.cmd_QUERY_DOOR_help)

        self.register_doorbutton(config, 'left_pin', self.left_callback)
        self.register_doorbutton(config, 'right_pin', self.right_callback)

    def left_callback(self, eventtime, state):
        self.left_state = state
        self.door_callback(eventtime)
        
    def right_callback(self, eventtime, state):
        self.right_state = state
        self.door_callback(eventtime)

    def register_doorbutton(self, config, name, callback, push_only=False):
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


    cmd_QUERY_DOOR_help = "Report on the state of a door"
    def cmd_QUERY_DOOR(self, gcmd):
        gcmd.respond_info(self.name + ": " + self.get_status()['state'])

    def door_callback(self, eventtime):
        self.last_state = False
        if self.left_state and self.right_state:
            self.last_state = True      
        #template = self.opendoor_template
        template = self.closedoor_template
        state = self.last_state
        if not self.inside_handlegcode :
            self.inside_handlegcode = True
            if not state:
                #template = self.closedoor_template
                template = self.opendoor_template
            try:
                self.gcode.run_script(template.render())
            except:
                logging.exception("Script running error")
            self.inside_handlegcode = False
        else:
            logging.exception("Script running repeat, switch fast")        

    def get_status(self, eventtime=None):
        if self.last_state:
            return {'state': "CLOSE"}
        return {'state': "OPEN"}

def load_config_prefix(config):
    return DoorAction(config)
