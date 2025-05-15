# Support door action
#
# Copyright (C) 2024-2028  jinqiang <jinqiang@ecomedge.io>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
#True: key is pressed
#False: key is released


class DoorAction:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split(' ')[-1]
        self.reactor = self.printer.get_reactor()
        self.left_state = 0
        self.right_state = 0
        self.last_state = False
        self.pre_laststate = None
        
        self.inside_handlegcode = False
        buttons = self.printer.load_object(config, "buttons")
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.opendoor_template = gcode_macro.load_template(config, 'opendoor_gcode', '')
        self.closedoor_template = gcode_macro.load_template(config,
                                                          'closedoor_gcode', '')
        self.gcode = self.printer.lookup_object('gcode')
        # self.gcode.register_mux_command("QUERY_DOOR", "DOOR", self.name,
        #                                 self.cmd_QUERY_DOOR,
        #                                 desc=self.cmd_QUERY_DOOR_help)

        self.register_doorbutton(config, 'left_pin', self.left_callback)
        chkleftpin = config.get('left_pin', None)
        if chkleftpin is None:
            self.left_state = 1    
        self.register_doorbutton(config, 'right_pin', self.right_callback)
        chkrightpin = config.get('right_pin', None)
        if chkrightpin is None:
            self.right_state = 1          

        self.gcode.register_command("QUERY_DOOR", self.cmd_QUERY_DOOR,
                                    desc=self.cmd_QUERY_DOOR_help)
        self.gcode.register_command("CHK_DOOR", self.cmd_CHK_DOOR)  

        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)        

    def handle_connect(self):
        #eventtm = self.reactor.monotonic()
        #self.door_callback(eventtm)
        cur_state = False
        if self.left_state and self.right_state:
            cur_state = True  
        self.last_state =  cur_state  
        self.pre_laststate = self.last_state      
        pass        

    def cmd_CHK_DOOR(self, gcmd):
        if self.last_state:
            msg = "door is closeed"
            gcmd.respond_info(msg)
            #logging.info("\n door status: %s\n" ,msg)
            pass
        else:
            toolhead = self.printer.lookup_object('toolhead')
            msg = "door is opened,please close door"
            #gcmd.respond_info(msg)
            #logging.info("\n door status: %s\n" ,msg)
            m = "%s" % (msg)
            #return  toolhead.printer.command_error(m)  
            eobj = toolhead.printer.command_error(m)     
            raise eobj          

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
        gcmd.respond_info(self.name + ": " + self.get_status()['state'] + ", left: " + str(self.get_status()['lstate']) + ", right: " + str(self.get_status()['rstate']))

    def door_callback(self, eventtime):
        cur_state = False
        if self.left_state and self.right_state:
            cur_state = True  
        self.last_state =  cur_state
        #logging.info("door callback :%s\n\n",self.last_state)  
        if self.pre_laststate is None or self.pre_laststate != self.last_state:     
            #template = self.opendoor_template
            template = self.closedoor_template
            state = self.last_state
            if not self.inside_handlegcode :
                self.inside_handlegcode = True
                if not state:
                    #template = self.closedoor_template
                    template = self.opendoor_template
                try:
                    #logging.info("door action :%s\n\n",state)  
                    self.gcode.run_script(template.render())
                except:
                    logging.exception("Script running error")
                self.inside_handlegcode = False
            else:
                logging.exception("Script running repeat, switch fast") 
        self.pre_laststate  = self.last_state         

    def get_status(self, eventtime=None):
        return {
            'state': "CLOSE" if self.last_state else "OPEN",
            'isclose': self.last_state,
            'lstate': self.left_state,
            'rstate': self.right_state
        }


def load_config_prefix(config):
    return DoorAction(config)
