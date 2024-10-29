# Support z axis move by key  
#
# Copyright (C) 2024-2028  jinqiang <jinqiang@ecomedge.io>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

LONG_PRESS_DURATION = 0.400
#LONG_PRESS_DURATION = 1.800
UP_DIR_VAL  = 0
DOWN_DIR_VAL  = 1
#1mm
GAP_NEED_VAL  = 1
MAX_Z_SOFT_VAL = 100
TM_INTER_VAL = 0.5

class ZctrlPanel:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()        
        self.name = config.get_name().split(' ')[-1]
        self.up_state = 0
        self.down_state = 0
        self.upkey_usef = False
        self.downkey_usef = False   
        self.upkey_longmode = 0
        self.downkey_longmode = 0          
        #self.z_soft_high_val = MAX_Z_SOFT_VAL  
        self.inside_handlegcode = False
        self.isidle_mode = False
        self.isidle_mode = True

        self.z_soft_high_val = config.getfloat(
            'max_z_safe_d', MAX_Z_SOFT_VAL, above=40, maxval=120)
        self.pshort_e_dist = config.getfloat(
            'short_d_unit', 1, above=0., maxval=10)
        self.plong_e_dist = config.getfloat(
             'long_d_unit', 2, above=0., maxval=10)   

        self.pshort_speed = config.getfloat(
            'pshort_speed', 5, above=0., maxval=10)
        self.plong_speed = config.getfloat(
             'plong_speed', 10, above=0., maxval=30)  

        self.ptime_interval = config.getfloat(
            'time_interval', 0.1, above=0., maxval=1)                          


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



    def check_isprinting(self,eventtime):
        print_stats= self.printer.lookup_object('print_stats')
        #eventtime = self.reactor.monotonic()
        state = print_stats.get_status(eventtime)['state']
        self.isidle_mode = True
        if state ==  "printing":
            self.isidle_mode = False
        elif state ==  "paused":
            self.isidle_mode = False


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


    def checkz_allow_run(self, rdir, rdist, rspeed):
        toolhead = self.printer.lookup_object('toolhead')
        z_min_st = 0
        z_max_st = 0
        #limitswitch_check = self.printer.lookup_object('limitswitch_check')
        limitswitch_check = self.printer.lookup_object('limitswitch_check lswcheck')
        
        if limitswitch_check is not None:
            z_min_st, z_max_st = limitswitch_check.get_zlsw()
        pos = toolhead.get_position()  
        zpos = pos[2]
        rzpos = pos[2]
        
        if rdir == DOWN_DIR_VAL:
            allow_val = rdist
            exp_zpos = zpos + rdist
            rzpos = exp_zpos
            if  exp_zpos > self.z_soft_high_val: 
                allow_val = self.z_soft_high_val - zpos
                rzpos = self.z_soft_high_val
            if  z_max_st:
                allow_val = 0 

        else:
            rdist = -rdist
            allow_val = rdist
            exp_zpos = zpos + rdist
            rzpos = exp_zpos
            #if zpos < GAP_NEED_VAL:
            if zpos <= GAP_NEED_VAL: #important               
                allow_val = 0  
                rzpos = zpos
            else:                
                if  exp_zpos < GAP_NEED_VAL:
                    allow_val = GAP_NEED_VAL - exp_zpos
                    rzpos = GAP_NEED_VAL  

            if  z_min_st:
                allow_val = 0  

        if  allow_val : 
            toolhead.manual_move_zaxis(rzpos, rspeed)
        return allow_val

    def stop_now_run(self):
        flag = 0
        if self.upkey_longmode  > 0 :
            self.reactor.update_timer(self.upclick_timer, self.reactor.NEVER) 
            self.upkey_longmode = 0  
            flag = 1    
        if self.downkey_longmode  > 0 :
            self.reactor.update_timer(self.downclick_timer, self.reactor.NEVER)
            self.downkey_longmode = 0
            falg = 1
        if flag :
            toolhead = self.printer.lookup_object('toolhead')
            toolhead.dwell(0.001) 
            #toolhead.flush_step_generation()

    def long_upclick_event(self, eventtime):
        allow_continue = 0
        if (self.upkey_longmode > 0):
            self.upkey_longmode = 2 
            self.check_isprinting(eventtime)
            if self.isidle_mode:
                allow_continue = self.checkz_allow_run(UP_DIR_VAL, self.plong_e_dist, self.plong_speed)
            self.key_event_handle('uplngp', eventtime)
        else:
            self.upkey_longmode = 1        
            self.is_short_upclick = False
            self.is_long_upclick = True
            self.check_isprinting(eventtime)
            if self.isidle_mode:            
                allow_continue = self.checkz_allow_run(UP_DIR_VAL, self.plong_e_dist, self.plong_speed)
            self.key_event_handle('uplngp', eventtime)
        #return eventtime+TM_INTER_VAL  
        if (allow_continue != 0):
            return eventtime + self.ptime_interval
        else:    
            return self.reactor.NEVER

    def long_downclick_event(self, eventtime):
        allow_continue = 0        
        if self.downkey_longmode  > 0 :
           self.downkey_longmode = 2
           self.check_isprinting(eventtime)
           if self.isidle_mode:           
                allow_continue = self.checkz_allow_run(DOWN_DIR_VAL, self.plong_e_dist, self.plong_speed)
           self.key_event_handle('downlngp', eventtime)
        else:
            self.downkey_longmode = 1                 
            self.is_short_downclick = False
            self.is_long_downclick = True
            self.check_isprinting(eventtime)
            if self.isidle_mode:             
                allow_continue = self.checkz_allow_run(DOWN_DIR_VAL, self.plong_e_dist, self.plong_speed)
            self.key_event_handle('downlngp', eventtime)
        #return eventtime+TM_INTER_VAL 
        if (allow_continue != 0):
            return eventtime + self.ptime_interval
        else:    
            return self.reactor.NEVER 
        #return self.reactor.NEVER        

    def up_callback(self, eventtime, state):
        self.up_state = state
        if state:
            if self.downkey_usef:
                self.upkey_usef = False
            else:
                self.upkey_usef = True
            
            if self.upkey_usef:
                self.is_short_upclick = True
                self.is_long_upclick = False
                self.upkey_longmode = 0 
                self.reactor.update_timer(self.upclick_timer,
                                        eventtime + LONG_PRESS_DURATION)
        elif self.is_short_upclick:
            if self.upkey_usef:            
                self.upkey_usef = False
                self.reactor.update_timer(self.upclick_timer, self.reactor.NEVER)
                self.check_isprinting(eventtime)
                if self.isidle_mode:                 
                    self.checkz_allow_run(UP_DIR_VAL, self.pshort_e_dist, self.pshort_speed)
                self.key_event_handle('uppress', eventtime)
        #else:             
        elif self.is_long_upclick:
            if self.upkey_usef:
                self.upkey_usef = False  
                self.stop_now_run()     
                self.key_event_handle('uplngr', eventtime)


    def down_callback(self, eventtime, state):
        self.down_state = state
        if state:
            if self.upkey_usef:
                self.downkey_usef = False
            else:
                self.downkey_usef = True
            if self.downkey_usef:
                self.is_short_downclick = True
                self.is_long_downclick = False
                self.reactor.update_timer(self.downclick_timer,
                                        eventtime + LONG_PRESS_DURATION)
        elif self.is_short_downclick:
            if self.downkey_usef:
                self.reactor.update_timer(self.downclick_timer, self.reactor.NEVER)
                self.check_isprinting(eventtime)
                if self.isidle_mode:                  
                    self.checkz_allow_run(DOWN_DIR_VAL, self.pshort_e_dist, self.pshort_speed)
                self.key_event_handle('downpress', eventtime)
                self.downkey_usef = False

        #else:    
        elif self.is_long_downclick: 
            if self.downkey_usef:
                self.stop_now_run()            
                self.key_event_handle('downlngr', eventtime)
                self.downkey_usef = False  
           

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
