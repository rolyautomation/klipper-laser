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
MAX_Z_SOFT_VAL = 60
TM_INTER_VAL = 0.5

M_QLEN_MIN = 3


CCW_DIR_VAL  = 0
CW_DIR_VAL   = 1

DCM_CMD_START = 1
DCM_CMD_STOP  = 2



class ZctrlPanel:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()  
        self.mutex = self.reactor.mutex()      
        self.name = config.get_name().split(' ')[-1]
        self.up_state = 0
        self.down_state = 0
        self.upkey_usef = False
        self.downkey_usef = False   
        self.upkey_longmode = 0
        self.downkey_longmode = 0          
        #self.z_soft_high_val = MAX_Z_SOFT_VAL  
        self.inside_handlegcode = False
        #self.isidle_mode = False
        self.isidle_mode = True
        self.dcm_keymode = False
        self.dcm_longpress = False
        self.startdirpflag = False

        self.dcm_existf = config.has_section('angledcmove as5600m')
        #fail value
        #self.dcm_existf = config.has_section('angledcmove')
        self.limitswitch_existf = config.has_section('limitswitch_check lswcheck')

        self.machinepos = None
        self.z_soft_high_val = config.getfloat(
            'max_z_safe_d', MAX_Z_SOFT_VAL, above=20, maxval=120)
        self.pshort_e_dist = config.getfloat(
            'short_d_unit', 1, above=0., maxval=10)
        self.plong_e_dist = config.getfloat(
             'long_d_unit', 2, above=0., maxval=10)   

        self.pshort_speed = config.getfloat(
            'pshort_speed', 5, above=0., maxval=10)
        self.plong_speed = config.getfloat(
             'plong_speed', 10, above=0., maxval=30)  

        self.min_gap_d = config.getfloat(
            'min_gap_d', GAP_NEED_VAL, minval=-2., maxval=5)  

        self.qminlen  = config.getint('qminlen', M_QLEN_MIN, minval=0)   


        self.dcm_long_stepf = 2.8/4
        #self.dcm_short_stepf = 0.01
        #self.dcm_short_stepf = 0.05
        self.dcm_short_stepf = config.getfloat(
            'dcm_short_step', 0.05, above=0., maxval=1)          


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

        self.gcode.register_command('LOOKZCTRLPARAM', self.cmd_LOOKZCTRLPARAM)                                          


        self.is_short_upclick = False
        self.is_long_upclick = False
        self.upclick_timer = self.reactor.register_timer(self.long_upclick_event)
        self.register_zbutton(config, 'up_pin', self.up_callback)

        self.is_short_downclick = False
        self.is_long_downclick = False        
        self.downclick_timer = self.reactor.register_timer(self.long_downclick_event)        
        self.register_zbutton(config, 'down_pin', self.down_callback)

        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect_check)     


        self.gcode.register_command("SW_AS_KEYM", self.cmd_SW_AS_KEYM)  
        self.gcode.register_command("LOOK_AS_KEYM", self.cmd_LOOK_AS_KEYM)   


    def cmd_LOOKZCTRLPARAM(self, gcmd):
        msgstr = "max_z_d:%s sedist:%s " % (self.z_soft_high_val, self.pshort_e_dist)
        msgstr += "ledist:%s sspeed:%s lspeed:%s" % (self.plong_e_dist, self.pshort_speed, self.plong_speed)        
        gcmd.respond_info(msgstr)    

    def cmd_SW_AS_KEYM(self, gcmd):
        sw = gcmd.get_int('S',1, minval=0, maxval=2)
        self.dcm_keymode = False
        st_str = 'close'
        if sw > 0:
            self.dcm_keymode = True
            st_str = 'open'
        msg = "%s=%d" % (st_str,sw)
        gcmd.respond_info(msg) 

    def cmd_LOOK_AS_KEYM(self, gcmd):
        sw = 0
        st_str = 'close'
        if self.dcm_keymode :
            sw = 1
            st_str = 'open'
        msg = "%s=%d" % (st_str,sw)
        gcmd.respond_info(msg) 


    def _handle_connect_check(self):
        z_max_val = self.z_soft_high_val
        if self.machinepos is None:
            kin = self.printer.lookup_object('toolhead').get_kinematics()
            self.machinepos = kin.get_machine_pos()
            z_max_val = self.machinepos[2][1]
        self.z_soft_high_val = min(z_max_val, self.z_soft_high_val)
        logging.info("zsoft_high_val=%s\n",self.z_soft_high_val)
        #if self.dcm_existf :
            #logging.info("\n angledcmove \n")
        #else:
            #logging.info("\n not angledcmover \n")        
        pass


    def check_isprinting(self,eventtime):
        print_stats= self.printer.lookup_object('print_stats')
        #eventtime = self.reactor.monotonic()
        state = print_stats.get_status(eventtime)['state']
        self.isidle_mode = True
        if state ==  "printing":
            self.isidle_mode = False
        elif state ==  "paused":
            self.isidle_mode = False
        #piro high    
        if self.dcm_keymode :    
            self.isidle_mode = False

    def key_event_handle(self, key, eventtime):
        #logging.info("not oper:key_event=%s,%s",key,eventtime)      
        #return
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
                #self.gcode.run_script_from_command(template.render())
            except:
                logging.exception("Script running error key event handle")
            self.inside_handlegcode = False            
        else:
            logging.exception("Script running repeat, check zctrl")

    def checkdcm_allow_run_longpress(self, rdir, rdistf=0, rspeed=1):
        allow_run_next = 0
        if self.dcm_existf and self.dcm_keymode and not self.dcm_longpress:
            angledcmoveas = self.printer.lookup_object('angledcmove as5600m') 
            angledcmoveas.dcm_move_cmd(DCM_CMD_START,rdir)
            self.dcm_longpress =  True
        else:
            logging.info("not operation:checkdcm_allow_run_longpress\n")            
        return allow_run_next   

    def checkdcm_stop_longpress(self, rdir=0, rdistf=0, rspeed=1):
        allow_run_next = 0
        if self.dcm_longpress:
            angledcmoveas = self.printer.lookup_object('angledcmove as5600m') 
            angledcmoveas.dcm_move_cmd(DCM_CMD_STOP,rdir)
            self.dcm_longpress = False
            allow_run_next = 1
        return allow_run_next 


    def checkdcm_allow_run(self, rdir, rdistf, rspeed=1):
        allow_run_next = 0
        if self.dcm_existf and self.dcm_keymode:
            angledcmoveas = self.printer.lookup_object('angledcmove as5600m') 
            angledcmoveas.dcm_move_correct(rdistf,rdir)
        else:
            logging.info("not operation:checkdcm_allow_run\n")              
        return allow_run_next            


    def checkz_allow_run_drip(self, rdir, rdist, rspeed):
        allow_val = 0
        if not self.startdirpflag:
            self.startdirpflag = True
            instrstr = "M400\n M286  Z   E1  F%s" % (rspeed,)
            if rdir == DOWN_DIR_VAL:
                instrstr = "M400\n M286  Z   E0  F%s" % (rspeed,)
            try:   
                logging.info("start run drip=%s",instrstr)             
                self.gcode.run_script(instrstr)
                logging.info("end run drip=%s",instrstr)   
            except Exception as e:
                self.gcode.respond_info("checkz drip info: %s" % str(e))              
            self.startdirpflag = False 
        return  allow_val            



    def checkz_allow_run(self, rdir, rdist, rspeed):
        toolhead = self.printer.lookup_object('toolhead')
        z_min_st = 0
        z_max_st = 0
        logging.info("checkz_allow_run=%s,%s,%s",rdir,rdist,rspeed)
        qlen = toolhead.check_lookqueue_addnum()
        #if qlen >  M_QLEN_MIN:
        if qlen >  self.qminlen:        
            #logging.info("\nmore than=%d\n",self.qminlen)
            return 1
        #error limitswitch_check = self.printer.lookup_object('limitswitch_check')
        if self.limitswitch_existf :
            #logging.info("\n limitswitch_check \n")
            limitswitch_check = self.printer.lookup_object('limitswitch_check lswcheck')
            if limitswitch_check is not None:
                z_min_st, z_max_st = limitswitch_check.get_zlsw()

        logging.info("limit move=%s,%s",z_min_st, z_max_st)
        pos = toolhead.get_position()  
        zpos = pos[2]
        rzpos = pos[2]
        
        if rdir == UP_DIR_VAL:            
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
            #if zpos <= GAP_NEED_VAL: #important    
            if zpos <= self.min_gap_d:           
                allow_val = 0  
                rzpos = zpos
            else:                
                if  exp_zpos < self.min_gap_d:
                    allow_val = self.min_gap_d  - exp_zpos
                    rzpos = self.min_gap_d   

            if  z_min_st:
                allow_val = 0  

        logging.info("check move=%s,%s,%s",allow_val,rzpos,rspeed)
        if  allow_val : 
            toolhead.manual_move_zaxis(rzpos, rspeed)
        return allow_val


    def stop_now_run(self):
        flag = 0
        if self.startdirpflag:
            logging.info("stop_now_run drip")
            self.printer.send_event("jogging:trigger_to_stop", 1) 
            logging.info("end send drip")
            #self.gcode.run_script(stopinstr_str)
        flag = self.checkdcm_stop_longpress()
        if flag :
            toolhead = self.printer.lookup_object('toolhead')
            toolhead.dwell(0.001) 
            #toolhead.flush_step_generation()  


    def long_upclick_event(self, eventtime):
        allow_continue = 0
        self.is_short_upclick = False
        self.is_long_upclick = True  
        self.check_isprinting(eventtime)      
        self.check_isprinting(eventtime)
        if self.isidle_mode:
            allow_continue = self.checkz_allow_run_drip(UP_DIR_VAL, self.plong_e_dist, self.plong_speed)
        else:
            allow_continue = self.checkdcm_allow_run_longpress(CW_DIR_VAL, self.dcm_long_stepf)                     
        self.key_event_handle('uplngp', eventtime)        
        return self.reactor.NEVER


    def long_downclick_event(self, eventtime):
        allow_continue = 0 
        self.is_short_downclick = False
        self.is_long_downclick = True        
        self.check_isprinting(eventtime)
        if self.isidle_mode:           
            allow_continue = self.checkz_allow_run_drip(DOWN_DIR_VAL, self.plong_e_dist, self.plong_speed)
        else:
            allow_continue = self.checkdcm_allow_run_longpress(CCW_DIR_VAL, self.dcm_long_stepf)                      
        self.key_event_handle('downlngp', eventtime)        
        return self.reactor.NEVER    



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
                else:
                    allow_continue = self.checkdcm_allow_run(CW_DIR_VAL, self.dcm_short_stepf)                     
                self.key_event_handle('uppress', eventtime)
        #else:             
        elif self.is_long_upclick:
            if self.upkey_usef:
                self.upkey_usef = False  
                self.stop_now_run()   
                #logging.info("\n ack uplngr \n")  
                self.key_event_handle('uplngr', eventtime)
            else:    
                #logging.info("\n uplngr \n")
                pass

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
                else:
                    allow_continue = self.checkdcm_allow_run(CCW_DIR_VAL, self.dcm_short_stepf)                     
                self.key_event_handle('downpress', eventtime)
                self.downkey_usef = False

        #else:    
        elif self.is_long_downclick: 
            if self.downkey_usef:
                self.stop_now_run()            
                self.key_event_handle('downlngr', eventtime)
                #logging.info("\n ack downlngr \n")
                self.downkey_usef = False
            else:      
                #logging.info("\n downlngr \n")
                pass

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
