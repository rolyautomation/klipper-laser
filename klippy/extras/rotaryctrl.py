# Rotary controller interface
#
# Copyright (C) 2025-2028  jinqiang <jinqiang@ecomedge.io>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, ast

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
        #self.name = config.get_name().split(' ')[-1]
        self.name = 'rotary'
        self.last_rotarystate = [0, 0, 0, 0]
        self.pre_rotarystate = [0, 0, 0, 0]
        self.toolhead = None
        self.astepdict = {}        
        self.asteplist = []
        self.ardistllist = []
        self.arsprllist = []
        self.anum = 0
        self.epsilon = 0.0001

        #self.multimotor_existf = config.has_section('multimotor_axis')
        #if self.multimotor_existf :
        self.multimotor_axis_obj = None
        self.cur_selindex = -1
        self.rotary_exist = 0

        self.rmcu_name = config.get('rmcu_name', "rollerset")
        self.reconnect_event_name = f"danger:non_critical_mcu_{self.rmcu_name}:reconnected"
        self.disconnect_event_name = f"danger:non_critical_mcu_{self.rmcu_name}:disconnected"
        

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
        #self.gcode.register_command("TEST_ROTARY", self.cmd_TEST_ROTARY)


        self.gcode.register_command("SELECT_ROTARY", self.cmd_SELECT_ROTARY, desc=self.cmd_SELECT_ROTARY_help)
        self.gcode.register_command("SET_ROTARY_ENABLE", self.cmd_SET_ROTARY_ENABLE, desc=self.cmd_SET_ROTARY_ENABLE_help) 
        self.gcode.register_command("M520", self.cmd_M520_ACIRCUM, desc=self.cmd_M520_ACIRCUM_help)



        self.register_switchbutton(config, 'R0_pin', self.R0_callback)
        self.register_switchbutton(config, 'R1_pin', self.R1_callback)
        self.register_switchbutton(config, 'R2_pin', self.R2_callback)
        self.register_switchbutton(config, 'R3_pin', self.R3_callback)
        self.rotray_pkeystate = [0, 0, 0, 0]
        self.doaction_ins = 0

        self.printer.register_event_handler(
             self.reconnect_event_name,
             self.handle_reconnect
         )

        self.printer.register_event_handler(
            self.disconnect_event_name,
            self.handle_disconnect
        )

        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect_rotaryctrl)     



    def handle_reconnect(self):
        self.rotary_exist = 1
        self.last_rotarystate = [0, 0, 0, 0]
        self.pre_rotarystate = [0, 0, 0, 0]   
        self.rotray_pkeystate = [0, 0, 0, 0]   
        logging.info("rotaryctrl:handle_reconnect")       
        pass
 

    def handle_disconnect(self):
        self.rotary_exist = 0
        self.last_rotarystate = [0, 0, 0, 0]
        self.pre_rotarystate = [0, 0, 0, 0]   
        self.rotray_pkeystate = [0, 0, 0, 0]    
        logging.info("rotaryctrl:handle_disconnect") 
        #self.cmd_SET_MULTIMOTOR_AXIS(-1)            
        pass


    def _handle_connect_rotaryctrl(self):
        self.multimotor_axis_obj = self.printer.lookup_object('multimotor_axis',None)
        self.update_rotary_selectindex()
        if self.toolhead is None:
            self.toolhead = self.printer.lookup_object('toolhead')
        #self.toolhead.flush_step_generation()
        kin = self.toolhead.get_kinematics()
        allsteppers = kin.get_steppers()
        aaxis_str = "_a"
        for s in allsteppers :
            stepper_name = s.get_name()
            if aaxis_str in stepper_name:
                #self.astepdict[stepper_name] = s
                self.asteplist.append(s)
                rotation_dist, steps_per_rotation = s.get_rotation_distance()
                self.ardistllist.append(rotation_dist)
                #self.arsprllist.append(steps_per_rotation)
        self.anum = len(self.ardistllist)
        logging.info("%s:%s:%s", self.rmcu_name, self.reconnect_event_name, self.disconnect_event_name)
        #pass 


    def update_rotary_selectindex(self):
        if self.multimotor_axis_obj is not None:
            self.cur_selindex = self.multimotor_axis_obj.get_curselindex()
        pass  


    cmd_SELECT_ROTARY_help = "select rotary controller by enable status"
    def cmd_SELECT_ROTARY(self, gcmd):
        if self.rotary_exist == 0:
            gcmd.respond_info("rotary not exist")
            return
        index = gcmd.get_int('NUM', minval=-1, maxval=10)
        selectindex = 10
        if index >= 0 and index < 4:
            selectindex = index
            if self.last_rotarystate[index] == 0:
                selectindex = 9
        elif index < 0:
              selectindex = -1       
        msgh = "rotary select index:%d" % (selectindex,)
        if  selectindex == 10:
            msgh = "input index out of range" 
        elif selectindex == 9:  
            msgh = "please enable the number=%d rotary  first" % (index,) 
        else:
            self.gcode.run_script_from_command(
                "SET_MULTIMOTOR_AXIS NUM=%d \n" 
                "M118  SELECT_ROTARY %d SUCCESS \n" 
                % (selectindex, selectindex)
                )            
        gcmd.respond_info(msgh)  


    def cmd_SET_MULTIMOTOR_AXIS(self, selectindex=-1): 
        self.gcode.run_script_from_command(
            "SET_MULTIMOTOR_AXIS NUM=%d \n" 
            "M118  SELECT_ROTARY %d SUCCESS \n" 
            % (selectindex, selectindex)
            )                
        pass



    cmd_SET_ROTARY_ENABLE_help = "set rotary enable status"
    def cmd_SET_ROTARY_ENABLE(self, gcmd):
        if self.rotary_exist == 0:
            gcmd.respond_info("rotary not exist")
            return
        msgh = "setrotary:"
        gval_string = gcmd.get('ARRAY',None)
        if gval_string == None:
            gcmd.respond_info(msgh + "no input value")  
            return  
        data_list = ast.literal_eval(gval_string)
        logging.info("inputvalue: " + str(data_list))             
        if 4 == len(data_list):
            self.pre_rotarystate = self.last_rotarystate.copy()
            self.last_rotarystate[0] = data_list[0]
            self.last_rotarystate[1] = data_list[1]
            self.last_rotarystate[2] = data_list[2]
            self.last_rotarystate[3] = data_list[3]
            self.update_action_by_status()
            gcmd.respond_info(msgh + "success")  
        else:
            gcmd.respond_info(msgh + " input value num error, expect " + str(4))


    def cmd_TEST_ROTARY(self, gcmd):
        if self.rotary_exist == 0:
            gcmd.respond_info("rotary not exist")
            return
        inputnum_val  =  gcmd.get_int('N', 0, minval=0, maxval=3)
        set_val  =  gcmd.get_int('V', 0, minval=0, maxval=1)   
        self.last_rotarystate[inputnum_val] = set_val   
        gcmd.respond_info(self.name + ":[" + str(self.last_rotarystate[0]) + str(self.last_rotarystate[1]) + str(self.last_rotarystate[2]) + str(self.last_rotarystate[3]) + "]")  


    cmd_M520_ACIRCUM_help = "set rotary a axis circum"
    def cmd_M520_ACIRCUM(self, gcmd):
        if self.rotary_exist == 0:
            gcmd.respond_info("rotary not exist")
            return        
        #if self.toolhead is None:
            #self.toolhead = self.printer.lookup_object('toolhead')
        #self.toolhead.flush_step_generation()
        #logging.info("step_list" + str(self.astepdict))   
        #logging.info("ardist_list" + str(self.ardistllist))  
        #logging.info("anum:" + str(self.anum)) 
        #logging.info("arspr_list" + str(self.arsprllist))   

        acircumval = gcmd.get_float('S', 0., minval=0.)
        msgh = "M520 S" + str(acircumval) + " "
        if self.cur_selindex >= 0 and self.cur_selindex < self.anum:
            s = self.asteplist[self.cur_selindex]
            rotation_dist, rd_notuse = s.get_rotation_distance()
            if acircumval < self.epsilon:
                acircumval = self.ardistllist[self.cur_selindex]
            if abs(acircumval-rotation_dist) > self.epsilon:
                self.toolhead.flush_step_generation()
                s.set_rotation_distance(acircumval)
                msgh = "M520 S" + str(acircumval) + " done" 
            else:
                msgh = "current value is equal to setting value[%s == %s]" % (acircumval, rotation_dist)          
        elif self.cur_selindex < 0:
            msgh = "please select rotary by SELECT_ROTARY first"
        else:
            msgh = "input index out of range[%d > %d]" % (self.cur_selindex, self.anum)
        gcmd.respond_info(msgh)  


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
                if self.rotary_exist > 0:
                    self.gcode.run_script(template.render())
                #self.gcode.run_script_from_command(template.render())
            except:
                logging.exception("Script running error key event handle in rotary ctrl")
            self.inside_handlegcode = False            
        else:
            logging.exception("Script running repeat, check press speed")



    def key_event_handle_softsync(self, actionid):
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
                #self.gcode.run_script(template.render())
                if self.rotary_exist > 0:
                    self.gcode.run_script_from_command(template.render())
            except:
                logging.exception("Script running error key event handle in rotary ctrl softsync")
            self.inside_handlegcode = False            
        else:
            logging.exception("Script running repeat, check press speed softsync")



    def update_action_by_status(self):
        if self.pre_rotarystate[0] != self.last_rotarystate[0]:
            actionid = R0_DOACTION_NO                
            if self.last_rotarystate[0]:
                actionid = R0_DOACTION_YES
            self.key_event_handle_softsync(actionid)  
        if self.pre_rotarystate[1] != self.last_rotarystate[1]:
            actionid = R1_DOACTION_NO                
            if self.last_rotarystate[1]:
                actionid = R1_DOACTION_YES
            self.key_event_handle_softsync(actionid)     
        if self.pre_rotarystate[2] != self.last_rotarystate[2]:
            actionid = R2_DOACTION_NO                
            if self.last_rotarystate[2]:
                actionid = R2_DOACTION_YES
            self.key_event_handle_softsync(actionid)     
        if self.pre_rotarystate[3] != self.last_rotarystate[3]:
            actionid = R3_DOACTION_NO                
            if self.last_rotarystate[3]:
                actionid = R3_DOACTION_YES
            self.key_event_handle_softsync(actionid)                             
        pass 


    def R0_callback(self, eventtime, state):
        self.rotray_pkeystate[R0_IND] = state
        if state:
            #self.last_rotarystate[R0_IND] = not self.last_rotarystate[R0_IND]
            self.last_rotarystate[R0_IND] = (self.last_rotarystate[R0_IND] + 1) % 2
            actionid = R0_DOACTION_NO                
            if self.last_rotarystate[R0_IND]:
                actionid = R0_DOACTION_YES
            self.doaction_ins = actionid
            self.key_event_handle(actionid, eventtime)

       
    def R1_callback(self, eventtime, state):
        self.rotray_pkeystate[R1_IND] = state
        if state:
            self.last_rotarystate[R1_IND] = (self.last_rotarystate[R1_IND] + 1) % 2
            actionid = R1_DOACTION_NO                
            if self.last_rotarystate[R1_IND]:
                actionid = R1_DOACTION_YES
            self.doaction_ins = actionid
            self.key_event_handle(actionid, eventtime)

    def R2_callback(self, eventtime, state):
        self.rotray_pkeystate[R2_IND] = state
        if state:
            self.last_rotarystate[R2_IND] = (self.last_rotarystate[R2_IND] + 1) % 2
            actionid = R2_DOACTION_NO                
            if self.last_rotarystate[R2_IND]:
                actionid = R2_DOACTION_YES
            self.doaction_ins = actionid
            self.key_event_handle(actionid, eventtime)

        
    def R3_callback(self, eventtime, state):
        self.rotray_pkeystate[R3_IND] = state
        if state:
            self.last_rotarystate[R3_IND] = (self.last_rotarystate[R3_IND] + 1) % 2
            actionid = R3_DOACTION_NO                
            if self.last_rotarystate[R3_IND]:
                actionid = R3_DOACTION_YES
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
        rotary_status['rotary_enabled'] = [ self.last_rotarystate[0], self.last_rotarystate[1], self.last_rotarystate[2], self.last_rotarystate[3] ]
        rotary_status['rotary_selected'] = self.cur_selindex
        rotary_status['rotary_exist'] = self.rotary_exist
        return dict(rotary_status)


#def load_config_prefix(config):
def load_config(config):
    return RotaryCtrlInterface(config)
