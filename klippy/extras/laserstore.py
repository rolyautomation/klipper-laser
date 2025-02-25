# laser store interface
# file: laserstore.py
# Copyright (C) 2025-2028  jinqiang <jinqiang@ecomedge.io>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#import logging
import os, logging, ast
import json


'''
[galvo_parameters fiber1064]
rotation_angle: -0.0
pitch_factor: 0.0
b_scale_plus: -0.0457644
b_scale_minus: -0.0358127
c_scale_plus: -0.0275848
c_scale_minus: -0.0348631
b2: -0.00000998146
b4: -0.00000927960
b6: -0.00000844448
b8: -0.00000770474
c2: 0.0000186495
c4: 0.0000183209
c6: 0.0000183374
c8: 0.0000191004

[galvo_parameters diode450]
rotation_angle: -0.0
pitch_factor: 0.0
b_scale_plus: 0.0283316
b_scale_minus: 0.0403397
c_scale_plus: 0.0251046
c_scale_minus: 0.0155440
b2: -0.0000103735
b4: -0.0000100012
b6: -0.00000984857
b8: -0.00000864826
c2: 0.0000255225
c4: 0.0000239867
c6: 0.0000245435
c8: 0.0000245894

fiber1064_rotation_angle = 0.0
fiber1064_pitch_factor = 0.0
fiber1064_b_scale_plus = -0.0457644
fiber1064_b_scale_minus = -0.0358127
fiber1064_c_scale_plus = -0.0275848
fiber1064_c_scale_minus = -0.0348631
fiber1064_b2 =  -0.00000998146
fiber1064_b4 =  -0.00000927960
fiber1064_b6 = -0.00000844448
fiber1064_b8 = -0.00000770474
fiber1064_c2 = 0.0000186495
fiber1064_c4 = 0.0000183209
fiber1064_c6 = 0.0000183374
fiber1064_c8 = 0.0000191004

diode450_rotation_angle: -0.0
diode450_pitch_factor: 0.0
diode450_b_scale_plus: 0.0283316
diode450_b_scale_minus: 0.0403397
diode450_c_scale_plus: 0.0251046
diode450_c_scale_minus: 0.0155440
diode450_b2: -0.0000103735
diode450_b4: -0.0000100012
diode450_b6: -0.00000984857
diode450_b8: -0.00000864826
diode450_c2: 0.0000255225
diode450_c4: 0.0000239867
diode450_c6: 0.0000245435
diode450_c8: 0.0000245894

#fiber_selected: 1
#diode_high: 0

'''

ITEM_LEN = 15


class LaserStoreInterface:
    def __init__(self, config):
        self.printer = config.get_printer()
        #self.name = config.get_name().split(' ')[-1]
        self.filename = os.path.expanduser(config.get('filename'))

        self.default_laser_parameters = {
            'galvo_params_diode': {
                'rotation_angle': 0.0,
                'pitch_factor': 0.0,
                'working_distance': 0.0,
                'b_scale_plus': -0.0457644,
                'b_scale_minus': -0.0358127,
                'c_scale_plus': -0.0275848,
                'c_scale_minus': -0.0348631,
                'b2': -0.00000998146,
                'b4': -0.00000927960,
                'b6': -0.00000844448,
                'b8': -0.00000770474,
                'c2': 0.0000186495,
                'c4': 0.0000183209,
                'c6': 0.0000183374,
                'c8': 0.0000191004,
            },
            'galvo_params_fiber': {
                'rotation_angle': -0.0,
                'pitch_factor': 0.0,
                'working_distance': 0.0,                
                'b_scale_plus': 0.0283316,
                'b_scale_minus': 0.0403397,
                'c_scale_plus': 0.0251046,
                'c_scale_minus': 0.0155440,
                'b2': -0.0000103735,
                'b4': -0.0000100012,
                'b6': -0.00000984857,
                'b8': -0.00000864826,
                'c2': 0.0000255225,
                'c4': 0.0000239867,
                'c6': 0.0000245435,
                'c8': 0.0000245894,
            }
        }  

        self.keys_order = [ 'rotation_angle', 'pitch_factor', 'working_distance',  'b_scale_plus', 'b_scale_minus', 'c_scale_plus', 'c_scale_minus', 'b2', 'b4', 'b6', 'b8', 'c2', 'c4', 'c6', 'c8']  

        self.update_file = 0
        self.load_param_from_file()

        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command("QUERY_LASER_STORE", self.cmd_QUERY_LASER_STORE, desc=self.cmd_QUERY_LASER_STORE_HELP)  
        self.gcode.register_command('SET_LASER_STORE', self.cmd_SET_LASER_STORE, desc=self.cmd_SET_LASER_STORE_HELP) 
        #self.printer.register_event_handler("klippy:connect",
                                            #self._handle_connect_laserctrl) 

    #def _handle_connect_laserctrl(self):
        #extruderpwm1 = self.printer.lookup_object('extruderpwm1')
        #if extruderpwm1 is not None:
            #self.opticalfiber_existf = 1    
        #pass   

    def load_param_from_file(self):   
        if os.path.exists(self.filename):
            with open(self.filename, 'r') as json_file:
                try:
                    loaded_parameters = json.load(json_file)
                    self.laser_parameters = loaded_parameters 
                    #logging.info("laser_parameters: " + str(self.laser_parameters)) 
                except json.JSONDecodeError:
                    logging.error("Failed to decode JSON from file. The file may be empty or corrupted.")
                    self.laser_parameters = self.default_laser_parameters
                    self.update_file = 1
                    self.save_param_to_file() 

        else:
            self.laser_parameters = self.default_laser_parameters 
            self.update_file = 1 
            self.save_param_to_file() 


    def save_param_to_file(self):
        if self.update_file > 0:
            with open(self.filename, 'w') as json_file:
                json.dump(self.laser_parameters, json_file, indent=4)
            self.update_file = 0  


    cmd_QUERY_LASER_STORE_HELP = "display  laser store  parameters value"
    def cmd_QUERY_LASER_STORE(self, gcmd):
        mindex = gcmd.get_int('S',0, minval=0, maxval=1)
        if mindex == 0:
            galvo_params = self.laser_parameters['galvo_params_diode'] 
            msgh = "galvo_params_diode:"
        else:
            galvo_params = self.laser_parameters['galvo_params_fiber'] 
            msgh = "galvo_params_fiber:"
        gcmd.respond_info(msgh + str(galvo_params))   


    KEY_DIODE_N = "DIODE"
    KEY_FIBER_N = "FIBER"
    cmd_SET_LASER_STORE_HELP = "set laser store parameters value"
    def cmd_SET_LASER_STORE(self, gcmd): 
        #pass
        #name = gcmd.get('KEY').upper()
        keyname = gcmd.get('K',None)
        if keyname == None:
            keyname = self.KEY_DIODE_N
        keyname = keyname.upper()  
        msgh = "galvo_params_" + keyname + ":"  
        gval_string = gcmd.get('V',None)
        if gval_string == None:
            gcmd.respond_info(msgh + "no input value")  
            return
        if keyname  in [self.KEY_DIODE_N, self.KEY_FIBER_N]:
            data_list = ast.literal_eval(gval_string)
            logging.info("inputvalue: " + str(data_list))
            keyname = keyname.lower()
            if len(self.keys_order) == len(data_list):
                #self.laser_parameters['galvo_params_' + keyname] = data_list
                self.laser_parameters['galvo_params_' + keyname] = dict(zip(self.keys_order, data_list))    
                self.update_file = 1
                self.save_param_to_file()
                gcmd.respond_info(keyname + " update done") 

            else:
                gcmd.respond_info(msgh + " input value num error, expect " + str(len(self.keys_order)))                 
        else:
            msgstr = "[" + self.KEY_DIODE_N + ", " + self.KEY_FIBER_N + "]"
            gcmd.respond_info(keyname + " is not valid, expect: " + msgstr)    
            return  

    def get_status(self, eventtime=None):
        return dict(self.laser_parameters)
        

def load_config(config):
    return LaserStoreInterface(config)


#def load_config_prefix(config):
#   return LaserCtrlInterface(config)

