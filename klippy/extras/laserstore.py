# laser store interface
# file: laserstore.py
# Copyright (C) 2025-2028  jinqiang <jinqiang@ecomedge.io>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, logging, ast
import json

ITEM_LEN = 16

class LaserStoreInterface:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.filename = None
        fn = config.get('filename', None)
        if fn is not None:
            self.filename = os.path.expanduser(fn)        

        self.default_laser_parameters = {
            'galvo_params_diode': {
                'center_offset': [0.0, 0.0],
                'working_distance': 0.0,
                'rotation_angle': -0.0,
                'pitch_factor': 0.0,               
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
            },
            'galvo_params_fiber': {
                'center_offset': [0.0, 0.0],
                'working_distance': 0.0,
                'rotation_angle': 0.0,
                'pitch_factor': 0.0,
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
            }
        }

        self.keys_order = [
            'center_offset', 'working_distance', 'rotation_angle', 'pitch_factor', 
            'b_scale_plus', 'b_scale_minus', 'c_scale_plus', 'c_scale_minus', 
            'b2', 'b4', 'b6', 'b8', 
            'c2', 'c4', 'c6', 'c8'
        ]

        self.update_file = False
        self.load_param_from_file()

        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command("QUERY_LASER_STORE", self.cmd_QUERY_LASER_STORE, desc=self.cmd_QUERY_LASER_STORE_HELP)
        self.gcode.register_command('SET_LASER_STORE', self.cmd_SET_LASER_STORE, desc=self.cmd_SET_LASER_STORE_HELP)

    def load_param_from_file(self):  
        if self.filename is None:
            self.laser_parameters = self.default_laser_parameters 
            return
        
        if os.path.exists(self.filename):
            with open(self.filename, 'r') as json_file:
                try:
                    loaded_parameters = json.load(json_file)
                    self.laser_parameters = loaded_parameters 
                except json.JSONDecodeError:
                    logging.error("Failed to decode JSON from file. The file may be empty or corrupted.")
                    self.laser_parameters = self.default_laser_parameters
                    self.update_file = True
                    self.save_param_to_file()
        else:
            self.laser_parameters = self.default_laser_parameters 
            self.update_file = True
            self.save_param_to_file()

    def save_param_to_file(self):
        if self.filename is None:
            self.update_file = False
            return        
        if self.update_file:
            with open(self.filename, 'w') as json_file:
                json.dump(self.laser_parameters, json_file, indent=4)
            self.update_file = False

    cmd_QUERY_LASER_STORE_HELP = "Display laser store parameters"
    def cmd_QUERY_LASER_STORE(self, gcmd):
        keyname = gcmd.get('K', None)
        if keyname == None:
            keyname = "diode"
            
        if keyname in ["diode", "fiber"]:
            msgh = "galvo_params_" + keyname.lower() + ":"
            galvo_params = self.laser_parameters['galvo_params_' + keyname.lower()]
            gcmd.respond_info(msgh + str(galvo_params))
        else:
            msgstr = "[diode, fiber]"
            gcmd.respond_info(keyname + " is not valid, expect: " + msgstr)    
            return

    cmd_SET_LASER_STORE_HELP = "Set laser store parameters"
    def cmd_SET_LASER_STORE(self, gcmd):
        keyname = gcmd.get('K', None)
        if keyname == None:
            keyname = "diode"
        keyname = keyname.lower()
        msgh = "galvo_params_" + keyname + ":"
        
        gval_string = gcmd.get('V',None)
        if gval_string == None:
            gcmd.respond_info(msgh + "no input value")
            return
        if keyname in ["diode", "fiber"]:
            data_list = ast.literal_eval(gval_string)
            logging.info("inputvalue: " + str(data_list))
            
            if len(self.keys_order) == len(data_list):
                self.laser_parameters['galvo_params_' + keyname] = dict(zip(self.keys_order, data_list))    
                self.update_file = True
                self.save_param_to_file()
                gcmd.respond_info("Updated " + keyname + " galvo parameters")
            else:
                gcmd.respond_info(msgh + " input value num error, expect " + str(len(self.keys_order)))                 
        else:
            msgstr = "[diode, fiber]"
            gcmd.respond_info(keyname + " is not valid, expect: " + msgstr)    
            return

    def get_status(self, eventtime=None):
        return dict(self.laser_parameters)

def load_config(config):
    return LaserStoreInterface(config)

