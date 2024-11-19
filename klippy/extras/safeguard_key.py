# Support safeguard  key
#
# Copyright (C) 2024-2028  jinqiang <jinqiang@ecomedge.io>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, logging, ast, configparser

RIGHT_ALLOW = 'allow'

class SafeGuardUkey:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.filename = os.path.expanduser(config.get('filename'))
        self.allVariables = {}

    def check_filepath_status(self): 
        fst = False
        file_dir_path = os.path.dirname(self.filename)
        fst = os.path.exists(file_dir_path)
        return fst

    def check_file_status(self): 
        fst = False
        file_dir_path = self.filename
        fst = os.path.exists(file_dir_path)
        return fst


    def check_ukey_status(self): 
        iretcode = 0
        ukey_st = False
        ukey_st = self.check_file_status()
        if not ukey_st:
            iretcode = 1
            return iretcode 

        allvars = {}
        varfile = configparser.ConfigParser()
        try:
            varfile.read(self.filename)
            if varfile.has_section('Safeguardukey'):
                for name, val in varfile.items('Safeguardukey'):
                    allvars[name] = ast.literal_eval(val)
        except:
            #msg = "Unable to parse existing ukey file"
            #logging.exception(msg)
            #raise self.printer.command_error(msg)
            iretcode = 2
            return iretcode             
        self.allVariables = allvars

        machine_idname = 'machid'
        machine_id = 0
        with open('/etc/machine-id', 'r') as f:
            machine_id = f.read().strip()  
        if  machine_id != allvars[machine_idname]:  
            iretcode = 3
            return iretcode

        if allvars[RIGHT_ALLOW] > 0:
            iretcode = 0
        else:
            iretcode = 4     
        return iretcode

    def cmd_save_ukey(self, varname, value):
        iretcode = 99
        ukey_st = False
        ukey_st = self.check_filepath_status()
        if not ukey_st:
            iretcode = 1
            return iretcode         

        current_date = os.popen('date').read().strip()  
        current_date =  repr(current_date)
        cur_datename = 'date' 
        machine_idname = 'machid'
        machine_id = 0
        with open('/etc/machine-id', 'r') as f:
            machine_id = f.read().strip()   
        machine_id =  repr(machine_id)

        try:
            value = ast.literal_eval(value)
        except ValueError as e:
            msg = "Unable to parse '%s' as a literal" % (value,)
            raise self.printer.command_error(msg)

        try:
            current_date = ast.literal_eval(current_date)
        except ValueError as e:
            msg = "Unable to parse '%s' as a literal date" % (current_date,)
            raise self.printer.command_error(msg)

        try:
            machine_id = ast.literal_eval(machine_id)
        except ValueError as e:
            msg = "Unable to parse '%s' as a literal machine id" % (machine_id,)
            raise self.printer.command_error(msg)                    
        newvars = dict()
        newvars[varname] = value
        newvars[cur_datename] = current_date
        newvars[machine_idname] = machine_id        

        # Write file
        varfile = configparser.ConfigParser()
        varfile.add_section('Safeguardukey')
        for name, val in sorted(newvars.items()):
            varfile.set('Safeguardukey', name, repr(val))
        try:
            f = open(self.filename, "w")
            varfile.write(f)
            f.close()
        except:
            msg = "Unable to save safeguardukey in program"
            logging.exception(msg)
            #raise gcmd.error(msg)
            raise self.printer.command_error(msg)
        iretcode = 0
        return iretcode             

class SafeGuardkeyt:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.safeguardkey = SafeGuardUkey(config)
        self.laststatus = 0
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('CHK_SAFEKEY', self.cmd_CHK_SAFEKEY,
                               desc=self.cmd_CHK_SAFEKEY_help)            

        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('QUERY_SAFEKEY', self.cmd_QUERY_SAFEKEY,
                               desc=self.cmd_QUERY_SAFEKEY_help)   

        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('GEN_SAFEKEY', self.cmd_GEN_SAFEKEY,
                               desc=self.cmd_GEN_SAFEKEY_help)  
                               

    cmd_CHK_SAFEKEY_help = "check safeguard on lasvLaser carving file"
    def cmd_CHK_SAFEKEY(self, gcmd):
        stcode = self.safeguardkey.check_ukey_status()
        self.laststatus = stcode        
        if stcode == 0:
            msg = "safeguard is right"
            gcmd.respond_info(msg)
            pass
        else:
            toolhead = self.printer.lookup_object('toolhead')
            msg = "safeguard fail status on file:"
            m = "%s %d" % (msg,stcode)
            eobj = toolhead.printer.command_error(m)     
            raise eobj  

    cmd_QUERY_SAFEKEY_help = "query safeguard status"
    def cmd_QUERY_SAFEKEY(self, gcmd):
        stcode = self.safeguardkey.check_ukey_status()
        self.laststatus = stcode
        if stcode == 0:
            msg = "safeguard is right"
            gcmd.respond_info(msg)
            pass
        else:
            msg = "safeguard fail status:"
            m = "%s %d" % (msg,stcode)
            gcmd.respond_info(m)    


    cmd_GEN_SAFEKEY_help = "write safe key for usbkey"
    def cmd_GEN_SAFEKEY(self, gcmd):
        allownum = gcmd.get_int('A', minval=0, maxval=10)
        stcode = self.safeguardkey.cmd_save_ukey(RIGHT_ALLOW, repr(allownum))
        if stcode == 0:
            msg = "gen file success:"
            m = "%s %d" % (msg,stcode)
        else:    
            msg = "gen file fail status:"
            m = "%s %d" % (msg,stcode)
        gcmd.respond_info(m)          
 
    def get_status(self, eventtime):
        return {'status': self.laststatus}    

def load_config(config):
    return SafeGuardkeyt(config)
