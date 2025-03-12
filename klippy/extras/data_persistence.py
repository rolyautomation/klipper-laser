# data persistence support many file
#
# Copyright (C) 2025-2028  jinqiang <jinqiang@ecomedge.io>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, logging, ast, configparser


#DP_SECTION = 'Variables'
DP_SECTION = 'DataSection'
DP_SECTION_ST = 'datasection'


class DataPersistence:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.dp_name = config.get_name().split()[-1]

        self.filename = os.path.expanduser(config.get('filename'))
        self.allVariables = {}
        try:
            if not os.path.exists(self.filename):
                open(self.filename, "w").close()
            self.loadVariables()
        except self.printer.command_error as e:
            raise config.error(str(e))
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_mux_command(
            "SAVE_DATAFILEP", "FNAME", self.dp_name,
            self.cmd_SAVE_DATAFILEP,
            desc=self.cmd_SAVE_DATAFILEP_help)
        self.gcode.register_mux_command(
            "DELETE_DATAFILEP"  , "FNAME", self.dp_name,
            self.cmd_DELETE_DATAFILEP,
            desc=self.cmd_DELETE_DATAFILEP_help)

    def loadVariables(self):
        allvars = {}
        varfile = configparser.ConfigParser()
        try:
            varfile.read(self.filename)
            if varfile.has_section(DP_SECTION):
                for name, val in varfile.items(DP_SECTION):
                    allvars[name] = ast.literal_eval(val)
        except:
            msg = "Unable to parse existing variable file"
            logging.exception(msg)
            raise self.printer.command_error(msg)
        self.allVariables = allvars

    cmd_SAVE_DATAFILEP_help = "Save arbitrary data to disk file"
    def cmd_SAVE_DATAFILEP(self, gcmd):
        varname = gcmd.get('FIELDN')
        value = gcmd.get('VALUE')
        try:
            value = ast.literal_eval(value)
        except ValueError as e:
            raise gcmd.error("Unable to parse '%s' as a literal" % (value,))
        newvars = dict(self.allVariables)
        newvars[varname] = value
        # Write file
        varfile = configparser.ConfigParser()
        varfile.add_section(DP_SECTION)
        for name, val in sorted(newvars.items()):
            varfile.set(DP_SECTION, name, repr(val))
        try:
            f = open(self.filename, "w")
            varfile.write(f)
            f.close()
        except:
            msg = "Unable to save variable on data persistence"
            logging.exception(msg)
            raise gcmd.error(msg)
        self.loadVariables()


    cmd_DELETE_DATAFILEP_help = "Delete arbitrary data from disk file"
    def cmd_DELETE_DATAFILEP(self, gcmd):
        varname = gcmd.get('FIELDN')
        # value = gcmd.get('VALUE')
        # try:
        #     value = ast.literal_eval(value)
        # except ValueError as e:
        #     raise gcmd.error("Unable to parse '%s' as a literal" % (value,))
        if varname not in self.allVariables:
            gcmd.respond_info("Fieldname '%s' not found" % (varname,))
            return
        # Create new dictionary without the variable to delete
        newvars = dict(self.allVariables)
        del newvars[varname]        
        # newvars = dict(self.allVariables)
        # newvars[varname] = value
        # Write file
        varfile = configparser.ConfigParser()
        varfile.add_section(DP_SECTION)
        for name, val in sorted(newvars.items()):
            varfile.set(DP_SECTION, name, repr(val))
        try:
            f = open(self.filename, "w")
            varfile.write(f)
            f.close()
        except:
            msg = "Unable to save variable after deletion"
            logging.exception(msg)
            raise gcmd.error(msg)
        self.loadVariables()


    def cmd_PROG_VARIABLE(self, varname, value):
        #varname = gcmd.get('VARIABLE')
        #value = gcmd.get('VALUE')
        try:
            value = ast.literal_eval(value)
        except ValueError as e:
            #raise gcmd.error("Unable to parse '%s' as a literal" % (value,))
            msg = "Unable to parse '%s' as a literal" % (value,)
            raise self.printer.command_error(msg)
            
        newvars = dict(self.allVariables)
        newvars[varname] = value
        # Write file
        varfile = configparser.ConfigParser()
        varfile.add_section(DP_SECTION)
        for name, val in sorted(newvars.items()):
            varfile.set(DP_SECTION, name, repr(val))
        try:
            f = open(self.filename, "w")
            varfile.write(f)
            f.close()
        except:
            msg = "Unable to save variable data persistence in program"
            logging.exception(msg)
            #raise gcmd.error(msg)
            raise self.printer.command_error(msg)
        self.loadVariables()

    def get_status(self, eventtime):
        return {DP_SECTION_ST: self.allVariables}

#def load_config(config):
def load_config_prefix(config):
    return DataPersistence(config)    
