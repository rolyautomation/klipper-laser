# Support auto select config file 
# autoselcfile.py
# Copyright (C) 2025-2028  jinqiang <jinqiang@ecomedge.io>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#import sys, os, glob, re, time, logging, configparser, io
#import sys, os, configparser
import os, logging, ast, configparser

DEFAULT_CFGFILE = '~/printer_data/config/configflist.ini'
DEFAULT_USERFILE = '~/printer_data/config/userdata.ini'

DP_SECTION = 'DataSection'
CONFIGN_STR = 'confign'
YAXIS_STR = "y_origin_position"
UDEV_PATH  =  '/dev/serial/by-id/' 
FIBER_STR = "mfiber"  
offset_filelist = 2
DEBUG_LOG = 0
FUN_GPIO_EN = 0


def fiber_is_exist():  
    iret = 0
    try:
        devlistfa = os.listdir(UDEV_PATH)
        devlistf = [s for s in devlistfa if len(s) > 5]            
    except FileNotFoundError:
        iret = -101
    except PermissionError: 
        iret = -102
    stcode = iret
    if stcode == 0:
        #iret = 0
        devnum = len(devlistf)
        if devnum > 0 :
            for mcuindev in devlistf : 
                if FIBER_STR in mcuindev :        
                    iret = 1
                    break
    return iret

def read_data_from_disk_file(filename):
    allVariables = {}
    ufilename = os.path.expanduser(filename)
    if os.path.exists(ufilename):
        allvars = {}
        varfile = configparser.ConfigParser()
        try:
            varfile.read(ufilename)
            if varfile.has_section(DP_SECTION):
                for name, val in varfile.items(DP_SECTION):
                    allvars[name] = ast.literal_eval(val)
        except:
            msg = "Unable to parse existing variable file=%s" % (filename, )
            logging.exception(msg)
        allVariables = allvars
    return allVariables  
                

def get_autoselcfile():
    #open_rpi_gpio6()
    allvars = {}
    allvars = read_data_from_disk_file(DEFAULT_USERFILE)
    yaxis_origin_val = allvars.get(YAXIS_STR, 0)
    if DEBUG_LOG > 0:
        logging.info("read:%s,%d[%s]",DEFAULT_USERFILE,yaxis_origin_val,allvars) 
    ival = fiber_is_exist()
    if ival < 0:
        if DEBUG_LOG > 0:
            logging.info("fiber_is_exist_status:%d",ival)
        ival = 0
    if DEBUG_LOG > 0:
        logging.info("fiber_is_exist:%d",ival)
    if yaxis_origin_val > 0:
        yaxis_origin_val = offset_filelist
    isel = ival + yaxis_origin_val
    if DEBUG_LOG > 0:
        logging.info("isel:%d",isel)
    allvars = read_data_from_disk_file(DEFAULT_CFGFILE)
    filelist = {}
    filelist = allvars.get(CONFIGN_STR,filelist)
    if DEBUG_LOG > 0:
        logging.info("filelist:%s",filelist)
    #istr = str(isel)
    istr = isel    
    filename = filelist.get(istr,"")
    if len(filename) > 0:
        filename = os.path.expanduser(filename) 
    if DEBUG_LOG > 0:
        logging.info("filename:%s",filename)
    return filename


def open_rpi_gpio6():
    iret = 0
    if FUN_GPIO_EN > 0:
        import RPi.GPIO as GPIO
        # Disable warnings
        GPIO.setwarnings(False)
        # Set the GPIO mode
        GPIO.setmode(GPIO.BCM) 
        # Use BCM pin numbering
        # Set up GPIO pin 6 as an output
        GPIO.setup(6, GPIO.OUT)
        # Set GPIO pin 6 high
        GPIO.output(6, GPIO.HIGH)
        iret = 1
    return iret

class AutoSelectConfigFile:
    def __init__(self, config):
        self.printer = config.get_printer()
        # Register commands
        gcode = config.get_printer().lookup_object('gcode')
        gcode.register_command("LOOK_AUTOSELECT_CONFIG", self.cmd_LOOK_AUTOSELECT_CONFIG)
        gcode.register_command("SET_YAXIS_ORIGIN", self.cmd_SET_YAXIS_ORIGIN)         
        self.filename = ""
        self.uservariety_vars = None
        self.yaxis_origin_val = 0
        self.printer.register_event_handler("klippy:connect",
                                    self._handle_connect_init) 

    def _handle_connect_init(self):
        self.uservariety_vars = self.printer.lookup_object('data_persistence uservariety',None)
        if self.uservariety_vars is not None:
            self.yaxis_origin_val = self.uservariety_vars.allVariables.get(YAXIS_STR, 0)                              

    def cmd_LOOK_AUTOSELECT_CONFIG(self, gcmd):
        filename = get_autoselcfile()
        self.filename = filename
        gcmd.respond_info("AutoSelectConfigFile: %s" % (filename,))

    def get_status(self, eventtime):
        return {
                "filename": self.filename,
                "yaxis_origin": self.yaxis_origin_val
                }

    def cmd_SET_YAXIS_ORIGIN(self, gcmd):
        pos = gcmd.get_int('S',1, minval=0, maxval=10) 
        if self.uservariety_vars is not None:        
            if pos == 1:
                self.yaxis_origin_val = pos 
            elif pos == 0:
                self.yaxis_origin_val = pos 
            self.uservariety_vars.cmd_PROG_VARIABLE(YAXIS_STR, repr(self.yaxis_origin_val))
            msg = "yaxis origin=%s " % (self.yaxis_origin_val, ) 
        else:
            msg = "yaxis origin not modified "  
        gcmd.respond_info(msg)  


def load_config(config):
    return AutoSelectConfigFile(config)

