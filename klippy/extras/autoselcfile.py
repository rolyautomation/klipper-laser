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
#CHK_DEV_FILE="/dev/ttyAMA0"
CHK_DEV_FILE="/dev/serial/by-id/usb-mfiber_rp2040_E66368254F456427-if00"



'''
def verify_klipper_device(port='/dev/ttyAMA0', baudrate=250000, timeout=1):
    """
    Verify if Klipper device communication is normal
    Parameters:
    port:  Serial device path, default is'/dev/ttyAMA0 '
    baudrate:  Baud rate, Klipper defaults to 250000
    timeout:  Timeout (seconds)
    returns:
    (bool, str):  (Whether communication is normal, detailed information)
    """
    import serial
    import time
    import binascii

    if not os.path.exists(port):
        return False, f"devf:{port} is not exist"

    if DEBUG_LOG > 0:
        logging.info("verify_klipper_device:%s",port)

    try:
        # Open serial port
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        
        if not ser.is_open:
            return False, f"Unable to open device {port}"
        
        # Clear buffer
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # Send Klipper identify command
        # This is the first command of Klipper communication, used to get device information
        #identify_cmd = b"identify offset=0 count=40\n"
        #identify_cmd = b"identify offset=0 count=10\n"
        #identify_cmd = bytes([0x08, 0x00, 0x01, 0x00, 0x02, 0x03, 0xE7, 0x7E])        
        #identify_cmd = bytes([0x08, 0x00, 0x01, 0x00, 0x28, 0x4F, 0x91, 0x7E])
        #identify_cmd = bytes([0x08, 0x00, 0x01, 0x00, 0x00,0x28, 0x4F, 0x91, 0x7E])
        #identify_cmd = bytes([0x08, 0x01, 0x01, 0x00, 0x28, 0xB5, 0x79, 0x7E])
        identify_cmd = bytes([0x08, 0x10, 0x01, 0x00, 0x28, 0x3F, 0xE6, 0x7E])
        ser.write(identify_cmd)


        # Wait for response
        time.sleep(0.1)
        
        # Read response
        response = b""
        start_time = time.time()
        while time.time() - start_time < timeout:
            if ser.in_waiting > 0:
                response += ser.read(ser.in_waiting)
                break
                #if b"identify_response" in response:
                #    break
            time.sleep(0.1)
        
        # Close serial port
        ser.close()
        
        # Analyze response
        if b"identify_response" in response:
            # Successfully received Klipper response
            return True, f"Device communication normal, received Klipper response: {response[:100]}..."
        else:
            # Did not receive expected response
            if response:
                return False, f"Received unknown response: {binascii.hexlify(response[:100])}"
            else:
                return False, "No response received, device may not be running Klipper firmware or communication parameters are incorrect"
    
    except serial.SerialException as e:
        return False, f"Serial port exception: {str(e)}"
    except Exception as e:
        return False, f"Error during verification: {str(e)}"
'''


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
        #gcode.register_command("CHK_ROTARY_EXIST", self.cmd_CHK_ROTARY_EXIST)                
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


    '''
    def cmd_CHK_ROTARY_EXIST(self, gcmd):
        success, message = verify_klipper_device(CHK_DEV_FILE)
        msg = "no"
        if success:
            msg = "yes"    
        msg = "%s:%s" % (msg, message)
        gcmd.respond_info(msg)
    '''
  

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

