# report status to telnet, section:tnreportstatus   
#
# Copyright (C) 2024-2028  jinqiang <jinqiang@ecomedge.io>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging


UNDEF_CODE = 1499
UNDEF_STR  = "unknown"

REPORTINFO_CODES = {
    "ok":      [0, "ok"],
    "right":   [9999, "right"],    
    "error:1":  [1001, "Letter was not found"],
    "error:2":  [1002, "Numeric value format is not valid"],
    "error:3":  [1003, "Grbl$system command"],
    "error:4":  [1004, "Negative value"],
    "error:5":  [1005, "Homing cycle"],
    "error:6":  [1006, "Minimum step pulse"],
    "error:7":  [1007, "EEPROM read failed"],
    "error:8":  [1008, "Grb$command cannot be used "],
    "error:9":  [1009, "G-code locked out"],
    "error:10": [1010, "Soft limits cannot be enabled"],
    "error:11": [1011, "Max characters per line exceeded"],
    "error:12": [1012, "Compile Option"],
    "error:13": [1013, "Safety door detected"],
    "error:14": [1014, "Grbl-Mega Only"],
    "error:15": [1015, "Jog target exceeds machine travel"],
    "error:16": [1016, "Jog command"],
    "error:20": [1020, "Unsupported or invalid g-code"], 
    "error:21": [1021, "More than one g-code command"], 
    "error:22": [1022, "Feed rate has not yet been"], 
    "error:23": [1023, "G-code command"], 
    "error:24": [1024, "Two G-code commands"], 
    "error:25": [1025, "A G-code word"], 
    "error:26": [1026, "A G-code command implicitly"],   
    "error:27": [1027, "N line number value"], 
    "error:28": [1028, "A G-code command was sent"],  
    "error:29": [1029, "rbl supports six work coordinate"], 
    "error:30": [1030, "The G53 G-code command"], 
    "error:31": [1031, "There are unused axis words"], 
    "error:32": [1032, "A G2 or G3 arc was commanded"], 
    "error:33": [1033, "The motion command has an invalid target"], 
    "error:34": [1034, "A G2 or G3 arc, traced "], 
    "error:35": [1035, "A G2 or G3 arc"], 
    "error:36": [1036, "There are unused"], 
    "error:37": [1037, "The G43.1 dynamic tool"], 
    "error:38": [1038, "An invalid tool number"],    

    "ALARM:1":  [1101, "Hard limit triggered"],
    "ALARM:2":  [1102, "G-code motion target exceeds machine travel"], 
    "ALARM:3":  [1103, "Reset while in motion"],
    "ALARM:4":  [1104, "Probe fail triggered"],
    "ALARM:5":  [1105, "Probe fail triggered"],
    "ALARM:6":  [1106, "Homing fail triggered"],
    "ALARM:7":  [1107, "Homing fail triggered"],
    "ALARM:8":  [1108, "Homing fail triggered"],
    "ALARM:9":  [1109, "Homing fail triggered"],
    "Hold:0":   [1200, "Hold complete"],
    "Hold:1":   [1201, "Hold in-progress"],
    "Door:0" :  [1300, "Door closed. Ready to resume"],
    "Door:1" :  [1301, "Machine stopped"],
    "Door:2" :  [1302, "Door opened"],
    "Door:3" :  [1303, "Door closed and resuming"],

    "FileEnd" :  [1401, "file is over"],
    "WebCancel" :  [1402, "cancelbymainsail"],  
    "Webpause" :  [1403, "pasuebymainsail"], 
    "Webresume" :  [1404, "resumebymainsail"],  
    "Opendoor" :  [1405, "Opendoorbyuser"],
    "Closedoor" :  [1406, "Closedoorbyuser"],                  
    UNDEF_STR :  [UNDEF_CODE, UNDEF_STR], 
           
}


FILTER_COMMANDS = [1401,1402,1403,1404]




class TnreportStatus:
    def __init__(self, config):
        self.printer = config.get_printer()

        wh = self.printer.lookup_object('webhooks')
        wh.register_endpoint("gcode/updatestatus", self._handle_updatestatus_web)
        self.reportcode = 0
        self.reportstr  = "ok"
        #self.prnstatus  = "idle"
        self.gcode = self.printer.lookup_object('gcode')
        #self.gcode.register_command("ENTER_INPUT_ECODE", self.cmd_ENTER_INPUT_ECODE)  
        self.gcode.register_command("REPORT_ECODE", self.cmd_ENTER_INPUT_ECODE)  
        

        self.printer.register_event_handler("reportstatus:updatenewstatus",
                                            self.handle_receivestatus)           

        #by user:self.printer.send_event("reportstatus:updatenewstatus", "error:1") 

    def handle_receivestatus(self, reportstr):
        logging.info("receivestatus=%s\n",reportstr)
        info = REPORTINFO_CODES.get(reportstr)            
        if info is not None:
            if info[0] not in FILTER_COMMANDS:
                self.send_report_exception(info[0], info[1])
                #self.reportcode = info[0]
                #self.reportstr  = info[1] 
            else:
                logging.info("receivestatus filter=%s\n",reportstr)                          
        else:
            logging.info("receivestatus undefined=%s\n",reportstr)
            self.reportcode = UNDEF_CODE
            self.reportstr  = UNDEF_STR  
        
    def cmd_ENTER_INPUT_ECODE(self, gcmd):
        reportcode =  gcmd.get_int('E', 0, minval=0)  
        reportstr  =  "not found"    
        found_key = None
        found_msg = None

        msg = "%s:%d,%s" % ("overecode:",self.reportcode,self.reportstr)  
        logging.info(f"{msg}")        
        for key, value in REPORTINFO_CODES.items():
            if value[0] == reportcode:
                found_key = key
                found_msg = value[1]
                break
        if found_key is not None:
            reportstr = found_key
            self.send_report_exception(reportcode, reportstr)
  
        msg = "%s:%d,%s" % ("inputerrorcode:",reportcode,reportstr)                           
        gcmd.respond_info(msg) 

    def send_report_exception(self, reportcode, reportstr):
        self.reportcode = reportcode
        self.reportstr  = reportstr

   
    def _handle_updatestatus_web(self, web_request):
        self.send_report_exception(0, "ok")


    def get_status(self, eventtime):
        return {
            'reportcode': self.reportcode,            
            'reportstr': self.reportstr
        }


def load_config(config):
    return TnreportStatus(config)
     
