# Support bind mcu by name 
#
# Copyright (C) 2024-2028  jinqiang <jinqiang@ecomedge.io>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#import os, logging, ast, configparser
import os, logging

UDEV_PATH  =  '/dev/serial/by-id/'        
#CONFIG_PATH_F = '~/printer_data/config/mainsail.cfg'
CONFIG_PATH = '~/printer_data/config/'
FILE_EXT_PNM = '_mcu.cfg'
MAIN_MCUD = 'mcu'
MAIN_MCUFN = 'mdctl'
PART_SERSTR = 'serial'
SPEC_FIBER  = 'fiber'
SPEC_GALVO  = 'galvo'
SPEC_MAIN  =  'main'
#mmain(must), mgalvo(must), mfiber(option), mroller（UART）  

class BindmcuByName:
    def __init__(self, config):
        self.printer = config.get_printer()

        self.laststatus = 0
        self.devlistf = None
        self.savepath = os.path.expanduser(CONFIG_PATH)

        #self.type
        self.laserlnkfn = config.get("laserlnkfn")
        self.fiberfn = config.get("fiberfn")
        self.semifn = config.get("semifn")

        if self.laserlnkfn is None or self.fiberfn is None or self.semifn is None:
            raise config.error("please laser type link filename") 

        usbmcutemp = config.get('usbmcu', None)
        mcuconfigtemp = config.get('mcuconfig', None)
        self.usbmcu = None
        self.mcuconfig = None        
        if usbmcutemp is not None:
            self.usbmcu = [d.strip() for d in usbmcutemp.split(',')]   
        if mcuconfigtemp is not None:
            self.mcuconfig = [d.strip() for d in mcuconfigtemp.split(',')]   

        if self.mcuconfig is None or self.usbmcu is None :   
            raise config.error("please config usbmcu and mcuconfig") 

        if len(self.usbmcu) != len(self.mcuconfig) :
            raise config.error("please check usbmcu and mcuconfig num")    


        webhooks = self.printer.lookup_object('webhooks')
        webhooks.register_endpoint(
            "mcu_dev/list", self._handle_devlist_request
        )
        webhooks.register_endpoint(
            "mcu_dev/update", self._handle_devupdate_request
        )

        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('BMCU_LIST', self.cmd_BMCU_LIST,
                               desc=self.cmd_BMCU_LIST_help)            

        gcode.register_command('BMCU_UPDATE', self.cmd_BMCU_UPDATE,
                               desc=self.cmd_BMCU_UPDATE_help) 

        gcode.register_command('BMCU_CHK_UPDT', self.cmd_BMCU_CHK_UPDT,
                               desc=self.cmd_BMCU_CHK_UPDT_help)    

        gcode.register_command('SEL_LASER_TYPE', self.cmd_SEL_LASER_TYPE,
                               desc=self.cmd_SEL_LASER_TYPE_help)                                                                  

    def update_info_tofile(self, serialmcustr, mcuconfigname):
        iret = 0
        fmcuconfig = mcuconfigname
        fmcufilen = mcuconfigname
        if mcuconfigname == MAIN_MCUD:
            fmcuconfig = ''
            fmcufilen = MAIN_MCUFN
        mcufilename = self.savepath + fmcufilen + FILE_EXT_PNM
        self.mcu_file = None
        needupdatef = 0
        try:
            linecnt = 0
            self.mcu_file = open(mcufilename, "r")
            data = self.mcu_file.read()
            for line in data.split('\n'):
                if MAIN_MCUD in line:
                    if mcuconfigname in line:
                        linecnt = linecnt + 1
                    else:
                        mainmcu = "[mcu]"
                        if mainmcu in line:
                            linecnt = linecnt + 1
                elif PART_SERSTR in line:
                    if serialmcustr in line:
                        linecnt = linecnt + 1
            if linecnt < 2:
                needupdatef = 1
            else:
                needupdatef = 0  
                logging.info("\nsame_%s:%s\n", mcuconfigname,serialmcustr)           
            self.mcu_file.close()
        except:
            needupdatef = 1

        if   needupdatef > 0:
            try:
                with open(mcufilename, 'w') as f:
                    if len(fmcuconfig) > 0:
                        linemcu = "[mcu %s]\n" % (fmcuconfig,)
                    else:
                        linemcu = "[mcu]\n"
                    lineserial = "serial: %s\n" % (serialmcustr,)
                    f.write(linemcu)
                    f.write(lineserial)
            except PermissionError:                                     
                iret = 1
        return iret


    def get_serialdev_list(self, devpath):
        iret = 0
        try:
            devlistfa = os.listdir(devpath)
            self.devlistf = [s for s in devlistfa if len(s) > 5]            
        except FileNotFoundError:
            iret = 1
        except PermissionError: 
            iret = 2
        return iret  

    def _handle_devlist_request(self, web_request):  
        pass                             

    def _handle_devupdate_request(self, web_request):  
        pass   

    cmd_BMCU_LIST_help = "list bind mcu name"
    def cmd_BMCU_LIST(self, gcmd):
        stcode = self.get_serialdev_list(UDEV_PATH)
        if stcode == 0:
            msg = "list dev mcu:"
            devstr = "null"
            devnum = len(self.devlistf)
            if devnum > 0 :
                devstr = ', '.join(self.devlistf)
            m = "%s %s" % (msg,devstr)    
            gcmd.respond_info(m)
            pass
        else:
            msg = "list dev mcu fail:"
            m = "%s %d" % (msg,stcode)
            gcmd.respond_info(m)
        #self.laststatus = stcode     

    #retcode: 1-2 is right , other is wrong
    def check_bmcu_list_status(self):
        isupporttype = 10
        stcode = self.get_serialdev_list(UDEV_PATH)
        if stcode == 0:
            flag_list = [0] * len(self.usbmcu)
            rec_dict = {}
            repeatflag = 0
            nouserflag = 0
            devnum = len(self.devlistf)
            if devnum > 0 :
                for mcuindev in self.devlistf : 
                    isused = 0 
                    for ind, mcuspecstr in enumerate(self.usbmcu):
                        if mcuspecstr in mcuindev : 
                            isused = 1
                            if flag_list[ind] == 0 :
                                flag_list[ind] = 1 
                                rec_dict[mcuspecstr] = 1
                            else:
                                repeatflag = repeatflag + 1 
                                isupporttype = 9
                    if  isused == 0 :
                        nouserflag = nouserflag + 1
                        isupporttype = 8
                chklen = len(rec_dict)        
                if  nouserflag == 0  and  repeatflag == 0 and chklen > 0:
                        fiber_keys = [key for key in rec_dict.keys() if SPEC_FIBER in key]
                        fiber_flag = len(fiber_keys)
                        galvo_mk = [key for key in rec_dict.keys() if SPEC_GALVO in key]
                        galvo_flag = len(galvo_mk)
                        main_mk = [key for key in rec_dict.keys() if SPEC_MAIN in key]
                        main_flag = len(main_mk)                                                
                        if main_flag > 0 and galvo_flag > 0 and  fiber_flag > 0:
                            isupporttype = 2
                        elif main_flag > 0 and galvo_flag > 0:       
                            isupporttype = 1
                        else:
                            isupporttype = 7

            else:
                isupporttype = 10  
            return isupporttype

    cmd_BMCU_CHK_UPDT_help = "check update bind mcu name when right"
    def cmd_BMCU_CHK_UPDT(self, gcmd):
        isuppcode = self.check_bmcu_list_status()
        if isuppcode > 0 and isuppcode < 3 :
            stcode = 0
            self.laststatus = 0
        else:
            stcode =  isuppcode  
            self.laststatus = stcode 
        if stcode == 0:
            flag_list = [0] * len(self.usbmcu)
            runst_list = []
            devnum = len(self.devlistf)
            if devnum > 0 :
                for mcuindev in self.devlistf : 
                    isused = 0 
                    for ind, mcuspecstr in enumerate(self.usbmcu):
                        if mcuspecstr in mcuindev : 
                            isused = 1
                            if flag_list[ind] == 0 :
                                fst = self.update_info_tofile(mcuindev,self.mcuconfig[ind])
                                if fst > 0 :
                                    onest = "%s: %d" % (mcuindev,fst)
                                    runst_list.append(onest)                                     
                                flag_list[ind] = 1  
                            else:
                                onest = "%s %s" % ("repeat:",mcuindev)
                                runst_list.append(onest)  
                    if  isused == 0 :
                        onest = "%s %s" % ("nomach:",mcuindev)
                        runst_list.append(onest) 
                if len(runst_list) > 0:
                    errorstr = ', '.join(runst_list)
                    msg = "%s %s" % ("error:",errorstr)
                else:
                    msg = "update mcu is successful"                                                                                             
            else:                
                msg = "update mcu is no dev"                
            #msg = "safeguard is right"
            gcmd.respond_info(msg)
            pass
        else:
            msg = "check mcu fail status:"
            m = "%s %d" % (msg,stcode)
            gcmd.respond_info(m) 


    cmd_BMCU_UPDATE_help = "update bind mcu name when right"
    def cmd_BMCU_UPDATE(self, gcmd):
        stcode = self.get_serialdev_list(UDEV_PATH)
        #self.laststatus = stcode
        if stcode == 0:
            flag_list = [0] * len(self.usbmcu)
            runst_list = []
            devnum = len(self.devlistf)
            if devnum > 0 :
                for mcuindev in self.devlistf : 
                    isused = 0 
                    for ind, mcuspecstr in enumerate(self.usbmcu):
                        if mcuspecstr in mcuindev : 
                            isused = 1
                            if flag_list[ind] == 0 :
                                fst = self.update_info_tofile(mcuindev,self.mcuconfig[ind])
                                if fst > 0 :
                                    onest = "%s: %d" % (mcuindev,fst)
                                    runst_list.append(onest)                                     
                                flag_list[ind] = 1  
                            else:
                                onest = "%s %s" % ("repeat:",mcuindev)
                                runst_list.append(onest)  
                    if  isused == 0 :
                        onest = "%s %s" % ("nomach:",mcuindev)
                        runst_list.append(onest) 
                if len(runst_list) > 0:
                    errorstr = ', '.join(runst_list)
                    msg = "%s %s" % ("error:",errorstr)
                else:
                    msg = "update mcu is successful"                                                                                             
            else:                
                msg = "update mcu is no dev"                
            #msg = "safeguard is right"
            gcmd.respond_info(msg)
            pass
        else:
            msg = "update mcu fail status:"
            m = "%s %d" % (msg,stcode)
            gcmd.respond_info(m)   

    
    def Set_laser_type_link(self, ltype=0): 
        ist = 1
        curlinkfn =  self.semifn
        if ltype > 0:
            curlinkfn =  self.fiberfn  
        sourcfn = self.savepath + curlinkfn       
        linkpathf = self.savepath + self.laserlnkfn
        try:
            if os.path.islink(linkpathf): 
                cur_targetlink = os.readlink(linkpathf)
                if  curlinkfn not in cur_targetlink :
                    os.unlink(linkpathf) 
                    os.symlink(sourcfn, linkpathf) 
                    ist = 0
                else:
                    ist = 0
                    logging.info("\nstay_%s:%s\n", curlinkfn,cur_targetlink) 

            else:
                os.symlink(sourcfn, linkpathf) 
                ist = 0 
        except Exception as e:
            ist = 2
            m = "An error occurred: %s" % (e,)
            logging.info("\n%s\n", m)                  
        return ist
 

    cmd_SEL_LASER_TYPE_help = "select laser type  fiber or Semiconductor "
    def cmd_SEL_LASER_TYPE(self, gcmd): 
        mtype = gcmd.get_int('S',0, minval=0, maxval=1)
        stcode = self.Set_laser_type_link(mtype)
        msg = "select laser status:"
        m = "%s%d(in=%d)" % (msg,stcode,mtype)
        gcmd.respond_info(m)         

    #def get_status(self, eventtime):
        #return {'status': self.laststatus} 

def load_config(config):
    return BindmcuByName(config)

