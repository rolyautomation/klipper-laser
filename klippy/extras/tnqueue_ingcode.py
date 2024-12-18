# telnet send gcode to queue   
#
# Copyright (C) 2024-2028  jinqiang <jinqiang@ecomedge.io>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, sys, logging, io
import queue


DEFAULT_ERROR_GCODE = """
{% if 'heaters' in printer %}
   TURN_OFF_HEATERS
{% endif %}
"""


QUEUE_OVERFLOW_ERROR =  2000
QGCODE_RUN_ERROR     =  2001
QMAXSIZE=32


class TnqueueGcode:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:shutdown",
                                            self.handle_qshutdown)

        #self.trail_mustnewline = 1
        self.trail_mustnewline  = config.getint('tendline', 0, minval=0)  
        self.queue_maxsize  = config.getint('qlenmax', QMAXSIZE, minval=4) 
        self.triggerint_size  = config.getint('qlimitint', 10, minval=4)                                           

        wh = self.printer.lookup_object('webhooks')
        wh.register_endpoint("gcode/qscript", self._handle_qscript)

        #self.queue_maxsize = QMAXSIZE
        self.gcode_queue = queue.Queue(self.queue_maxsize)
        self.queue_usedsize = 0

        self.error_code = 0
        self.warn_code = 0
        self.stopbuttonflag = 0


        # Print Stat Tracking
        #self.print_stats = self.printer.load_object(config, 'print_stats')
        # Work timer
        self.reactor = self.printer.get_reactor()
        self.must_pause_work = self.cmd_from_sd = False
        self.must_exit_work = False
        #self.next_file_position = 0
        self.work_timer = None
        # Error handling
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.on_error_gcode = gcode_macro.load_template(
            config, 'on_error_gcode', DEFAULT_ERROR_GCODE)
        # Register commands
        self.gcode = self.printer.lookup_object('gcode')
        for cmd in ['M411', 'M412', 'M413', 'M414']:
            self.gcode.register_command(cmd, getattr(self, 'cmd_' + cmd))

    def send_error_exception(self, errorcode):
        self.error_code = errorcode

    def clean_error_exception(self):
        self.error_code = 0

    def get_queue_size(self):        
        usedsize = self.gcode_queue.qsize()
        return usedsize,self.queue_maxsize

    def check_trailing_newline(self, qinstr):
        nline = qinstr
        if self.trail_mustnewline > 0:
            endb = qinstr[-1] == '\n' if qinstr else False  
            if not endb:
                nline = qinstr + '\n'           
        return nline       

    def _handle_qscript(self, web_request):
        ingcode = self.check_trailing_newline(web_request.get_str('script'))
        #self.gcode.run_script(web_request.get_str('script'))
        is_full = self.gcode_queue.full()
        if not is_full:
            self.gcode_queue.put(ingcode)
            #logging.info(f"in queue:{ingcode} #e")
            self.do_qstart()    
        else:
            self.send_error_exception(QUEUE_OVERFLOW_ERROR)
            self.do_work_aftererror()

    def handle_qshutdown(self):
        if self.work_timer is not None:
            self.must_pause_work = True
            self.error_code = 0
            self.do_qcancel()

    def stats(self, eventtime):
        if self.work_timer is None:
            return False, ""
        usedsize,totalsize = self.get_queue_size()    
        return True, "queue_sta=%d" % (usedsize/totalsize,)

    def get_status(self, eventtime):
        usedsize,totalsize = self.get_queue_size()
        #totalstatus =  1  if usedsize > totalsize/3 else 0 
        totalstatus =  1  if usedsize > self.triggerint_size else 0 
        return {
            'queue_used': usedsize,
            'queue_maxs': totalsize,
            'queue_status': totalstatus,
            'isactive': self.is_active(),
            'runstatus': [self.error_code, self.warn_code],
        }

    def is_active(self):
        return self.work_timer is not None

    def do_work_aftererror(self):
        try:
            self.gcode.run_script(self.on_error_gcode.render())
        except:
            logging.exception("net queue on_error") 

    def do_qpause(self):
        if self.work_timer is not None:
            self.must_pause_work = True
            #while self.work_timer is not None and not self.cmd_from_sd:
                #self.reactor.pause(self.reactor.monotonic() + .001)

    def do_qstart(self):
        if  self.work_timer is  None:
            self.must_exit_work = False
            self.must_pause_work = False            
            self.work_timer = self.reactor.register_timer(
                     self.work_qhandler, self.reactor.NOW)
                
    def do_qresume(self):
        #if self.work_timer is not None:
        #   raise self.gcode.error("SD busy")
        self.must_exit_work = False
        self.must_pause_work = False
        if  self.work_timer is  None:
            self.work_timer = self.reactor.register_timer(
                     self.work_qhandler, self.reactor.NOW)

    def do_qcancel(self):
        self.must_exit_work = True
        self.must_pause_work = False
        while self.work_timer is not None:
            self.reactor.pause(self.reactor.monotonic() + .001)        

    # G-Code commands
    def cmd_error(self, gcmd):
        raise gcmd.error("net queue not supported")

    def cmd_M414(self, gcmd):
        # clean error
        self.clean_error_exception()
        
    def cmd_M413(self, gcmd):
        # Start/resume telnet print
        self.do_qresume()
    def cmd_M412(self, gcmd):
        # Pause telnet print
        self.do_qpause()

    def cmd_M411(self, gcmd):
        # Reset and stop telnet print
        self.do_qcancel()

    def is_cmd_from_sd(self):
        return self.cmd_from_sd

    def clear_queue(self, q):
        while not q.empty():
            q.get() 
          
    # Background work timer
    def work_qhandler(self, eventtime):
        #logging.info("Starting SD card print (position %d)", self.file_position)
        logging.info("Starting net queue")
        self.reactor.unregister_timer(self.work_timer)
        gcode_mutex = self.gcode.get_mutex()
        partial_input = ""
        lines = []
        error_message = None
        #while not self.must_pause_work:
        while not self.must_exit_work:
            if self.must_pause_work:
                self.reactor.pause(self.reactor.monotonic() + 0.100)
                continue                
            if not lines:
                if not self.gcode_queue.empty():
                    data = self.gcode_queue.get()
                else:
                    self.reactor.pause(self.reactor.monotonic() + 0.100)
                    continue  
                if data:
                    lines = data.split('\n')
                    lines[0] = partial_input + lines[0]
                    partial_input = lines.pop()
                    lines.reverse()
                    logging.info(f"pinput:{partial_input}")
                    self.reactor.pause(self.reactor.NOW)
                    continue  
                else:
                    lines = []
                    logging.info("no data net queue") 
                    self.reactor.pause(self.reactor.monotonic() + 0.100)
                    continue                                          
                #continue
            # Pause if any other request is pending in the gcode class
            if gcode_mutex.test():
                self.reactor.pause(self.reactor.monotonic() + 0.100)
                continue
            # Dispatch command
            self.cmd_from_sd = True
            line = lines.pop()
            #logging.info(f"run:{line}")
            try:
                self.gcode.run_script(line)
            except self.gcode.error as e:
                error_message = str(e)
                try:
                    self.gcode.run_script(self.on_error_gcode.render())
                except:
                    logging.exception("net queue on_error")
                break
            except:
                logging.exception("net queue dispatch")
                break
            self.cmd_from_sd = False

        #logging.info("Exiting SD card print (position %d)", self.file_position)
        logging.info("Exiting net queue")
        self.work_timer = None
        partial_input = ""
        self.clear_queue(self.gcode_queue)
        self.cmd_from_sd = False
        if error_message is not None:
            self.send_error_exception(QGCODE_RUN_ERROR)
        return self.reactor.NEVER

def load_config(config):
    return TnqueueGcode(config)

    
