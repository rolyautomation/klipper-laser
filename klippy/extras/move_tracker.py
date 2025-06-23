# move_tracker support
#
# Copyright (C) 2018-2025 
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import time
import os
import subprocess


DEBUG_MODE = True

class MoveTracker:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()

        self.max_num  = config.getint('max_num', 10, minval=1) 
        self.time_interval = config.getfloat('time_interval', 0.001, above=0.005, maxval=2)
        self.image_name = config.get('image_name', "g_tracker")

        input_image_dir = config.get('image_dir', "rolyvision/temps")
        self.image_dir = os.path.join(os.path.expanduser('~'), input_image_dir)
        self.camera_cmd = config.get('camera_cmd', "")


        self.image_trigger_timer = self.reactor.register_timer(self.image_trigger_fun)  
        self.image_cindex = 0
        self.image_cindex_max = 0
        self.image_tinterval = 0.0
        self.last_image_time = 0.0

        self.toolhead = None
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('TRACK_MOVE', self.cmd_TRACK_MOVE)  
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        # Register dump webhooks
        webhooks = self.printer.lookup_object('webhooks')
        webhooks.register_endpoint(
            "move_tracker/tracker_time", self._handle_mtracker_request
        )

        self.move_t = 0         
        self.last_start_t = 0
        self.last_end_t = 0
        self.last_current_t = 0
        self.start_system_time = 0
        self.start_system_time_clock = 0


    def _handle_ready(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        pass  

    def image_trigger_fun(self, eventtime):
        next_time = eventtime + self.image_tinterval
        
        if self.last_image_time  < eventtime:
            self.last_image_time = eventtime + self.time_interval
            timestamp = f"{eventtime:.6f}"  
            image_name = f"{self.image_name}_{self.image_cindex}_{timestamp}"
            try:
                if self.camera_cmd:
                    image_path = f"{self.image_dir}/{image_name}"
                    cmd = self.camera_cmd.replace('%FILENAME%', image_path)
                    if DEBUG_MODE:
                        logging.info("cmd: %s", cmd)
                    #subprocess.Popen([cmd])
                    subprocess.Popen(cmd, shell=True)
            except subprocess.CalledProcessError as e:
                logging.error('Failed to capture image: %s' % str(e))  
                return self.reactor.NEVER  
            #image_name = self.image_name + str(self.image_cindex)
            #image_name = f"{self.image_name}_{self.image_cindex}"
            if DEBUG_MODE:
                logging.info("image_name: %s, eventtime: %.3f", image_name, eventtime)

        else:
            if DEBUG_MODE:
                logging.info("time interval too short %.3f, last_image_time: %.3f",self.image_tinterval, self.last_image_time)
            pass
        self.image_cindex += 1
        if self.image_cindex >= self.image_cindex_max:
            if DEBUG_MODE:
                logging.info("finish eventtime: %.3f", eventtime)
            return self.reactor.NEVER
        return next_time    

    def  send_time_camera(self):
        #self.start_system_time_clock
        #self.image_cindex_max
        #self.move_t
        #curl -X POST -H "Content-Type: application/json" -d '{"capture": "low", "stime": 0.2, "mduration": 0.2, "num_times": 3}' http://192.168.1.106:8767/process-capture
        #curl -X POST -H "Content-Type: application/json" -d '{"capture": "low",  "mduration": 2, "num_times": 3}' http://192.168.1.106:8767/process-capture
        cmd = f'''curl -X POST -H "Content-Type: application/json" -d '{{"capture": "low", "stime": {self.start_system_time_clock}, "mduration": {self.move_t}, "num_times": {self.image_cindex_max}}}' http://192.168.1.106:8767/process-capture'''
        if DEBUG_MODE:
            logging.info("cmd: %s", cmd)
        try:    
            subprocess.Popen(cmd, shell=True)
        except subprocess.CalledProcessError as e:
            logging.error('Failed to capture image: %s' % str(e))  
        pass
        

    def cmd_TRACK_MOVE(self, gcmd):
        move_cmd = gcmd.get('MOVE', 'G4 P10')
        total_num = gcmd.get_int('N',3)
        self.image_cindex_max = total_num
        if total_num > self.max_num:
            self.image_cindex_max = self.max_num
        if self.image_cindex_max < 2:
            self.image_cindex_max = 2
        self.image_cindex = 0
        self.image_tinterval = 0.020

        self.start_system_time = self.printer.get_reactor().monotonic()
        self.start_print_time = self.toolhead.get_last_move_time()
        def move_complete(eventtime):
            self.end_system_time = self.printer.get_reactor().monotonic()
            self.end_print_time = eventtime
            duration = self.end_print_time - self.start_print_time
            self.gcode.respond_info(
                 f"print move start: {self.start_print_time:.3f}, end time: {self.end_print_time:.3f}, duration: {duration:.3f} seconds\n"
                 f"start system time: {self.start_system_time:.3f}, end: {self.end_system_time:.3f}, duration: {self.end_system_time - self.start_system_time:.3f} seconds"
            )
            return True


        last_move_before = self.toolhead.lookahead.get_last()
        self.gcode.run_script_from_command(move_cmd)
        last_move = self.toolhead.lookahead.get_last()
        self.toolhead.register_lookahead_callback(move_complete)
        self.toolhead.lookahead.flush()
        #self.toolhead.register_lookahead_callback(move_complete)
        if last_move is not None and last_move != last_move_before:
            self.accel_t = last_move.accel_t
            self.cruise_t = last_move.cruise_t
            self.decel_t = last_move.decel_t
            self.move_t = self.accel_t + self.cruise_t + self.decel_t 
            self.image_tinterval =  self.move_t / (self.image_cindex_max - 1)
            #self.last_start_t =1
            #self.last_end_t = 2
            #self.last_current_t = 0    

            current_time = self.printer.get_reactor().monotonic()
            current_system_time = time.time()
            time_diff = current_system_time - current_time
                        
            est_print_time = self.toolhead.mcu.estimated_print_time(current_time)
            print_time_delta = self.start_print_time - est_print_time
            estimated_end_time = current_time + print_time_delta + self.move_t
            self.last_end_t = estimated_end_time
            self.last_start_t  = self.end_system_time + print_time_delta
            self.last_current_t = current_time
            self.start_system_time_clock = time_diff + self.last_start_t
            self.reactor.update_timer(self.image_trigger_timer, self.last_start_t)
            self.send_time_camera()
            if DEBUG_MODE:
                logging.info("current_time: %.3f, est_print_time: %.3f, print_time_delta: %.3f, estimated_end_time: %.3f",current_time, est_print_time,print_time_delta,estimated_end_time)
            def move_end_callback(eventtime):
                self.end_move_time = eventtime
                self.gcode.respond_info(
                    f"move end time: {self.end_move_time:.3f}\n"
                    f"move duration: {self.end_move_time - self.start_system_time:.3f} seconds"
                )
                return self.printer.get_reactor().NEVER  
        
            if DEBUG_MODE:
                self.printer.get_reactor().register_timer(
                    move_end_callback, 
                    estimated_end_time
                )

            self.gcode.respond_info(
                f"accel time: {self.accel_t:.3f}\n"
                f"cruise time: {self.cruise_t:.3f}\n"
                f"decel time: {self.decel_t:.3f}\n" 
                f"move duration: {self.move_t:.3f} seconds\n"
                f"current time: {self.last_current_t:.3f}\n"
                f"move start time: {self.last_start_t:.3f}\n"
                f"move end time: {self.last_end_t:.3f}\n"  
                f"time until start: {self.last_start_t - self.last_current_t:.3f} seconds\n"
                f"time until end: {self.last_end_t - self.last_current_t:.3f} seconds\n" 
                f"time interval: {self.image_tinterval:.3f} seconds\n"  
                f"csystem time: {current_system_time:.3f} time diff: {time_diff:.3f} seconds"                            
            )

        else:
            self.gcode.respond_info("no new move added to queue")


    def _handle_mtracker_request(self, web_request):
        move_cmd = web_request.get_str('move_cmd', 'G4 P10')
        #total_num = web_request.get_int('total_num', 3)

        self.start_system_time = self.printer.get_reactor().monotonic()
        self.start_print_time = self.toolhead.get_last_move_time()
        def move_complete(eventtime):
            self.end_system_time = self.printer.get_reactor().monotonic()
            self.end_print_time = eventtime
            duration = self.end_print_time - self.start_print_time
            if DEBUG_MODE:
                self.gcode.respond_info(
                    f"print move start: {self.start_print_time:.3f}, end time: {self.end_print_time:.3f}, duration: {duration:.3f} seconds\n"
                    f"start system time: {self.start_system_time:.6f}, end: {self.end_system_time:.6f}, duration: {self.end_system_time - self.start_system_time:.6f} seconds"
                )
            return True

        last_move_before = self.toolhead.lookahead.get_last()
        self.gcode.run_script_from_command(move_cmd)
        last_move = self.toolhead.lookahead.get_last()
        self.toolhead.register_lookahead_callback(move_complete)
        self.toolhead.lookahead.flush()
        #self.toolhead.register_lookahead_callback(move_complete)
        result = {
            'stime': 0,
            'mduration': 0,
            'accel_t': 0,
            'cruise_t': 0,
            'decel_t': 0,
            'msg': 'not new move added to queue'
        }        
        if last_move is not None and last_move != last_move_before:
            self.accel_t = last_move.accel_t
            self.cruise_t = last_move.cruise_t
            self.decel_t = last_move.decel_t
            self.move_t = self.accel_t + self.cruise_t + self.decel_t 
            #self.image_tinterval =  self.move_t / (self.image_cindex_max - 1)

            current_time = self.printer.get_reactor().monotonic()
            current_system_time = time.time()
            time_diff = current_system_time - current_time

            est_print_time = self.toolhead.mcu.estimated_print_time(current_time)
            print_time_delta = self.start_print_time - est_print_time
            estimated_end_time = current_time + print_time_delta + self.move_t
            self.last_end_t = estimated_end_time
            self.last_start_t  = self.end_system_time + print_time_delta
            self.last_current_t = current_time
            self.start_system_time_clock = time_diff + self.last_start_t
            #self.reactor.update_timer(self.image_trigger_timer, self.last_start_t)
            #self.send_time_camera()
            if DEBUG_MODE:
                logging.info("current_time: %.3f, est_print_time: %.3f, print_time_delta: %.3f, estimated_end_time: %.3f",current_time, est_print_time,print_time_delta,estimated_end_time)
                logging.info("start_system_time_clock: %.6f, move_t: %.6f",self.start_system_time_clock, self.move_t)
            def move_end_callback(eventtime):
                self.end_move_time = eventtime
                self.gcode.respond_info(
                    f"move end time: {self.end_move_time:.3f}\n"
                    f"move duration: {self.end_move_time - self.start_system_time:.3f} seconds"
                )
                return self.printer.get_reactor().NEVER  

            if DEBUG_MODE:
                self.printer.get_reactor().register_timer(
                    move_end_callback, 
                    estimated_end_time
                )
            result['stime'] = self.start_system_time_clock
            result['mduration'] = self.move_t
            result['accel_t'] = self.accel_t
            result['cruise_t'] = self.cruise_t
            result['decel_t'] = self.decel_t
            result['msg'] = 'new move added to queue'
        web_request.send(result)


    def get_status(self, eventtime):
        return {
            "start_system_time_clock": self.start_system_time_clock,
            "current_time": self.last_current_t,
            "last_move_start_time": self.last_start_t,
            "last_move_end_time": self.last_end_t,
            "last_move_duration": self.move_t
        }

#def load_config_prefix(config):
def load_config(config):
    return MoveTracker(config)  

