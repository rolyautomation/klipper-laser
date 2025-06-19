# move_tracker support
#
# Copyright (C) 2018-2025 
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging



class MoveTracker:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()

        # self.gpio_trigpin  = config.getint('trig_pin', 23, minval=0) 
        # self.last_outval   = 0  
        # self.trig_sw  = config.getint('trig_sw', 1, minval=0) 
        # self.chktrigger_timer = self.reactor.register_timer(self.chktrigger_fun)  
        self.toolhead = None
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('TRACK_MOVE', self.cmd_TRACK_MOVE)  
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.last_start_t = 0
        self.last_end_t = 0
        self.move_t = 0 
        self.last_current_t = 0

    def _handle_ready(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        pass        

    def cmd_TRACK_MOVE(self, gcmd):
        move_cmd = gcmd.get('MOVE', 'G4 P10')
        self.last_current_t = self.printer.get_reactor().monotonic()
        last_move_before = self.toolhead.lookahead.get_last()
        self.gcode.run_script_from_command(move_cmd)
        last_move = self.toolhead.lookahead.get_last()
        self.toolhead.lookahead.flush()
        if last_move is not None and last_move != last_move_before:
            self.accel_t = last_move.accel_t
            self.cruise_t = last_move.cruise_t
            self.decel_t = last_move.decel_t
            self.last_start_t = self.last_current_t +1
            self.last_end_t = self.last_start_t + 2
            self.move_t = self.accel_t + self.cruise_t + self.decel_t 
            self.gcode.respond_info(
                f"accel time: {self.accel_t:.3f}\n"
                f"cruise time: {self.cruise_t:.3f}\n"
                f"decel time: {self.decel_t:.3f}\n" 
                f"current time: {self.last_current_t:.3f}\n"
                f"move start time: {self.last_start_t:.3f}\n"
                f"move end time: {self.last_end_t:.3f}\n"
                f"move duration: {self.move_t:.3f} seconds\n"
                f"time until start: {self.last_start_t - self.last_current_t:.3f} seconds\n"
                f"time until end: {self.last_end_t - self.last_current_t:.3f} seconds"
            )

        else:
            self.gcode.respond_info("no new move added to queue")

    def get_status(self, eventtime):
        return {
            "current_time": self.last_current_t,
            "last_move_start_time": self.last_start_t,
            "last_move_end_time": self.last_end_t,
            "last_move_duration": self.move_t
        }


#def load_config_prefix(config):
def load_config(config):
    return MoveTracker(config)  

