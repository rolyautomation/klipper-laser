# Support multi motor single axis modes 
#
# Copyright (C) 2024-2028  jinqiang <jinqiang@ecomedge.io>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import math
import chelper

INACTIVE = 'INACTIVE'
ACTIVE = 'ACTIVE'
AXIS_A =  3


class MultiMotorAxis:
    #VALID_MODES = [PRIMARY, COPY, MIRROR]
    def __init__(self, ma_config, rail_0, rail_1, rail_2, rail_3, axis):
        self.printer = ma_config.get_printer()
        self.axis = axis
        self.ma = tuple(x for x in (rail_0,rail_1,rail_2,rail_3) if x is not None)
        #self.ma = (rail_0, rail_1)
        self.ma_len = len(self.ma)
        self.saved_states = {}
        #safe_dist = ma_config.getfloat('safe_distance', None, minval=0.)
        #self.safe_dist = safe_dist
        self.printer.add_object('multimotor_axis', self)
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command(
                   'SET_MULTIMOTOR_AXIS', self.cmd_SET_MULTIMOTOR_AXIS,
                   desc=self.cmd_SET_MULTIMOTOR_AXIS_help)
        gcode.register_command(
                   'SAVE_MULTIMOTOR_AXIS_STATE',
                   self.cmd_SAVE_MULTIMOTOR_AXIS_STATE,
                   desc=self.cmd_SAVE_MULTIMOTOR_AXIS_STATE_help)
        gcode.register_command(
                   'RESTORE_MULTIMOTOR_AXIS_STATE',
                   self.cmd_RESTORE_MULTIMOTOR_AXIS_STATE,
                   desc=self.cmd_RESTORE_MULTIMOTOR_AXIS_STATE_help)

    def get_rails(self):
        return self.ma
    def get_active_rail(self):
        for rail in self.ma:
            if rail.mode == ACTIVE:
                return rail
        return None

    def get_status_rail(self,index):
        retst = 0
        for i, ma in enumerate(self.ma):
            if i == index:
                if ma.is_active():
                    retst = 1
        return retst 

    def get_curma_index(self):
        index = 0
        for i, ma in enumerate(self.ma):
            if ma.is_active():
                index = i
        return index         

    def inactive_ma_rail_all(self):
        for i, ma in enumerate(self.ma):
            ma_rail = ma.get_rail()
            if ma.is_active():
                ma.inactivate()

    def toggle_active_dc_rail(self, index):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        pos = toolhead.get_position()
        kin = toolhead.get_kinematics()
        #kin.update_limits(self.axis, target_dc.get_rail().get_range())
        pass
    def home(self, homing_state):
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        # Restore the original rails ordering
        #self.toggle_active_dc_rail(0)
    def get_status(self, eventtime=None):
        return {('mmsa_%d' % (i,)) : ma.mode
                for (i, ma) in enumerate(self.ma)}
    def get_kin_range(self, toolhead, mode):
        pos = toolhead.get_position()
        #return (range_min, range_max)
    def get_dc_order(self, first, second):
        if first == second:
            return 0
    def activate_ma_mode(self, index, mode):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        kin = toolhead.get_kinematics()
        if mode == INACTIVE:
            self.ma[index].inactivate()
        else:
            self.ma[index].activate()
            toolhead = self.printer.lookup_object('toolhead')
            toolhead.flush_step_generation()
            pos = toolhead.get_position()
            pos[3] = 0
            newpos = pos
            #kin.set_position_roller(newpos)
            toolhead.set_position(newpos)
        # A axis  zero     
        #kin.update_limits(self.axis, self.get_kin_range(toolhead, mode))

    def _handle_ready(self):
        # Apply the transform later during Klipper initialization to make sure
        # that input shaping can pick up the correct stepper kinematic flags.
        for ma in self.ma:
            ma.apply_transform()
    cmd_SET_MULTIMOTOR_AXIS_help = "Configure the multi motor single axis mode"
    def cmd_SET_MULTIMOTOR_AXIS(self, gcmd):
        index = gcmd.get_int('NUM', minval=-1, maxval=10)
        #mode = gcmd.get('MODE', PRIMARY).upper()
        if index >= self.ma_len:
            raise gcmd.error("Invalid num=%d out of range" % (index,))
        st = self.get_status_rail(index)
        if st == 0 :
            self.inactive_ma_rail_all()
            msg = "%s:%d" % ("Unselected all",index)
            if  index >= 0 :
                mode =  ACTIVE                  
                self.activate_ma_mode(index, mode)
                msg = "%s:%d" % ("Selected",index)
        else:
            msg = "%s:%d" % ("Selected already",index)                           
         gcmd.respond_info(msg)  
    cmd_SAVE_MULTIMOTOR_AXIS_STATE_help = \
            "Save multi motor single axis modes and idnum"
    def cmd_SAVE_MULTIMOTOR_AXIS_STATE(self, gcmd):
        state_name = gcmd.get('NAME', 'default')
        pos = self.printer.lookup_object('toolhead').get_position()
        self.saved_states[state_name] = {
            'mmsa_modes': [ma.mode for ma in self.ma],
            'mmsa_idnum': [ma.get_idnum() for ma in self.ma],
        }
    cmd_RESTORE_MULTIMOTOR_AXIS_STATE_help = \
            "Restore multi motor single axis modes and idnum"
    def cmd_RESTORE_MULTIMOTOR_AXIS_STATE(self, gcmd):
        state_name = gcmd.get('NAME', 'default')
        saved_state = self.saved_states.get(state_name)
        if saved_state is None:
            raise gcmd.error("Unknown MULTIMOTOR_AXIS state: %s" % (state_name,))
        #toolhead = self.printer.lookup_object('toolhead')
        #toolhead.flush_step_generation()
        #pos = toolhead.get_position()
        for i, ma in enumerate(self.ma):
            saved_mode = saved_state['mmsa_modes'][i]
            self.activate_ma_mode(i, saved_mode)


class MultiMotorAxisRail:
    ENC_AXES = [b'a', b'a']
    def __init__(self, rail, axis, idnum, selnum):
        self.rail = rail
        self.axis = axis
        self.idnum = idnum
        self.mode = INACTIVE
        self.offset = 0.
        self.scale = 0.
        if self.idnum == selnum :
            self.scale = 1.
            self.mode = ACTIVE

        ffi_main, ffi_lib = chelper.get_ffi()
        self.ma_stepper_kinematics = []
        self.orig_stepper_kinematics = []
        for s in rail.get_steppers():
            sk = ffi_main.gc(ffi_lib.multimotor_axis_alloc(), ffi_lib.free)
            orig_sk = s.get_stepper_kinematics()
            ffi_lib.multimotor_axis_set_sk(sk, orig_sk)
            # Set the default transform for the other axis
            ffi_lib.multimotor_axis_set_transform(
                    sk, self.ENC_AXES[0], self.scale, self.offset)
            self.ma_stepper_kinematics.append(sk)
            self.orig_stepper_kinematics.append(orig_sk)
            s.set_stepper_kinematics(sk)
    def get_rail(self):
        return self.rail
    def is_active(self):
        return self.mode != INACTIVE
    def get_idnum(self):
        return self.idnum     
    def get_axis_position(self, position):
        #return position[self.axis] * self.scale + self.offset
        return position[AXIS_A] * self.scale + self.offset
    def apply_transform(self):
        ffi_main, ffi_lib = chelper.get_ffi()
        for sk in self.ma_stepper_kinematics:
            ffi_lib.multimotor_axis_set_transform(
                    sk, self.ENC_AXES[0], self.scale, self.offset)
    def activate_onoff(self, selnum, position=None):
        self.offset = 0  
        self.scale = 0.
        self.mode = INACTIVE        
        if self.idnum == selnum :
            self.scale = 1.
            self.mode = ACTIVE                          
        self.apply_transform()
    def activate(self, position=None):
        self.offset = 0  
        self.scale = 1.
        self.mode = ACTIVE                          
        self.apply_transform()
    def inactivate(self, position=None):
        self.offset = 0
        self.scale = 0.
        self.apply_transform()
        self.mode = INACTIVE
