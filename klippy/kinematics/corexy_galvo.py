# Code for handling the kinematics of corexy and galvo robots
#
# Copyright (C) 2024  Leo QU <leo@rolyautomation.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
import stepper
from . import mmsa_modes
CUR_SEL_A = 0

class CoreXYGalvoKinematics:
    def __init__(self, toolhead, config):
        # Setup axis rails
        self.rails = [stepper.LookupMultiRail(config.getsection('stepper_' + n))
                      for n in 'xyzabc']
        for s in self.rails[1].get_steppers():
            self.rails[0].get_endstops()[0][0].add_stepper(s)
        for s in self.rails[0].get_steppers():
            self.rails[1].get_endstops()[0][0].add_stepper(s)

        """
        self._hradiation_angle = 0.
        self._focus_distance = 0.
        self._half_distance_galvo = 0.
        self.rails[0].setup_itersolve('corexy_stepper_alloc', b'+')
        self.rails[1].setup_itersolve('corexy_stepper_alloc', b'-')        
        """
        
        self.rails[0].setup_itersolve('corexy_stepper_alloc', b'+')
        self.rails[1].setup_itersolve('corexy_stepper_alloc', b'-')
        self.rails[2].setup_itersolve('cartesian_stepper_alloc', b'z')

        self._hradiation_angle = config.getfloat('hradiation_angle', 0.35, minval=0.,
                                      maxval=1)
        self._focus_distance = config.getfloat('focus_distance', 160, minval=10,
                                      maxval=500)

        self._magnify_factor = config.getfloat('magnify_factor', 1, minval=1,
                                      maxval=300)

        #self._half_distance_galvo =  self._focus_distance * math.tan(self._hradiation_angle)
        self._half_distance_galvo =  self._focus_distance * self._hradiation_angle

        logging.info("galvo param at (%.3f %.3f %.2f)",
                     self._hradiation_angle, self._focus_distance, self._half_distance_galvo)

        self.rails[3].setup_itersolve('galvo_stepper_alloc', b'a', 
                                  self._hradiation_angle, self._focus_distance,
                                  self._half_distance_galvo, self._magnify_factor)                        

        self.rails[4].setup_itersolve('galvo_stepper_alloc', b'b', 
                                  self._hradiation_angle,self._focus_distance,
                                  self._half_distance_galvo, self._magnify_factor)     
        self.rails[5].setup_itersolve('galvo_stepper_alloc', b'c',
                                  self._hradiation_angle, self._focus_distance,
                                  self._half_distance_galvo, self._magnify_factor)  

        ranges = [r.get_range() for r in self.rails]
        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.)
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.)    

        self.ma_module = None
        if config.has_section('rollergp_a1'):
            ma1_config = config.getsection('rollergp_a1') 
            ma2_config = None
            ma3_config = None
            if config.has_section('rollergp_a2'):  
                ma2_config = config.getsection('rollergp_a2') 
            if config.has_section('rollergp_a3'):  
                ma3_config = config.getsection('rollergp_a3') 

            self.rails.append(stepper.LookupMultiRail(ma1_config))
            self.rails[6].setup_itersolve('galvo_stepper_alloc', b'a', 
                                  self._hradiation_angle, self._focus_distance,
                                  self._half_distance_galvo, self._magnify_factor) 

            ma_rail_0 =  mmsa_modes.MultiMotorAxisRail(self.rails[3], 3, 0, CUR_SEL_A)
            ma_rail_1 =  mmsa_modes.MultiMotorAxisRail(self.rails[6], 3, 1, CUR_SEL_A)
            ma_rail_2 =  None 
            ma_rail_3 =  None 

            if ma2_config is not None:
                self.rails.append(stepper.LookupMultiRail(ma2_config))
                self.rails[7].setup_itersolve('galvo_stepper_alloc', b'a', 
                                  self._hradiation_angle, self._focus_distance,
                                  self._half_distance_galvo, self._magnify_factor) 
                ma_rail_2 =  mmsa_modes.MultiMotorAxisRail(self.rails[7], 3, 2, CUR_SEL_A)                                   

            if ma3_config is not None:
                self.rails.append(stepper.LookupMultiRail(ma3_config))
                self.rails[8].setup_itersolve('galvo_stepper_alloc', b'a', 
                                  self._hradiation_angle, self._focus_distance,
                                  self._half_distance_galvo, self._magnify_factor)
                ma_rail_3 =  mmsa_modes.MultiMotorAxisRail(self.rails[8], 3, 3, CUR_SEL_A)                                   

            self.ma_module = mmsa_modes.MultiMotorAxis(
                                                ma1_config, ma_rail_0, ma_rail_1,
                                                ma_rail_2, ma_rail_3, 3)

        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        config.get_printer().register_event_handler("stepper_enable:motor_off",
                                                    self._motor_off)

        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', max_velocity, above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat(
            'max_z_accel', max_accel, above=0., maxval=max_accel)
        '''
        self.max_g_velocity = config.getfloat(
            'max_g_velocity', None, above=0.)
        self.max_g_accel = config.getfloat(
            'max_g_accel', None, above=0.)  
        '''  
        self.mech_enable_alone = config.getfloat(
            'mech_enable_alone', 0, above=0.)

        self.max_m_velocity = config.getfloat(
            'max_m_velocity', max_velocity, above=0., maxval=max_velocity)
        self.max_m_accel = config.getfloat(
            'max_m_accel', max_accel, above=0., maxval=max_accel)
        
        #xyz
        #self.endstop_hit = [(0, 0)] * 3
        self.limits = [(1.0, -1.0)] * (3+3)
        #ranges = [r.get_range() for r in self.rails]
        #self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.)
        #self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.)

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def get_galvo_coord_confactor(self):
        return self._half_distance_galvo        

    def calc_position(self, stepper_positions):
        pos = [stepper_positions[rail.get_name()] for rail in self.rails]
        x = 0.5 * (pos[0] + pos[1])
        y = 0.5 * (pos[0] - pos[1])
        #return [0.5 * (pos[0] + pos[1]), 0.5 * (pos[0] - pos[1]), pos[2]]
        #bx = math.tan(pos[0]-self._hradiation_angle)*self._focus_distance + self._half_distance_galvo
        #cy = math.tan(pos[1]-self._hradiation_angle)*self._focus_distance + self._half_distance_galvo
        #bx = math.tan(pos[4]-self._hradiation_angle)*self._focus_distance + self._half_distance_galvo
        #cy = math.tan(pos[5]-self._hradiation_angle)*self._focus_distance + self._half_distance_galvo     
        bx = (pos[4]/self._magnify_factor-self._hradiation_angle)*self._focus_distance + self._half_distance_galvo
        cy = (pos[5]/self._magnify_factor-self._hradiation_angle)*self._focus_distance + self._half_distance_galvo             
        #return [bx, cy, pos[2]]
        if ( self.ma_module is not None and 0 != self.ma_module.get_curma_index() ):
            offp = self.ma_module.get_curma_index()
            return [x, y, pos[2], pos[5+offp], bx, cy]
        else:
            return [x, y, pos[2], pos[3], bx, cy]

    def soft_homing_BC_AXIS(self): 
        self.rails[4].soft_homing_BC_AXIS()       
        self.rails[5].soft_homing_BC_AXIS() 
         
    #Reserved functions  
    def set_position_roller(self, newpos):
        firstrail = self.rails[3]
        firstrail.set_position(newpos)
        if  self.ma_module is not None:
            for i, rail in enumerate(self.rails):
                if (i > 5):
                    rail.set_position(newpos)                        

    def set_position(self, newpos, homing_axes):
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
            if i in homing_axes:
                self.limits[i] = rail.get_range()
        #reason: A: soft reset zero        
    def note_z_not_homed(self):
        # Helper for Safe Z Home
        self.limits[2] = (1.0, -1.0)
    def home(self, homing_state):
        # Each axis is homed independently and in order
        for axis in homing_state.get_axes():
            rail = self.rails[axis]
            # Determine movement
            position_min, position_max = rail.get_range()
            hi = rail.get_homing_info()
            homepos = [None, None, None, None]
            homepos[axis] = hi.position_endstop
            forcepos = list(homepos)
            if hi.positive_dir:
                forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
            else:
                forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
            # Perform homing
            homing_state.home_rails([rail], forcepos, homepos)
            #reason: A: soft reset zero 
    def _motor_off(self, print_time):
        self.limits = [(1.0, -1.0)] * (3+3)
    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in (0, 1, 2):
            if (move.axes_d[i]
                and (end_pos[i] < self.limits[i][0]
                     or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis XYZ first")
                raise move.move_error()


        for i in (3, 4, 5):
            if (i == 3) :
                continue
            #logging.info("check limits (%.3f %.3f %.2f)",
                     #end_pos[i], self.limits[i][0], self.limits[i][1])                
            if (move.axes_d[i]
                and (end_pos[i] < self.limits[i][0]
                     or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis ABC first")
                raise move.move_error()

    def check_move(self, move):
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (xpos < limits[0][0] or xpos > limits[0][1]
            or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
        #bc
        bpos, cpos = move.end_pos[4:6]
        if (bpos < limits[0+4][0] or bpos > limits[0+4][1]
            or cpos < limits[1+4][0] or cpos > limits[1+4][1]):
            self._check_endstops(move)   

        #if  not move.axes_d[0] and not move.axes_d[1]:  
        #XY is not move
        # todo bc
        '''
        if  not move.axes_d[0] and not move.axes_d[1] and not move.axes_d[2] and not move.axes_d[3]: 
            #if move.axes_d[4] > 0 or move.axes_d[5] > 0 :
            if move.axes_d[4] or move.axes_d[5] :    
                if self.max_g_velocity is not None and self.max_g_accel is not None:
                    move.limit_speed(self.max_g_velocity, self.max_g_accel)
        ''' 
        #if self.mech_enable_alone is not None and self.mech_enable_alone > 0:
        if self.mech_enable_alone > 0:
            if  move.axes_d[0] or  move.axes_d[1] or move.axes_d[2] or move.axes_d[3]: 
                move.limit_speed(self.max_m_velocity, self.max_m_accel)                   

        if not move.axes_d[2]:
            # Normal XY move - use defaults
            return
        # Move with Z - update velocity and accel for slower Z axis
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)
        # todo abc

    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip("xyzabc", self.limits) if l <= h]
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return CoreXYGalvoKinematics(toolhead, config)
