# Code for handling the kinematics of corexy and galvo robots
#
# Copyright (C) 2024  Leo QU <leo@rolyautomation.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
import stepper
from . import mmsa_modes
#CUR_SEL_A = 0
CUR_SEL_A = -1

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

        # Grab max velocity and accelerations
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.mech_enable_alone = config.getfloat('mech_enable_alone', 0, above=0.)
        self.max_m_velocity = config.getfloat('max_m_velocity', max_velocity, above=0., maxval=max_velocity)
        self.max_m_accel = config.getfloat('max_m_accel', max_accel, above=0., maxval=max_accel)
        self.max_z_velocity = config.getfloat('max_z_velocity', max_velocity, above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', max_accel, above=0., maxval=max_accel)
        self.max_a_velocity = config.getfloat('max_a_velocity', max_velocity, above=0., maxval=max_velocity)
        self.max_a_accel = config.getfloat('max_a_accel', max_accel, above=0., maxval=max_accel)
        self.max_h_velocity = max_velocity
        self.max_h_accel = max_accel
        # Set G0 speeds
        self.g0_galvo_velocity = config.getfloat('g0_speed_galvo', max_velocity, minval=0.)
        self.g0_m_velocity = config.getfloat('g0_speed_xy', 200, minval=0.)
        self.g0_a_velocity = config.getfloat('g0_speed_a', 50, minval=0.)
        self.g0_z_velocity = config.getfloat('g0_speed_z', 50, minval=0.)

        self.g0_m_ratio = config.getfloat('g0_ratio_xy', 0., minval=0.)
        self.g0_a_ratio = config.getfloat('g0_ratio_a', 0., minval=0.)
        self.g0_z_ratio = config.getfloat('g0_ratio_z', 0., minval=0.)

        self.g1_m_ratio = config.getfloat('g1_ratio_xy', 0., minval=0.)
        self.g1_a_ratio = config.getfloat('g1_ratio_a', 0., minval=0.)
        self.g1_z_ratio = config.getfloat('g1_ratio_z', 0., minval=0.)

        
        # Set placeholder limits
        self.limits = [(1.0, -1.0)] * (3+3)

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def get_galvo_coord_confactor(self):
        return self._half_distance_galvo        

    def calc_position(self, stepper_positions):
        pos = [stepper_positions[rail.get_name()] for rail in self.rails]
        x = 0.5 * (pos[0] + pos[1])
        y = 0.5 * (pos[0] - pos[1])   
        bx = (pos[4]/self._magnify_factor-self._hradiation_angle)*self._focus_distance + self._half_distance_galvo
        cy = (pos[5]/self._magnify_factor-self._hradiation_angle)*self._focus_distance + self._half_distance_galvo
        
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

    def jogrun_sta(self, axes_xyz):
        # Each axis is homed independently and in order
        #for axis in axes_xyz:
        axis = axes_xyz[0]
        rail = self.rails[axis]
            # Determine movement
        position_min, position_max = rail.get_range()
            #hi = rail.get_homing_info()
            #homepos = [None, None, None, None]
            #homepos[axis] = hi.position_endstop
            #forcepos = list(homepos)
            #if hi.positive_dir:
            #    forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
            #else:
            #    forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
            # Perform homing
            #jogrun_state.jogrun_rails([rail], forcepos, homepos)
            #jogrun_state.jogrun_rails([rail])
            #reason: A: soft reset zero
        raildata = { 
                    'rail':[rail],
                    'min' : position_min,
                    'max' : position_max,
                    }            
        return raildata     

    def jogrun_end(self, jogrun_state):
        # Each axis is homed independently and in order
        for axis in jogrun_state.get_axes():
            rail = self.rails[axis]
            # Determine movement
            position_min, position_max = rail.get_range()
            #hi = rail.get_homing_info()
            #homepos = [None, None, None, None]
            #homepos[axis] = hi.position_endstop
            #forcepos = list(homepos)
            #if hi.positive_dir:
            #    forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
            #else:
            #    forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
            # Perform homing
            #jogrun_state.jogrun_rails([rail], forcepos, homepos)
            jogrun_state.jogrun_rails([rail])
            #reason: A: soft reset zero    

    def jogrun_drip(self, jogrun_state):
        # Each axis is homed independently and in order
        for axis in jogrun_state.get_axes():
            rail = self.rails[axis]
            position_min, position_max = rail.get_range()
            # Determine movement
            dripparam = jogrun_state.get_dripparam()
            godist = dripparam[0]
            if dripparam[1] < 2:
                if  dripparam[1] == 1:
                    godist = position_max
                else:
                    godist = position_min  
            #hi = rail.get_homing_info()
            homepos = [None, None, None, None]
            #homepos[axis] = hi.position_endstop
            gopos = list(homepos)
            gopos[axis] = godist
            #if hi.positive_dir:
            #    forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
            #else:
            #    forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
            # Perform homing
            #homing_state.jogrun_rails_drip([rail], forcepos, homepos)
            jogrun_state.jogrun_rails_drip([rail], gopos)

    def get_machine_pos(self):
        axes_xyz = [0,1,2]
        posdata = []
        for axis in axes_xyz:
            rail = self.rails[axis]
            position_min, position_max = rail.get_range() 
            posdata.append([position_min,position_max]) 
        return posdata   

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
            if (move.axes_d[i]
                and (end_pos[i] < self.limits[i][0]
                     or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis ABC first")
                raise move.move_error()

    def check_move(self, move):      
        # Check endstops as needed
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        bpos, cpos = move.end_pos[4:6]
        if (xpos < limits[0][0] or xpos > limits[0][1] or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
        elif (bpos < limits[4][0] or bpos > limits[4][1] or cpos < limits[5][0] or cpos > limits[5][1]):
            self._check_endstops(move)
        elif move.axes_d[2]:
            self._check_endstops(move)

        # For G0 moves, set the speed to preset g0 speed
        if not move.pwmsw:
            move.set_speed_for_g0(self.g0_galvo_velocity)

        # Limit speed based on movement ratios between different axis
        if (move.axes_d[0] or move.axes_d[1] or move.axes_d[2] or move.axes_d[3]):
            m_capped_speed = z_capped_speed = a_capped_speed = self.max_h_velocity
            m_capped_accel = z_capped_accel = a_capped_accel = self.max_h_accel
            
            if move.axes_d[0] or move.axes_d[1]:
                # Baseline: The overall acceleration of the move cannot exceed m-values if there are no galvo moves
                m_ratio = 1.0
                # If there are galvo moves as well, then there is potential to scale up
                if move.axes_d[4] or move.axes_d[5]:
                    if move.axes_d[0] and not move.axes_d[1]:
                        m_ratio = move.move_d / abs(move.axes_d[0])
                    elif move.axes_d[1] and not move.axes_d[0]:
                        m_ratio = move.move_d / abs(move.axes_d[1])
                    elif move.axes_d[0] and move.axes_d[1]:
                        m_d = math.sqrt(sum([move.axes_d[0]**2, move.axes_d[1]**2]))
                        m_ratio = move.move_d / m_d
                if not move.pwmsw:
                    if self.g0_m_ratio > 0:
                        m_ratio = min(m_ratio, self.g0_m_ratio)
                    m_capped_speed = self.g0_m_velocity * m_ratio
                else:
                    if self.g1_m_ratio > 0:
                        m_ratio = min(m_ratio, self.g1_m_ratio)
                    m_capped_speed = self.max_m_velocity * m_ratio
                m_capped_accel = self.max_m_accel * m_ratio
            if move.axes_d[2]:
                z_ratio = move.move_d / abs(move.axes_d[2])
                if not move.pwmsw:
                    if self.g0_z_ratio > 0:
                        z_ratio = min(z_ratio, self.g0_z_ratio)
                    z_capped_speed = self.g0_z_velocity * z_ratio
                else:
                    if self.g1_z_ratio > 0:
                        z_ratio = min(z_ratio, self.g1_z_ratio)
                    z_capped_speed = self.max_z_velocity * z_ratio
                z_capped_accel = self.max_z_accel * z_ratio
            if move.axes_d[3]:
                a_ratio = move.move_d / abs(move.axes_d[3])
                if not move.pwmsw:
                    if self.g0_a_ratio > 0:
                        a_ratio = min(a_ratio, self.g0_a_ratio)
                    a_capped_speed = self.g0_a_velocity * a_ratio
                else:
                    if self.g1_a_ratio > 0:
                        a_ratio = min(a_ratio, self.g1_a_ratio)
                    a_capped_speed = self.max_a_velocity * a_ratio
                a_capped_accel = self.max_a_accel * a_ratio
            
            capped_speed = min(m_capped_speed, z_capped_speed, a_capped_speed)
            capped_accel = min(m_capped_accel, z_capped_accel, a_capped_accel)
            move.limit_speed(capped_speed, capped_accel)
            # if not move.pwmsw:
            #     logging.info("g0_v=%s g0_a=%s \n",capped_speed,capped_accel) 
            #     logging.info("m_s_v=%s z_s_v=%s a_s_v=%s \n",m_capped_speed, z_capped_speed, a_capped_speed)  
            #     logging.info("m_s_a=%s z_s_a=%s a_s_a=%s \n",m_capped_accel, z_capped_accel, a_capped_accel)   
        
    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip("xyzabc", self.limits) if l <= h]
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return CoreXYGalvoKinematics(toolhead, config)
