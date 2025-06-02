# Code for handling the kinematics of SCARA robots
#
# Copyright (C) 2025       Tomáš Batelka <tomas.batelka@vofy.cz>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import math
import stepper

class ScaraKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        self.toolhead = toolhead

        stepper_configs = [config.getsection('stepper_' + n) for n in 'abz']
        rail_a = stepper.LookupMultiRail(
            stepper_configs[0], units_in_radians=True)
        rail_b = stepper.LookupMultiRail(
            stepper_configs[1], units_in_radians=True)
        rail_z = stepper.LookupMultiRail(
            stepper_configs[2], units_in_radians=False)
        self.rails = [rail_a, rail_b, rail_z]

        printer_config = config.getsection('printer')
        self.offset_x = printer_config.getfloat('offset_x', default=0.)
        self.offset_y = printer_config.getfloat('offset_y', default=0.)
        self.ecr = printer_config.getfloat('ecr', default=1.)
        self.l1 = printer_config.getfloat('link1_length', above=0.)
        self.l2 = printer_config.getfloat('link2_length', above=0.)
        self.base_protection_radius = printer_config.getfloat(
            'base_protection_radius', default=60.0)
        self.l1_sq = self.l1**2
        self.l2_sq = self.l2**2
        self.limit_xy2 = -1
        self.limit_z = (0, 1)
        self.home_position = [
            float(x) for x in printer_config.get(
                'home_position', default=f"{self.offset_x}, {self.l1 + self.l2 + self.offset_y}, 0"
            ).split(',')
        ]

        self.rails[0].setup_itersolve('scara_stepper_alloc', b'a',
            self.l1, self.l2, self.offset_x, self.offset_y, self.ecr)
        self.rails[1].setup_itersolve('scara_stepper_alloc', b'b',
            self.l1, self.l2, self.offset_x, self.offset_y, self.ecr)
        self.rails[2].setup_itersolve('cartesian_stepper_alloc', b'z')
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        config.get_printer().register_event_handler("stepper_enable:motor_off",
                                                    self._motor_off)

        self.max_velocity, self.max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', self.max_velocity,
            above=0., maxval=self.max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', self.max_accel,
                                          above=0., maxval=self.max_accel)
        self.need_home = True

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def calc_position(self, stepper_positions):            
        a_position = stepper_positions.get(self.rails[0].get_name(), 0.0)
        b_position = stepper_positions.get(self.rails[1].get_name(), 0.0)
        z_pos = stepper_positions.get(self.rails[2].get_name(), 0.0)
            
        x_pos, y_pos = self._forward_kinematics(a_position, b_position)
        return [x_pos, y_pos, z_pos]

    def set_position(self, newpos, homing_axes):
        for rail in self.rails:
            rail.set_position(newpos)
        if 0 in homing_axes and 1 in homing_axes:
            max_reach = self.l1 + self.l2
            self.limit_xy2 = max_reach ** 2
        if 2 in homing_axes:
            self.limit_z = self.rails[2].get_range()
        if tuple(homing_axes) == (0, 1, 2):
            self.need_home = False

    def home(self, homing_state):
        homing_axes = homing_state.get_axes()        
        homing_xy = 0 in homing_axes or 1 in homing_axes
        homing_z = 2 in homing_axes
            
        if homing_xy:
            self.set_position(self.home_position, [0, 1, 2])
            homing_state.set_homed_position(self.home_position)
                        
        if homing_z:
            hi = self.rails[2].get_homing_info()
            position_min, position_max = self.rails[2].get_range()
            homepos = [None, None, None]
            homepos[2] = hi.position_endstop
            forcepos = list(homepos)
            if hi.positive_dir:
                forcepos[2] -= hi.position_endstop - position_min
            else:
                forcepos[2] += position_max - hi.position_endstop
                
            homing_state.home_rails([self.rails[2]], forcepos, homepos)

    def _motor_off(self, _):
        self.need_home = True
        
    def check_move(self, move):
        if self.need_home:
            raise self.printer.command_error("Must home axes first")
        
        start_pos = move.start_pos
        end_pos = move.end_pos
        
        z_min, z_max = self.limit_z        
        if end_pos[2] < z_min - 30 or end_pos[2] > z_max:
            raise self.printer.command_error(
                "End Z position %.1fmm is outside limits (%.1f to %.1fmm)" % 
            (end_pos[2], z_min, z_max))
        
        xy_movement = (end_pos[0] != start_pos[0]) or (end_pos[1] != start_pos[1])
        if not xy_movement:
            return
        
        if end_pos[1] < 0.0:
            raise self.printer.command_error(
                "End position (%.1f, %.1f) is beyond y axis limit (%.1fmm < 0)" % 
                (end_pos[0], end_pos[1], end_pos[1]))
            
        end_distance = math.sqrt(end_pos[0]**2 + end_pos[1]**2)
        max_reach = self.l1 + self.l2
        
        if end_distance > max_reach:
            raise self.printer.command_error(
                "End position (%.1f, %.1f) is beyond arm reach (%.1fmm > %.1fmm)" % 
                (end_pos[0], end_pos[1], end_distance, max_reach))
            
        end_circle_distance = math.sqrt(
            (end_pos[0] - self.l1)**2 + (end_pos[1] - 0.0)**2)
        
        if end_circle_distance < self.l2:
            raise self.printer.command_error(
                "End position (%.1f, %.1f) moves elbow too close to the base (%.1fmm < %.1fmm)" %
                (end_pos[0], end_pos[1], end_circle_distance, self.l2))
                        
        if end_distance < self.base_protection_radius:
            raise self.printer.command_error(
                "End position (%.1f, %.1f) is too close to the base (%.1fmm < %.1fmm)" % 
                (end_pos[0], end_pos[1], end_distance, self.base_protection_radius))
            
        speed_factor = 1.0
        if end_distance > max_reach - 0.999999999:
            speed_factor = 0.0001
        else:
            return
            
        original_velocity = move.max_cruise_v2**0.5
        limited_velocity = original_velocity * speed_factor
        move.limit_speed(limited_velocity, move.accel)
    
    def _forward_kinematics(self, a_position, b_position):
        a = a_position
        b = b_position - a / self.ecr
        x = self.l1 * math.cos(a) + self.l2 * math.cos(a + b) + self.offset_x
        y = self.l1 * math.sin(a) + self.l2 * math.sin(a + b) + self.offset_y
        return [x, y]

    def clear_homing_state(self, axes):
        if 0 in axes or 1 in axes:
            self.limit_xy2 = -1.0
        if 2 in axes:
            self.limit_z = (1.0, -1.0)

    def get_status(self, _):
        xy_home = "xy" if self.limit_xy2 >= 0. else ""
        z_home = "z" if self.limit_z[0] <= self.limit_z[1] else ""
        return {
            'homed_axes': xy_home + z_home,
        }

def load_kinematics(toolhead, config):
    return ScaraKinematics(toolhead, config)
