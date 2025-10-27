#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 23 09:23:36 2025

@author: katherinemeezan
"""


# class ClosedLoopControl:
#
#     def __init__(self, ref_value, meas_value, Kp):
#         self.ref_value = ref_value
#         self.meas_value = meas_value
#         self.Kp = Kp
#
#     def correct_error(self):
#         v_error = self.meas_value - self.ref_value  # Error (e)
#         a = v_error * self.Kp  # Output of block
#         return a

from time import ticks_diff
class CLMotorController():
    def __init__(self, target, old_ticks, old_state, Kp=0.1, Ki=0.1, min_sat=-100, max_sat=100, t_init=0, gain=0.01,
                 v_nom=5.0, threshold=4.0):
        # super().__init__(target, old_ticks, old_state, Kp, Ki, min_sat, max_sat, t_init)
        self.target = target
        self.old_ticks = old_ticks
        self.old_state = old_state
        self.Kp = Kp
        self.Ki = Kp
        self.min_sat = min_sat
        self.max_sat = max_sat
        self.error = 0
        self.acc_error = 0
        self.dt = 1
        self.t_init = t_init
        self.gain = gain
        self.v_nom = v_nom
        self.v_bat = self.v_nom
        self.bat_gain = self.v_bat/self.v_nom
        self.threshold = threshold
        self.K1 = 65.35 # (encoder counts/sec)/ %pwm # PLACEHOLDER VALUE
        self.K2 = 30 # wheel degrees per motor count
        self.K3 = 1/self.K1
    def set_Kp(self, Kp):
        self.Kp = Kp

    def set_Ki(self, Ki):
        self.Ki = Ki

    def set_min_sat(self, min_sat):
        self.min_sat = min_sat

    def set_target(self, target):
        self.target = target
    def set_battery(self, v_bat):
        # sets the battery level and calculates the gain
        self.v_bat = v_bat
        self.bat_gain = self.v_bat/self.v_nom

    def get_action(self, new_ticks, new_state):
        # new_state is a velocity in counts/sec
        # new_ticks is a count in microseconds
        # To calculate error, first convert set point in effort to counts/sec
        # Scale for battery droop
        self.error = (self.target*self.K1*self.bat_gain - new_state)*self.K2
        self.dt = ticks_diff(new_ticks, self.old_ticks)/1E6
        self.acc_error = self.acc_error + self.error*self.dt #Integral error, equivalent to degrees
        # do control algorithm
        total = self.Kp*self.error + self.Ki*self.acc_error
        if total > self.max_sat:
            total = self.max_sat
        elif total < self.min_sat:
            total = self.min_sat
        total *= self.K3 # change action into an effort (%pwm) value
        total = max(total, self.min_sat)
        total = min(total, self.max_sat)
        return total


