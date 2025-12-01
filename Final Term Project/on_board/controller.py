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
    def __init__(self, target, old_ticks, old_state, Kp=1, Ki=1, min_sat=-100, max_sat=100, t_init=0,
                 v_nom=9.0, threshold=5.0, K3=0.61):
        # super().__init__(target, old_ticks, old_state, Kp, Ki, min_sat, max_sat, t_init)
        self.target = target
        self.old_ticks = old_ticks
        self.old_state = old_state
        self.Kp = Kp
        self.Ki = Ki
        self.min_sat = min_sat
        self.max_sat = max_sat
        self.error = 0
        self.acc_error = 0
        self.dt = 0
        self.t_init = t_init
        self.v_nom = v_nom # expected/ideal battery level
        self.v_bat = self.v_nom
        self.bat_gain = self.v_nom/self.v_bat
        self.threshold = threshold # threshold for battery signal
        self.K1 = 1.637 # (wheel degrees/sec)/ mm/s
        self.K2 = 0.25 # wheel degrees per encoder count
        self.K3 = K3 # effort=%pwm / (wheel degrees/sec)
        self.KWindup = 0.1
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
        self.bat_gain = self.v_nom/self.v_bat

    def get_action(self, new_ticks, new_state):
        # new_state is a velocity in counts/sec
        # new_ticks is a count in microseconds
        # To calculate error, first convert set point in mm/s to wheel deg/sec
        # Scale for battery droop
        raw_error = (self.target*self.K1 - new_state*self.K2) # error in WHEEL DEGREES/SEC
        if raw_error<5000 and raw_error>-5000: # hard-coded method of ignoring faulty encoder reading spikes
            self.error = raw_error
        else:
            self.error = 0
        if(self.old_ticks == 0):
            self.old_ticks = new_ticks
            # print(f"init!: self")
        else:
            self.dt = ticks_diff(new_ticks, self.old_ticks)/1E6
            self.acc_error = self.acc_error + self.error*self.dt #Integral error, equivalent to degrees
            self.old_ticks = new_ticks
        # do control algorithm
        raw_ctrl_sig = (self.Kp*self.error + self.Ki*self.acc_error) # control output in wheel degrees per second
        ctrl_sig = raw_ctrl_sig*self.K3
        # if ctrl_sig>self.max_sat:
        #     ctrl_sig -= self.KWindup*(ctrl_sig-self.max_sat)
        # elif ctrl_sig>self.max_sat:
        #     ctrl_sig -= self.KWindup*(ctrl_sig-self.max_sat)
        # Units: desired in deg/s, err in deg/s, acc in total deg, raw in deg/s, sig in %pwm=effort
        # print(f"desired: {self.target*self.K1}, curr: {new_state*self.K2},Err: {self.error}, Acc: {self.acc_error}, Raw: {raw_ctrl_sig}, Sig: {ctrl_sig}")
        # print(f"CTRL SIG: {ctrl_sig}, bat_gain: {self.bat_gain}")
        ctrl_sig = max(ctrl_sig, self.min_sat) # apply saturation
        ctrl_sig = min(ctrl_sig, self.max_sat)
        if abs(ctrl_sig) >= self.max_sat:
            # Stop integrating if saturated in same direction
            if (ctrl_sig > 0 and self.error > 0) or (ctrl_sig < 0 and self.error < 0):
                self.acc_error -= self.error*self.dt  # undo last integral term

        
        return ctrl_sig

class IRController():
    def __init__(self, target, old_ticks, old_state, Kp=1, Ki=1, min_sat=-100, max_sat=100, t_init=0,
                 K3=1):
        # super().__init__(target, old_ticks, old_state, Kp, Ki, min_sat, max_sat, t_init)
        self.target = target
        self.old_ticks = old_ticks
        self.old_state = old_state
        self.Kp = Kp
        self.Ki = Ki
        self.min_sat = min_sat
        self.max_sat = max_sat
        self.error = 0
        self.acc_error = 0
        self.dt = 0
        self.t_init = t_init
        self.K1 = 1 # no scaling needed since sensor reading and setpoint are both milimeters of deviation
        self.K2 = 1 # no scaling needed, ^
        self.K3 = 1 # sensitivity relation between PI signal and motor speed differential in mm/
    def set_Kp(self, Kp):
        self.Kp = Kp

    def set_Ki(self, Ki):
        self.Ki = Ki

    def set_min_sat(self, min_sat):
        self.min_sat = min_sat

    def set_target(self, target):
        self.target = target

    def get_action(self, new_ticks, new_state):
        # new_ticks is a count in microseconds
        self.error = (self.target*self.K1 - new_state*self.K2)
        if(self.old_ticks == 0):
            self.old_ticks = new_ticks
            # print(f"init!: self")
        else:
            self.dt = ticks_diff(new_ticks, self.old_ticks)/1E6
            self.acc_error = self.acc_error + self.error*self.dt #Integral error, equivalent to degrees
            self.old_ticks = new_ticks
        # do control algorithm
        raw_ctrl_sig = (self.Kp*self.error + self.Ki*self.acc_error) # control output in wheel degrees per second
        ctrl_sig = raw_ctrl_sig*self.K3
        # Units: desired in deg/s, err in deg/s, acc in total deg, raw in deg/s, sig in %pwm=effort
        # print(f"desired: {self.target*self.K1}, curr: {new_state*self.K2},Err: {self.error}, Acc: {self.acc_error}, Raw: {raw_ctrl_sig}, Sig: {ctrl_sig}")
        # print(f"CTRL SIG: {ctrl_sig}, bat_gain: {self.bat_gain}")
        ctrl_sig = max(ctrl_sig, self.min_sat) # apply saturation
        ctrl_sig = min(ctrl_sig, self.max_sat)
        return ctrl_sig