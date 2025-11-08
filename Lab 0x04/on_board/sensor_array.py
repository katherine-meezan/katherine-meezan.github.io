from pyb import Pin, Timer, ADC
from ir_sensor import IR_sensor

class sensor_array:
    # def __init__(self, s1: ir_sensor, s2: ir_sensor, s3: ir_sensor, s4: ir_sensor, s5: ir_sensor, s6: ir_sensor, s7: ir_sensor):
    def __init__(self, sensors: list[IR_sensor], center: int, dist_mm: float):
        self.sensors = sensors
        self.center = center # index of central sensor
        self.dist_mm = dist_mm  # distance in milimeters between sensors
        self.raw_reads = [0 for sensor in sensors]
        self.whites = [0 for sensor in sensors]
        self.blacks = [0 for sensor in sensors]

    # CALCULATE AVERAGE OVER ONE HUNDRED READINGS FOR EACH SENSOR.
    def calibrate_black(self):
        reads_calib = 0
        sums = [0 for sensor in self.sensors]
        for i in range(100):
            for j in range(len(self.sensors)):
                sums[j] += self.sensors[j].read() # sum readings so they can be averaged
            reads_calib += 1
        for i in range(len(self.sensors)):
            self.sensors[i].set_black(sums[i]/reads_calib)
            self.blacks[i] = sums[i]/reads_calib
    def calibrate_white(self):
        reads_calib = 0
        sums = [0 for sensor in self.sensors]
        for i in range(100):
            for j in range(len(self.sensors)):
                sums[j] += self.sensors[j].read()
            reads_calib += 1
        for i in range(len(self.sensors)):
            self.sensors[i].set_white(sums[i] / reads_calib)
            self.whites[i] = sums[i] / reads_calib
    def array_read(self):
        for i in range(len(self.sensors)):
            self.raw_reads[i] = self.sensors[i].read()

    def find_centroid(self):
        # sum_vals = 0
        # weighted_sum = 0
        # for i in range(len(self.sensors)):
        #     raw_val = self.sensors[i].read()
        #     self.norm_val = (self.sensors[i].get_white()-white)/(self.sensors[i].get_white-self.sensors[i].get_black())
        #     sum_vals += self.norm_val
        #     weighted_sum += self.norm_val*i
        # self.centroid = weighted_sum/sum_vals
        sum_vals = 0
        weighted_sum = 0
        for i in range(len(self.sensors)):
            raw_val = self.sensors[i].read() # Read raw sensor value
            norm_val = (raw_val - self.whites[i])/(self.blacks[i] - self.whites[i]) # normalize between calibrated black and white values, cap values at 0 and 1
            sum_vals += norm_val
            weighted_sum += norm_val*(i - self.center)
        centroid = weighted_sum/sum_vals
        centroid = min(centroid, 8)
        centroid = max(centroid, -8)
        return centroid        
    # def sensor_on:

    # def sensor_off:
