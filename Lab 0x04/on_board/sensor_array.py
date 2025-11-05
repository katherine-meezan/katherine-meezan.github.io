from pyb import Pin, Timer, ADC
import ir_sensor

class sensor_array:

    # def __init__(self, s1: ir_sensor, s2: ir_sensor, s3: ir_sensor, s4: ir_sensor, s5: ir_sensor, s6: ir_sensor, s7: ir_sensor):
    def __init(self, sensors: list[ir_sensor], center: int, dist_mm: float):
        self.sensors = sensors
        self.center = center # index of central sensor
        self.dist_mm = dist_mm  # distance in milimeters between sensors
        self.reads = [0 for sensor in sensors]
    def calibrate_black(self):
        reads_calib = 0
        sums = [0 for sensor in self.sensors]
        for i in range(100):
            for j in range(len(self.sensors)):
            reads_calib += 1
                sums += self.sensors[j].read()
        for i in range(len(self.sensors)):
            self.sensors[i].set_black(sums[i]/reads_calib)
    def calibrate_white(self):
        reads_calib = 0
        sums = [0 for sensor in self.sensors]
        for i in range(100):
            for j in range(len(self.sensors)):
            reads_calib += 1
                sum += self.sensors[j].read()
        for i in range(len(self.sensors)):
            self.sensors[i].set_white(sums[i] / reads_calib)

    def find_centroid(self):
        sum_vals = 0
        weighted_sum = 0
        for i in range(len(self.sensors)):
            raw_val = self.sensors[i].read()
            self.norm_val = (self.sensors[i].get_white()-white)/(self.sensors[i].get_white-self.sensors[i].get_black())
            sum_vals += self.norm_val
            weighted_sum += self.norm_val*i
        self.centroid = weighted_sum/sum_vals
    
    def sensor(self):
        
    # def sensor_on:

    # def sensor_off:
