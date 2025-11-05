from pyb import Pin, Timer, ADC

class IR_sensor:
    def __init__(self, input_pin: Pin):
        self.input_pin = input_pin
        self.ADC = ADC(input_pin)
        self.threshold = 0

    



    # def calibrate_white:

    def read(self):
        return self.ADC.read()
    # def sensor_on:

    # def sensor_off: