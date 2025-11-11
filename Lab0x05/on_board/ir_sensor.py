from pyb import Pin, Timer, ADC

class IR_sensor:
    def __init__(self, input_pin: Pin):
        self.input_pin = input_pin
        self.ADC = ADC(input_pin)
        self.black_val = 0
        self.white_val  =0

    def read(self):
        return self.ADC.read()

    def set_white(self, white_val):
        self.white_val = white_val

    def set_black(self, black_val):
        self.black_val = black_val
    # def sensor_on:

    # def sensor_off:
