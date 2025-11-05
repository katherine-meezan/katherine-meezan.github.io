from pyb import Pin, Timer, ADC
from time import sleep_ms
from ir_sensor import IR_sensor
from sensor_array import sensor_array

ch4 = Pin(Pin.cpu.C3, mode=Pin.ANALOG))
ch5 = Pin(Pin.cpu.A4, mode=Pin.ANALOG))
ch6 = Pin(Pin.cpu.B0, mode=Pin.ANALOG))
ch7 = Pin(Pin.cpu.C1, mode=Pin.ANALOG))
ch8 = Pin(Pin.cpu.C0, mode=Pin.ANALOG))
ch9 = Pin(Pin.cpu.A5, mode=Pin.ANALOG))
ch10= Pin(Pin.cpu.C5, mode=Pin.ANALOG))
channels = [ch4, ch5, ch6, ch7, ch8, ch9, ch10]



out = [0 for l in channels]

while True:
    idx = 0
    for channel in channels:
        while idx < len(channels):
            out[idx] = channels[idx].read()
            idx += 1
    print(out)
    sleep_ms(100)


def line_follower():
    state = 0
    while True:
        if state == 0:
            sensors = sensor_array(channels, 4, 2)
            # Calibration
            state = 1
        if state == 1:
            # wait state
            if r:
                state = 2
            elif t:
                state = 3
            elif black_flag and white_flag:
                state = 4
        if state == 2:
            black_val = sensors.calibrate_black
            sleep_ms(500)
            black_flag = 1
            print("Black has been calibrated")
            state = 1 
        if state == 3:
            white_val = sensors.calibrate_white
            sleep_ms(500)
            white_flag = 1
            print("White has been claibrated")
        if state == 4:
            centroid = sensors.find_centroid
            
    
