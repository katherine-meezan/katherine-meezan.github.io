from pyb import Pin, Timer, ADC
from time import sleep_ms, ticks_ms, ticks_diff
from ir_sensor import IR_sensor
from sensor_array import sensor_array

ir_ch4 = IR_sensor(Pin(Pin.cpu.C3, mode=Pin.ANALOG))
ir_ch5 = IR_sensor(Pin(Pin.cpu.A4, mode=Pin.ANALOG))
ir_ch6 = IR_sensor(Pin(Pin.cpu.B0, mode=Pin.ANALOG))
ir_ch7 = IR_sensor(Pin(Pin.cpu.C1, mode=Pin.ANALOG))
ir_ch8 = IR_sensor(Pin(Pin.cpu.C0, mode=Pin.ANALOG))
ir_ch9 = IR_sensor(Pin(Pin.cpu.A5, mode=Pin.ANALOG))
ir_ch10= IR_sensor(Pin(Pin.cpu.C5, mode=Pin.ANALOG))
channels = [ir_ch4, ir_ch5, ir_ch6, ir_ch7, ir_ch8, ir_ch9, ir_ch10]
ir_sensor_array = sensor_array(channels, 4, 8)

centroid_set_point = 0

print("Starting Black Calibration!")
calib_start = ticks_ms()
ir_sensor_array.calibrate_black()
calib_end = ticks_ms()
calib_time = ticks_diff(calib_end, calib_start)
print(f"Calibration complete! Time elapsed: {calib_time/1000}")
print(f"Black Values: {ir_sensor_array.blacks}")
# calib_black.put(0)
# state = 0
