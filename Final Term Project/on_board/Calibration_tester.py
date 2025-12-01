from pyb import Pin, Timer, ADC
from time import sleep_ms, ticks_ms, ticks_diff
from ir_sensor import IR_sensor
from sensor_array import sensor_array

ir_ch1 = IR_sensor(Pin(Pin.cpu.C3, mode=Pin.ANALOG))
ir_ch3 = IR_sensor(Pin(Pin.cpu.A4, mode=Pin.ANALOG))
ir_ch5 = IR_sensor(Pin(Pin.cpu.B0, mode=Pin.ANALOG))
ir_ch7 = IR_sensor(Pin(Pin.cpu.C1, mode=Pin.ANALOG))
ir_ch9 = IR_sensor(Pin(Pin.cpu.C0, mode=Pin.ANALOG))
ir_ch11 = IR_sensor(Pin(Pin.cpu.C4, mode=Pin.ANALOG))
ir_ch13= IR_sensor(Pin(Pin.cpu.C5, mode=Pin.ANALOG))

PA7 = Pin(Pin.cpu.A7, mode=Pin.OUT)


channels = [ir_ch1, ir_ch3, ir_ch5, ir_ch7, ir_ch9, ir_ch11, ir_ch13]
ir_sensor_array = sensor_array(channels, 4, 8)

centroid_set_point = 0


print("Starting Black Calibration!")
sleep_ms(500)
calib_start = ticks_ms()
ir_sensor_array.calibrate_black()
calib_end = ticks_ms()
calib_time = ticks_diff(calib_end, calib_start)
print(f"Calibration complete! Time elapsed: {calib_time/1000}")
print(f"Black Values: {ir_sensor_array.blacks}")
# calib_black.put(0)
# state = 0
print("Move to white")

sleep_ms(3000)
print("Starting White Calibration!")
sleep_ms(500)
calib_start = ticks_ms()
ir_sensor_array.calibrate_white()
calib_end = ticks_ms()
calib_time = ticks_diff(calib_end, calib_start)
print(f"Calibration complete! Time elapsed: {calib_time/1000}")
print(f"White Values: {ir_sensor_array.whites}")
sleep_ms(1000)

print("calculating centroids")
while True:
    ir_sensor_array.array_read()
    # print(ir_sensor_array.raw_reads)
    sleep_ms(100)
    print(f"{ir_sensor_array.raw_reads}, centroid location: {ir_sensor_array.find_centroid()}")

