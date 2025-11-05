from pyb import Pin, Timer, ADC
from time import sleep_ms

ch4 = ADC(Pin(Pin.cpu.C3, mode=Pin.ANALOG))
ch5 = ADC(Pin(Pin.cpu.A4, mode=Pin.ANALOG))
ch6 = ADC(Pin(Pin.cpu.B0, mode=Pin.ANALOG))
ch7 = ADC(Pin(Pin.cpu.C1, mode=Pin.ANALOG))
ch8 = ADC(Pin(Pin.cpu.C0, mode=Pin.ANALOG))
ch9 = ADC(Pin(Pin.cpu.A5, mode=Pin.ANALOG))
ch10= ADC(Pin(Pin.cpu.C5, mode=Pin.ANALOG))
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