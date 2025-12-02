from pyb import Timer, Pin

PB12 = Pin(Pin.cpu.B12, mode=Pin.IN, pull=Pin.PULL_UP) # For Right Bump Sensor
PB13 = Pin(Pin.cpu.B13, mode=Pin.IN, pull=Pin.PULL_UP) # For Left Bump Sensor


# while True:
#     if not PB12.value():
#         print("Right Bump")
#     elif not PB13.value():
#         print("Left Bump")
#     else:
#         continue

while True:
    if (not PB12.value() or not PB13.value()):
        print("Bump")
    else:
        continue