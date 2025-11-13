from pyb import Timer, Pin, I2C
from IMU_I2C import IMU_I2C
from time import sleep_ms
import struct

BNO055_ADDRESS_A = 0x28

# Status Registers
BNO055_CALIB_STAT_ADDR = 0X35
BNO055_SELFTEST_RESULT_ADDR = 0X36
BNO055_INTR_STAT_ADDR = 0X37

BNO055_SYS_CLK_STAT_ADDR = 0X38
BNO055_SYS_STAT_ADDR = 0X39
BNO055_SYS_ERR_ADDR = 0X3A

# Unit selection register
BNO055_UNIT_SEL_ADDR = 0X3B

# Mode registers
BNO055_OPR_MODE_ADDR = 0X3D
BNO055_PWR_MODE_ADDR = 0X3E

BNO055_SYS_TRIGGER_ADDR = 0X3F
BNO055_TEMP_SOURCE_ADDR = 0X40

# Axis remap registers
BNO055_AXIS_MAP_CONFIG_ADDR = 0X41
BNO055_AXIS_MAP_SIGN_ADDR = 0X42

"""! Data to write to BNO055 !"""
config_op_mode = 0b0000 # Page 21
IMU_op_mode = 0b1000 # Page 21
compass_mode = 0b1001 # Page 21
m4g_mode = 0b1010 # Page 21
ndof_fmc_off_mode = 0b1011 # Page 21
amg_mode = 0b0111
ndof_mode = 0b1100

#Calibration data start

# # Create Pins for IMU operations
#
# PB10 = Pin(Pin.cpu.B10, mode=Pin.ALT, alt=4) # I2C2 SCL
# PB11 = Pin(Pin.cpu.B11, mode=Pin.ALT, alt=4) # I2C2 SDA
#
# # Create I2C controller object
# i2c = I2C(PB10, PB11, 400000) # BNO055 SCL freq = 400 kHz

I2C_BUS_2 = I2C(2, I2C.CONTROLLER)             # create and init as a controller
I2C_BUS_2.init(I2C.CONTROLLER, baudrate=400000) # init as a controller
# I2C_BUS_2.init(I2C.PERIPHERAL, addr=0x29)      # init as a peripheral with given address

IMU = IMU_I2C(I2C_BUS_2, BNO055_OPR_MODE_ADDR)

devs = []
while devs == []:
    devs = I2C_BUS_2.scan()

def setMode(mode: int):
    I2C_BUS_2.mem_write(mode, BNO055_ADDRESS_A, BNO055_OPR_MODE_ADDR)

print(f"DEVICES ON BUS: {devs}")
while not I2C_BUS_2.is_ready(BNO055_ADDRESS_A):
    continue

print("IMU READY!")

# set mode to make it calibrate
# IMU.changeOpMode(IMU_op_mode)
I2C_BUS_2.mem_write(config_op_mode, BNO055_ADDRESS_A, BNO055_OPR_MODE_ADDR, timeout=50, addr_size=8)
# I2C_BUS_2.mem_write(ndof_mode, BNO055_ADDRESS_A, BNO055_OPR_MODE_ADDR, timeout=5000, addr_size=8)
# I2C_BUS_2.mem_write(amg_mode, BNO055_ADDRESS_A, BNO055_OPR_MODE_ADDR, timeout=5000, addr_size=8)
sleep_ms(20)
I2C_BUS_2.mem_write(IMU_op_mode, BNO055_ADDRESS_A, BNO055_OPR_MODE_ADDR, timeout=50, addr_size=8)
# wait for the device to finish calibrating
calibrated = False
cal_stat = bytearray([0])
self_test = bytearray([0])
syst_status = bytearray([0])
# cal_stat = bytes(1)

while not calibrated:
    I2C_BUS_2.mem_read(self_test, BNO055_ADDRESS_A, BNO055_SELFTEST_RESULT_ADDR, addr_size=8)
    I2C_BUS_2.mem_read(syst_status, BNO055_ADDRESS_A, BNO055_SYS_STAT_ADDR, addr_size=8)
    I2C_BUS_2.mem_read(cal_stat, BNO055_ADDRESS_A, BNO055_CALIB_STAT_ADDR, addr_size=8)
    mag_stat = cal_stat[0]&0b11
    acc_stat = (cal_stat[0]>>2) & 0b11
    gyr_stat = (cal_stat[0]>>4) & 0b11
    sys_stat = (cal_stat[0]>>6) & 0b11
    print(f"MAG: {mag_stat}, ACC: {acc_stat}, GYR: {gyr_stat}, SYS: {sys_stat}, CAL_STAT: {cal_stat[0]}")
    if mag_stat==0b11 & acc_stat==0b11 & gyr_stat==0b11:
        calibrated = True
    # print(f"syst_status = {bin(syst_status[0])}, self_test = {bin(self_test[0])}"
    # print(f"buffer: {calib_buffer}")

# read the calibration profile
calib_profile_buffer = bytearray([0]*22)
I2C_BUS_2.mem_read(calib_profile_buffer, BNO055_ADDRESS_A, 0x55, addr_size=8)
print(calib_profile_buffer)


def readAngluarVelocity():  # Reads angular velocity from the IMU
    gyr_data_x_lsb = 0x14
    get_ang_vel = I2C_BUS_2.mem_read(6, BNO055_ADDRESS_A, gyr_data_x_lsb, timeout=50)
    x, y, z = struct.unpack('<hhh', get_ang_vel)
    return (x, y, z)

while True:
    print(readAngluarVelocity())
    sleep_ms(100)