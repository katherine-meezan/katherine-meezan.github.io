from pyb import Timer, Pin, I2C

BNO055_ADDRESS_B = 0x29

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

#Calibration data start

i2c = I2C(2)                             # create on bus 1
i2c = I2C(2, I2C.CONTROLLER)             # create and init as a controller
i2c.init(I2C.CONTROLLER, baudrate=20000) # init as a controller
# i2c.init(I2C.PERIPHERAL, addr=0x29)      # init as a peripheral with given addressss
# i2c.deinit()                             # turn off the I2C unit
devs = i2c.scan()
print(devs)

def setMode(mode: int):
    i2c.mem_write(mode, BNO055_ADDRESS_B, BNO055_OPR_MODE_ADDR)

while not i2c.is_ready(BNO055_ADDRESS_B):
    continue

# wait for the device to finish calibrating
calibrated = False
calib_buffer = bytearray([0])
while not calibrated:
    cal_stat = i2c.mem_read(1, BNO055_ADDRESS_B,BNO055_CALIB_STAT_ADDR, addr_size=8)
    cal_stat = calib_buffer[0]
    mag_stat = cal_stat[0]&0b11
    acc_stat = (cal_stat[0]>>2) & 0b11
    gyr_stat = (cal_stat[0]>>4) & 0b11
    sys_stat = (cal_stat[0]>>6) & 0b11
    if sys_stat==0b11:
        calibrated = True
# read the calibration profile
calib_profile_buffer = bytearray([0]*22)
i2c.mem_read(calib_profile_buffer, BNO055_ADDRESS_B,0x55, addr_size=8)








