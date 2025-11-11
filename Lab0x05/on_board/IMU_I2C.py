from pyb import I2C
import time
import struct

"""! BNO055 Memory addresses !"""

op_mode_addr = 0x3D # Page 70 of the BNO055 data sheet
"""! Operation mode byte:
    Bits 4-7 reserved
    Bits 0-3 Calibration Status
    
    !"""

calib_stat_addr = 0x35 # Pg 67, Calibration Status Byte
calib_coeff_addr = 0x55 # Pg 50, Calibration Coefficients Starting Address
eul_head_lsb = 0x1A #Pg 52, Heading for Euler angle Starting Address
"""!
Calibration Status Byte bit map;
3 indicates fully calibrated, 0 indicates not calibrated

Bit 0 and 1 - Mag calib status
Bit 2 and 3 - Acc
Bit 4 and 5 - Gyr
Bit 6 and 7 - Sys

!"""

acc_data_x_lsb = 0x08 # Mem location where 
gyr_data_x_lsb = 0x14 # Pg 52, gyro data

    
"""! Data to write to BNO055 !"""
config_op_mode = 0b0000 # Page 21
IMU_op_mode = 0b1000 # Page 21
compass_mode = 0b1001 # Page 21
m4g_mode = 0b1010 # Page 21
ndof_fmc_off_mode = 0b1011 # Page 21
ndof = 0b1100 # Page 21

class IMU_I2C:
    def __init__(self, I2Cobj, address): # Takes in I2C object configured in Controller mode
        self.I2Cobj = I2Cobj
        self.address = address
        
    def changeOpMode(self, mode):  # Change operating status to "fusion" mode provided by BNO055
        self.I2Cobj.mem_write(config_op_mode, op_mode_addr, timeout=5000, addr_size=8)
        time.sleep(0.01)
        self.I2Cobj.mem_write(mode, op_mode_addr, timeout=5000, addr_size=8)
        
    def retrieveCalStatus(self): # Retrieve and parse calibration status byte
        calStatus = self.I2Cobj.mem_read(1, self.address, calib_stat_addr, timeout=5000)[0]
        sys = (calStatus >> 6) & 0x03
        gyro = (calStatus >> 4) & 0x03
        acc = (calStatus >> 2) & 0x03
        mag = calStatus & 0x03
        return (sys, gyro, acc, mag)
        
    def retrieveCalCoefficients(self):  # Retrieve calibration coefficients from IMU as bianary data
        calData = bytearray(self.I2Cobj.mem_read(22, self.address, calib_coeff_addr, timeout=5000))
        return calData
    
    
    def writeCalCoefficients(self, calFileTxt): # Writes cal coefficients from pre-recorded bianary data
        with open(calFileTxt, "rb") as f: # calFileTxt is string of cal text file stored on the board
            calData = bytearray(f.read(22)) # Read text file and store into byte array
        self.I2Cobj.mem_write(calData, self.address, calib_coeff_addr, timeout=5000, addr_size=8)
    
    def readEulerAngles(self): # Reads Euler Angles from the IMU
        get_eulers = self.I2Cobj.mem_read(6, self.address, eul_head_lsb)
        h, r, p = struct.unpack('<hhh', get_eulers)
        # Pg 35,  16 units per degree conversion
        return (h / 16.0, r / 16.0, p / 16.0)
    
    def readAngluarVelocity(self): # Reads angular velocity from the IMU
        get_ang_vel = self.I2Cobj.mem_read(6, self.address, gyr_data_x_lsb)
        x, y, z = struct.unpack('<hhh', get_ang_vel)
        return (x, y, z)

