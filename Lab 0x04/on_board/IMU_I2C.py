"""! BNO055 Memory addresses !"""

op_mode_addr = 0x3D # Page 70 of the BNO055 data sheet
"""! Operation mode byte:
    Bits 4-7 reserved
    Bits 0-3 Calibration Status
    
    !"""

calib_stat_addr = 0x35 # Pg 67, Calibration Status Byte
"""!
Calibration Status Byte bit map;
3 indicates fully calibrated, 0 indicates not calibrated

Bit 0 and 1 - Mag calib status
Bit 2 and 3 - Acc
Bit 4 and 5 - Gyr
Bit 6 and 7 - Sys

!"""

acc_data_x_lsb = 0x08 # Mem location where 
    
    
"""! Data to write to BNO055 !"""
config_op_mode = 0b0000 # Page 21
IMU_op_mode = 0b1000 # Page 21



class IMU_I2C:
    def __init__(self, I2Cobj): # Takes in I2C object configured in Controller mode
        self.I2Cobj = I2Cobj
        
    def changeOpMode(self):  # Change operating status to "fusion" mode provided by BNO055
        self.I2Cobj.mem_write(IMU_op_mode, op_mode_addr, timeout=5000, addr_size=8)
        
    def changeOpModeConfig(self):  # Change operating status to "fusion" mode provided by BNO055
        self.I2Cobj.mem_write(config_op_mode, op_mode_addr, timeout=5000, addr_size=8)
        
    def retrieveCalStatus(self): # Retrieve and parse calibration status byte
        calStatus = self.I2Cobj.recv(1, calib_stat_addr, timeout=5000)
        return calStatus
        
    def retrieveCalCoefficients(self):  # Retrieve calibration coefficients from IMU as bianary data
        calData = bytearray(self.I2Cobj.recv(22, acc_data_x_lsb, timeout=5000))
        return calData
    
    
    def writeCalCoefficients(self, calFileTxt): # Writes cal coefficients from pre-recorded bianary data
        with open(calFileTxt, "rb") as f: # calFileTxt is string of cal text file stored on the board
            calData = bytearray(f.read(22)) # Read text file and store into byte array
        self.I2Cobj.mem_write(calData, acc_data_x_lsb, timeout=5000, addr_size=8)
    
    def readEulerAngles(): # Reads Euler Angles from the IMU
    
    
    def readAngluarVelocity(): # Reads angular velocity from the IMU
        

