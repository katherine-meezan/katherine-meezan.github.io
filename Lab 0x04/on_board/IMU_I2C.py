"""! BNO055 Memory addresses !"""

opr_mode_addr = 0x3D # Page 70 of the BNO055 data sheet
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

"""! Data to write to BNO055 !"""
config_opr_mode = 0b0000 # Page 21
IMU_opr_mode = 0b1000 # Page 21



class IMU_I2C:
    def __init__(self, I2Cobj): # Takes in I2C object configured in Controller mode
        self.I2Cobj = I2Cobj
        
    def changeOpMode():  # Change operating status to "fusion" mode provided by BNO055
        self.I2Cobj.mem_write(IMU_opr_mode, opr_mode_addr, timeout=5000, addr_size=8)
        
    def retrieveCalStatus(): # Retrieve and parse calibration status byte
        self.I2Cobj.recv(1, calib_stat_addr, timeout=5000)
        
        
    def retrieveCalCoefficients():  # Retrieve calibration coefficients from IMU as bianary data
    
    
    
    
    def writeCalCoefficients(): # Writes cal coefficients from pre-recorded bianary data
    
    
    def readEulerAngles(): # Reads Euler Angles from the IMU
    
    
    def readAngluarVelocity(): # Reads angular velocity from the IMU
        

