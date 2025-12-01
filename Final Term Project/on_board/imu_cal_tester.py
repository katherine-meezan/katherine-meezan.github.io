#Test IMU Driver Calibration

from pyb import Timer, Pin, I2C
from IMU_I2C import IMU_I2C
from time import sleep, sleep_ms
import os
from ulab import numpy as np
from time import ticks_ms, ticks_diff  # Use to get dt value in update()

"""! BNO055 Memory addresses !"""

op_mode_addr = 0x3D # Page 70 of the BNO055 data sheet
"""! Operation mode byte:
    Bits 4-7 reserved
    Bits 0-3 Operation Mode    
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
IMU_op_mode = 0b1000 # Page 21, does not use mag
compass_mode = 0b1001 # Page 21
m4g_mode = 0b1010 # Page 21
ndof_fmc_off_mode = 0b1011 # Page 21
ndof = 0b1100 # Page 21
full_sensor_fusion_op_mode = 0x0C # Uses all sensors

IMU_addr = 0x28 # BNO055 device address


# Create Pins for IMU operations

PB10 = Pin(Pin.cpu.B10, mode=Pin.ALT, alt=4) # I2C2 SCL
PB11 = Pin(Pin.cpu.B11, mode=Pin.ALT, alt=4) # I2C2 SDA

# Create I2C controller object
i2c = I2C(2)
i2c = I2C(2, I2C.CONTROLLER)
# i2c = I2C(I2C.CONTROLLER, PB10, PB11, 400000) # BNO055 SCL freq = 400 kHz
# print (i2c)

IMU = IMU_I2C(i2c, IMU_addr) # Create IMU_I2C object

cal_file = "IMU_cal.txt"
os_files = os.listdir()
if cal_file in os_files:
    IMU.changeOpMode(config_op_mode)
    sleep_ms(20)
    IMU.writeCalCoefficients(cal_file)
    sleep_ms(20)
    IMU.changeOpMode(full_sensor_fusion_op_mode)
    sleep_ms(20)
    cal_status = IMU.retrieveCalStatus()
    if cal_status[1]==cal_status[2]==cal_status[3]==3:
        print("IMU calibrated from text file")
        print(cal_status)
    else:
        print("IMU not calibrated for some reason")
        print(cal_status)
else:

    cal_bit = False
    IMU.changeOpMode(full_sensor_fusion_op_mode)
    while not cal_bit:
        cal_status = IMU.retrieveCalStatus()
        # print("Calibration status: sys, gyro, acc, mag")
        sleep_ms(100)
        print(cal_status)
        if (cal_status[1]==3 and cal_status[2]==3 and cal_status[3]==3):
            cal_bit = True
    
    print("IMU Calibrated")
    
    cal_data_bytes = IMU.retrieveCalCoefficients()
    print(cal_data_bytes)
    with open('IMU_cal.txt', 'wb') as f:  # 'wb' = write binary
        f.write(cal_data_bytes)   
    
    print(os.listdir()) # Can be used to check if cal file exists

# def RK4_solver(fcn, x_0, tstep):
#     '''!@brief        Implements a first-order forward euler solver
#         @param fcn    A function handle to the function to solve
#         @param x_0    The initial value of the state vector
#         @param tstep  The step size to use for the integration algorithm
#         @return       A tuple containing both an array of time values and an
#                       array of output values
#     '''
#     # Determine the dimension of the output vector
#     r = len(fcn(0, x_0)[1])
#     # Iterate through the algorithm but stop one cycle early because
#     # the algorithm predicts one cycle into the future
#
#     # Pull out a row from the solution array and transpose to get
#     # the state vector as a column
#     x = xout[[n]].T
#
#     # Pull out the present value of time
#     t = tout[n]
#
#     # Evaluate the function handle at the present time with the
#     # present value of the state vector to compute the derivative
#     _, k1 = fcn(xout[n])
#     _, k2 = fcn(tstep / 2, xout[n] + 0.5 * k1 * tstep)
#     _, k3 = fcn(tstep / 2, xout[n] + 0.5 * k2 * tstep)
#     _, k4 = fcn(tstep, xout[n] + k3 * tstep)
#     xd, y = fcn(t, x)
#
#     # Apply the update rule for RK4 method. The derivative value
#     # must be transposed back to a row here for the dimensions to line up.
#     xout[n + 1] = xout[n] + (k1 + 2 * k2 + 2 * k3 + k4) * tstep / 6
#     yout[n] = y.T
#
#     return tout, yout

A_d = np.array([0.4542,   0.4465,   0.2152,   0.0815,
                0.4465,   0.4542,   0.1958,   0.1210,
                0.1836,   0.1641,   0.2806,  -0.3541,
                0.0659,   0.1054,  -0.3541,   0.8245])
A_d.reshape(4,4)
B_d = np.array([1.6280,   1.0478,  -0.1070,  -0.1082,  -0.0000,  -1.6803,
                1.0478,   1.6280,  -0.0970,  -0.0987,  -0.0000,   1.9724,
                0.4308,   0.3851,   0.3572,   0.3622,   0.0000,  -0.4556,
                0.1547,   0.2474,   0.1758,   0.1783,   0.0000,   1.4227])
B_d.reshape(4,6)
C = np.array([  0,        0,   1.0000,  -70.5000,
                0,        0,   1.0000,  70.5000,
                0,        0,        0,   1.0000,
                -0.2482,  0.2482,   0,         0])
def syst_eq(x, u_aug):
    global A_d, B_d, C_d
    new_x = np.dot(A_d, x) + np.dot(B_d, u_aug)
    new_y = np.dot(C_d, x) + np.dot(0, u_aug)
    return new_x, new_y

old_time = ticks_ms()
x = np.array(np.zeros(4).reshape(4,))
while True:
    curr_time = ticks_ms()
    x[3] = IMU.readEulerAngles()[2]
    x[2] = IMU.readAngularVelocity()[2]
    if ticks_diff(curr_time, old_time) >= 50:
        old_time = curr_time()
        # do the estimation algorithm (IDK HOW, GIVING UP FOR NOW






# if cal_status[1]==cal_status[2]==cal_status[3]==3:
#     print("IMU calibrated")
    
# else: # Process to calibrate the IMU
#     print("IMU not calibrated")
#     IMU.changeOpMode(full_sensor_fusion_op_mode) # Changes op mode to full sensor fusion mode
#     print("Op mode changed")
#     sleep(2)
#     status = IMU.retrieveCalStatus()
#     if (status[1]==3 and status[2]==3 and status[3]==3):
#         IMU.retrieveCalCoefficients()
#         sleep(.2)
#         print(IMU.retrieveCalStatus())
# cal_data_bytes = IMU.retrieveCalCoefficients()
# print(cal_data_bytes)
# with open('IMU_cal.txt', 'wb') as f:  # 'wb' = write binary
#     f.write(cal_data_bytes)   

# print(os.listdir()) # Can be used to check if cal file exists

# while True:
#     sleep_ms(100)
#     print(IMU.readLinearAcceleration())
    # print(IMU.readAngularVelocity())

    
   # while True:
    #     print(IMU.retrieveCalStatus())




