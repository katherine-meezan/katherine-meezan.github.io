import gc
import pyb
import cotask
import task_share
import cqueue
from time import ticks_us, ticks_ms, ticks_diff  # Use to get dt value in update()
from pyb import Timer, Pin, I2C
from Encoder import Encoder
from motor_driver import motor_driver
from time import sleep_ms
from pyb import USB_VCP
from controller import CLMotorController, IRController
from ir_sensor import IR_sensor
from sensor_array import sensor_array
from machine import UART
from IMU_I2C import IMU_I2C
import os
from ulab import numpy as np

ser = USB_VCP()

# Setup Pins for bluetooth module
# Deconfigure default pins
Pin(Pin.cpu.A2, mode=Pin.ANALOG)  # Set pin modes back to default
Pin(Pin.cpu.A3, mode=Pin.ANALOG)

# Configure the selected pins in coordination with the alternate function table
Pin(Pin.cpu.B6, mode=Pin.ALT, alt=7)  # Set pin modes to UART matching column 7 in alt. fcn. table
Pin(Pin.cpu.B7, mode=Pin.ALT, alt=7)

uart = UART(1, 115200)  # init with given baudrate
uart.init(115200, bits=8, parity=None, stop=1)  # init with given parameters
uart.write("test".encode())

# L_en = 0
# L_eff = 0
L_prev_dir = 0
L_prev_en = 1
L_prev_eff = 0
L_t_start = 0
L_t_UI = 0
R_prev_dir = 0
R_prev_en = 1
R_prev_eff = 0
R_t_start = 0
R_t_UI = 0

PA8 = Pin(Pin.cpu.A8, mode=Pin.OUT_PP)  # Timer1_Channel1: Encoder A left
PA9 = Pin(Pin.cpu.A9, mode=Pin.OUT_PP)  # Timer1_Channel2: Encoder B left
timLeft = Timer(1, prescaler=0, period=0xFFFF)
ch1Left = timLeft.channel(1, Timer.ENC_AB, pin=PA8)
ch2Left = timLeft.channel(2, Timer.ENC_AB, pin=PA9)
left_encoder = Encoder(timLeft, ch1Left, ch2Left)

PB4 = Pin(Pin.cpu.B4, mode=Pin.OUT_PP)  # Timer3_Channel1: Encoder A right
PB5 = Pin(Pin.cpu.B5, mode=Pin.OUT_PP)  # Timer3_Channel2: Encoder B right
timRight = Timer(3, prescaler=0, period=0xFFFF)
ch1Right = timRight.channel(1, Timer.ENC_AB, pin=PB4)
ch2Right = timRight.channel(2, Timer.ENC_AB, pin=PB5)
right_encoder = Encoder(timRight, ch1Right, ch2Right)

mot_left = motor_driver(Pin.cpu.B9, Pin.cpu.C9, Pin.cpu.C8, Timer(17, freq=60000), 1)
mot_right = motor_driver(Pin.cpu.A6, Pin.cpu.A1, Pin.cpu.A0, Timer(16, freq=60000), 1)

PC2 = Pin(Pin.cpu.C2, mode=Pin.ANALOG)
BAT_READ = pyb.ADC(PC2)

# CONTROLLER SETPOINT IS IN MM/S
cl_ctrl_mot_left = CLMotorController(0, 0, 0, Kp=.5, Ki=5, min_sat=-100, max_sat=100, t_init=0,
                                     v_nom=9, threshold=5, K3=.0209)
cl_ctrl_mot_right = CLMotorController(0, 0, 0, Kp=.5, Ki=5, min_sat=-100, max_sat=100, t_init=0,
                                      v_nom=9, threshold=5, K3=.0216)

ir_ch1 = IR_sensor(Pin(Pin.cpu.C3, mode=Pin.ANALOG))
ir_ch3 = IR_sensor(Pin(Pin.cpu.A4, mode=Pin.ANALOG))
ir_ch5 = IR_sensor(Pin(Pin.cpu.B0, mode=Pin.ANALOG))
ir_ch7 = IR_sensor(Pin(Pin.cpu.C1, mode=Pin.ANALOG))
ir_ch9 = IR_sensor(Pin(Pin.cpu.C0, mode=Pin.ANALOG))
ir_ch11 = IR_sensor(Pin(Pin.cpu.C4, mode=Pin.ANALOG))
ir_ch13 = IR_sensor(Pin(Pin.cpu.C5, mode=Pin.ANALOG))
channels = [ir_ch1, ir_ch3, ir_ch5, ir_ch7, ir_ch9, ir_ch11, ir_ch13]
ir_sensor_array = sensor_array(channels, 4, 8)

centroid_set_point = 0
ir_controller = IRController(centroid_set_point, 0, 0, Kp=3, Ki=1)


def IR_sensor(shares):
    global centroid_set_point
    calib_black, calib_white, line_follow, L_speed_share, R_speed_share, wheel_diff = shares
    state = 0
    while True:
        # print("IR TASK CALLED")
        if state == 0:  # wait for a flag to be set
            if calib_black.get() == 1:
                state = 1
            elif calib_white.get() == 1:
                state = 2
            elif line_follow.get() == 1:
                state = 3
        elif state == 1:
            print("Starting Black Calibration!")
            calib_start = ticks_ms()
            # ir_sensor_array.calibrate_black()
            calib_end = ticks_ms()
            calib_time = ticks_diff(calib_end, calib_start)
            print(f"Calibration complete! Time elapsed: {calib_time / 1000}")
            print(f"Black Values: {ir_sensor_array.blacks}")
            # ir_sensor_array.blacks = [3206.13, 2983.14, 3063.67, 2910.69, 2800.52, 2930.22, 3063.97]
            calib_black.put(0)
            state = 0
        elif state == 2:
            print("Starting White Calibration!")
            calib_start = ticks_ms()
            # ir_sensor_array.calibrate_white()
            calib_end = ticks_ms()
            calib_time = ticks_diff(calib_end, calib_start)
            print(f"Calibration complete! Time elapsed: {calib_time / 1000}")
            print(f"White Values: {ir_sensor_array.whites}")
            # ir_sensor_array.whites = [360.62, 299.06, 297.68, 286.17, 281.66, 295.05, 316.05]
            calib_white.put(0)
            state = 0
        elif state == 3:
            ir_sensor_array.blacks = [3206.13, 2983.14, 3063.67, 2910.69, 2800.52, 2930.22, 3063.97]
            ir_sensor_array.whites = [360.62, 299.06, 297.68, 286.17, 281.66, 295.05, 316.05]
            ir_controller.set_target(centroid_set_point)
            ir_sensor_array.array_read()
            ir_ticks_new = ticks_us()  # timestamp sensor reading for controller
            wheel_speed_diff = ir_controller.get_action(ir_ticks_new, ir_sensor_array.find_centroid())
            # split the difference in wheel speeds evenly between the two wheels
            wheel_diff.put(wheel_speed_diff)
            print(f"Centroid: {ir_sensor_array.find_centroid()}, speed_diff: {wheel_speed_diff}")
            # print(wheel_speed_diff)
        yield state


"""! BNO055 Memory addresses !"""

op_mode_addr = 0x3D  # Page 70 of the BNO055 data sheet
"""! Operation mode byte:
    Bits 4-7 reserved
    Bits 0-3 Operation Mode    
!"""

calib_stat_addr = 0x35  # Pg 67, Calibration Status Byte
calib_coeff_addr = 0x55  # Pg 50, Calibration Coefficients Starting Address
eul_head_lsb = 0x1A  # Pg 52, Heading for Euler angle Starting Address
"""!
Calibration Status Byte bit map;
3 indicates fully calibrated, 0 indicates not calibrated

Bit 0 and 1 - Mag calib status
Bit 2 and 3 - Acc
Bit 4 and 5 - Gyr
Bit 6 and 7 - Sys

!"""

acc_data_x_lsb = 0x08  # Mem location where
gyr_data_x_lsb = 0x14  # Pg 52, gyro data

"""! Data to write to BNO055 !"""
config_op_mode = 0b0000  # Page 21
IMU_op_mode = 0b1000  # Page 21, does not use mag
compass_mode = 0b1001  # Page 21
m4g_mode = 0b1010  # Page 21
ndof_fmc_off_mode = 0b1011  # Page 21
ndof = 0b1100  # Page 21
full_sensor_fusion_op_mode = 0x0C  # Uses all sensors

IMU_addr = 0x28  # BNO055 device address
# Create Pins for IMU operations

PB10 = Pin(Pin.cpu.B10, mode=Pin.ALT, alt=4)  # I2C2 SCL
PB11 = Pin(Pin.cpu.B11, mode=Pin.ALT, alt=4)  # I2C2 SDA

# Create I2C controller object
# i2c = I2C(2)
i2c = I2C(2, I2C.CONTROLLER)

IMU = IMU_I2C(i2c, IMU_addr)  # Create IMU_I2C object


def IMU_OP(shares):
    L_pos_share, R_pos_share, L_voltage_share, R_voltage_share, L_vel_share, R_vel_share, \
        yaw_angle_share, yaw_rate_share, dist_traveled_share, IMU_time_share, test_start_time = shares
    heading_offset = 0
    robot_width = 141  # mm
    wheel_radius = 35  # mm
    T_s = 0.05  # observer timestep in seconds
    # create state variables

    x_hat_old = np.array(np.zeros(4).reshape(4, ))
    x_hat_new = np.array(np.zeros(4).reshape(4, ))
    u_aug = np.array(np.zeros(6).reshape(6, ))
    y_measured = np.array(np.zeros(4).reshape(4, ))
    y_hat = np.array(np.zeros(4).reshape(4, ))
    A_d = np.array(
        [0.50,0.06,0.01,0.000000,0.06,0.50,0.01,0.000000,0.02,0.02,0.98,0.000000,0.000000,0.000000,0.000000,1.000000]).reshape((4, 4)).transpose()
    B_d = np.array([
        0.18,0.02,0.000000,0.000000,0.02,0.18,0.000000,0.000000,0.000000,0.000000,0.45,0.000000,0.000000,0.000000,0.45,0.000000,0.000000,0.000000,0.000000,0.95,0.000000,0.000000,0.000000,0.050000]).reshape((6, 4)).transpose()
    B_d[3,5] = 0
    B_d[3, 4] = 1
    C = np.array(
        [0.000000,0.000000,0.000000,-0.248227,0.000000,0.000000,0.000000,0.248227,1.000000,1.000000,0.000000,0.000000,-70.500000,70.500000,1.000000,0.000000]).reshape((4, 4)).transpose()

    state = 0  # Calibration Procedure/Load calibration values
    old_time = ticks_us()
    while True:
        if state == 0:
            cal_file = "IMU_cal.txt"
            os_files = os.listdir()
            if cal_file in os_files:
                uart.write("Calibrating")
                IMU.changeOpMode(config_op_mode)
                sleep_ms(20)
                IMU.writeCalCoefficients(cal_file)
                sleep_ms(20)
                IMU.changeOpMode(full_sensor_fusion_op_mode)
                sleep_ms(20)
                cal_status = IMU.retrieveCalStatus()
                if cal_status[1] == cal_status[2] == cal_status[3] == 3:
                    print("IMU calibrated from text file")
                    cal_data_bytes = IMU.retrieveCalCoefficients()
                    print(cal_data_bytes)
                    print(cal_status)
                    state = 1
                else:
                    print("IMU not calibrated for some reason")
                    print(cal_status)
                    state = 1
            else:

                cal_bit = False
                IMU.changeOpMode(full_sensor_fusion_op_mode)
                while not cal_bit:
                    cal_status = IMU.retrieveCalStatus()
                    # print("Calibration status: sys, gyro, acc, mag")
                    sleep_ms(100)
                    print(cal_status)
                    if (cal_status[1] == 3 and cal_status[2] == 3 and cal_status[3] == 3):
                        cal_bit = True

                print("IMU Calibrated")

                cal_data_bytes = IMU.retrieveCalCoefficients()
                print(cal_data_bytes)
                with open('IMU_cal.txt', 'wb') as f:  # 'wb' = write binary
                    f.write(cal_data_bytes)
                    state = 1
                print(os.listdir())  # Can be used to check if cal file exists
        elif state == 1:  # Initialize reference Yaw based on encoder values
            # print("State 1")
            # y vector:
            y_measured[0] = L_pos_share.get() * .153  # in encoder counts, converted to mm
            y_measured[1] = R_pos_share.get() * .153  # in encoder counts, converted to mm
            y_measured[2] = IMU.readEulerAngles()[0]  # update yaw angle (rad)
            y_measured[3] = IMU.readAngularVelocity()[2]  # update yaw rate (rad/s)

            # Psi = Sr - Sl/w (use encoder values)
            y_measured[2] = (y_measured[1] - y_measured[0]) / robot_width
            heading_offset = y_measured[2]

            # u vector
            v_left = L_voltage_share.get()  # pwm effort converted to V in ops tasks
            v_right = R_voltage_share.get()
            # Create u* = u/y vector (vl, vr, sl, sr, psi, psi_dot)
            u_aug = np.concatenate((np.array([v_left, v_right]), y_measured))
            old_time = ticks_us()
            print(L_vel_share.get())
            print(R_vel_share.get())
            x_hat_old[0] = L_vel_share.get() / 35  # converted from mm/s to radians per second
            x_hat_old[1] = R_vel_share.get() / 35  # converted from mm/s to radians per second
            x_hat_old[2] = 0  # Romi has not travelled any linear distance yet
            x_hat_old[3] = y_measured[2]  # yaw angle is already known from output vector
            state = 2
        elif state == 2:
            # print("State 2")
            curr_time = ticks_us()
            old_time = curr_time
            new_time_meas = ticks_us()
            # Run observer and update equations
            # print(f"LINE 283{x_hat_new}")
            # x_hat_new = np.dot(A_d, x_hat_old) + np.dot(B_d, u_aug)
            x_hat_new = np.dot(B_d, u_aug)
            y_hat = np.dot(C, x_hat_old)
            dist_traveled = x_hat_new[2]
            # print(f"estimator distance: {dist_traveled}")
            dist_traveled_share.put(dist_traveled)
            IMU_time_share.put(ticks_diff(new_time_meas, test_start_time.get()))
            y_measured[0] = L_pos_share.get() * .153  # in encoder counts, converted to mm
            y_measured[1] = R_pos_share.get() * .153  # in encoder counts, converted to mm
            y_measured[2] = IMU.readEulerAngles()[0]  # update yaw angle
            y_measured[3] = IMU.readAngularVelocity()[2]  # update yaw rate
            v_left = L_voltage_share.get()  # pwm converted to V in ops tasks
            v_right = R_voltage_share.get()
            u_aug = np.concatenate((np.array([v_left, v_right]), y_measured))
            yaw_angle_share.put(y_measured[2])
            # print(f"left pos share: {y_measured[0]}, right pos share: {y_measured[1]}, estimator left distance: {dist_traveled}")
            # print(f"left wheel s: {y_measured[0]}")
            # print(f"Yaw Angles from IMU: {y_measured[2]}")
            # print(f"Angular velocity: {y_measured[3]}")
            # print(f"Yaw rate: {y_measured[3]}")
            yaw_rate_share.put(y_measured[3])
            # print(f"U_AUG: {u_aug}")
            print(L_pos_share.get())
            print(R_pos_share.get())
            
            print(f"LINE 306 {x_hat_new}")
            # update set points for motor controllers
            L_vel_share.put(x_hat_new[0])
            R_vel_share.put(x_hat_new[1])
            state = 2
        yield state


"""

AUTOMATICALLY UPDATES THE DIRECTION SHARE, ONLY SET EFFORT AND ENABLE SHARES:
L_eff_share, L_en_share, R_eff_share, R_en_share


IMPORTANT: EFFORT SHARE NOW STORES A SPEED IN MM/S, SHOULD BE RETITLED
IMPORTANT: EFFORT SHARE NOW STORES A SPEED IN MM/S, SHOULD BE RETITLED
IMPORTANT: EFFORT SHARE NOW STORES A SPEED IN MM/S, SHOULD BE RETITLED
IMPORTANT: EFFORT SHARE NOW STORES A SPEED IN MM/S, SHOULD BE RETITLED
IMPORTANT: EFFORT SHARE NOW STORES A SPEED IN MM/S, SHOULD BE RETITLED
IMPORTANT: EFFORT SHARE NOW STORES A SPEED IN MM/S, SHOULD BE RETITLED
IMPORTANT: EFFORT SHARE NOW STORES A SPEED IN MM/S, SHOULD BE RETITLED
IMPORTANT: EFFORT SHARE NOW STORES A SPEED IN MM/S, SHOULD BE RETITLED
IMPORTANT: EFFORT SHARE NOW STORES A SPEED IN MM/S, SHOULD BE RETITLED
"""


def left_ops(shares):
    print("LEFT OPS")
    state = 0
    # params: L dir, L eff, L en, L pos, L vel, L time
    L_lin_spd, L_en, L_pos, L_vel, L_time, line_follower_diff, follower_on, L_voltage = shares
    global L_prev_dir, L_prev_eff, L_prev_en, L_t_start
    # State 0: init
    while True:
        gc.collect()  # Run garbage collector so that it runs regularly
        if state == 0:  # initialize shares and vars
            mot_left.enable()
            left_encoder.zero()
            left_encoder.update()
            L_t_start = ticks_us()
            L_en.put(1)
            state = 1
        elif state == 1:  # task stays in state 1 permanently
            left_encoder.update()  # update encoder
            L_t_new = ticks_us()
            # global variables are used to store the internal data/state of the task
            if L_en.get() > 0:
                # left_encoder.zero()
                mot_left.enable()
            else:
                mot_left.disable()
            left_base_target = L_lin_spd.get()
            if follower_on.get():
                follower_diff = line_follower_diff.get() / 2
                left_target = left_base_target - follower_diff
            else:
                left_target = left_base_target

            cl_ctrl_mot_left.set_target(left_target)
            pwm_percent = cl_ctrl_mot_left.get_action(L_t_new, left_encoder.get_velocity())
            mot_left.set_effort(pwm_percent)
            # print(f"left target speed: {left_target}")
            # print(f"PWM percent: {pwm_percent}")
            L_voltage.put(pwm_percent * 9 / 100)  # pwm percent sent to motor
            L_pos.put(left_encoder.get_position())  # counts
            L_vel.put(left_encoder.get_velocity())  # counts
            L_time.put(ticks_diff(L_t_new, L_t_start))
        yield state


def right_ops(shares):
    # print("RIGHT OPS")
    state = 0
    # params: R dir, R eff, R en, R pos, R vel, R time
    R_lin_spd, R_en, R_pos, R_vel, R_time, line_follower_diff, follower_on, R_voltage = shares
    global R_prev_dir, R_prev_eff, R_prev_en, R_t_start
    # State 0: init
    while True:
        # print("RIGHT OPS LOOP")
        if state == 0:
            mot_right.enable()
            right_encoder.zero()
            right_encoder.update()
            R_t_start = ticks_us()
            R_en.put(1)
            state = 1
        elif state == 1:
            right_encoder.update()
            R_t_new = ticks_us()
            if R_en.get() > 0:
                # right_encoder.zero()
                mot_right.enable()
            else:
                mot_right.disable()
                R_prev_en = R_en.get()
            right_base_target = R_lin_spd.get()
            cl_ctrl_mot_right.set_target(right_base_target)
            R_prev_eff = R_lin_spd.get()  # store and update the effort
            if follower_on.get():  # implement the speed adjustment from the line follower task
                follower_diff = line_follower_diff.get() / 2
                cl_ctrl_mot_right.set_target(right_base_target + follower_diff)
            pwm_percent = cl_ctrl_mot_right.get_action(R_t_new, right_encoder.get_velocity())  # t_print is a pwm%
            mot_right.set_effort(pwm_percent)
            R_voltage.put(pwm_percent * 9 / 100)  # voltage input to motor
            R_pos.put(right_encoder.get_position())
            R_vel.put(right_encoder.get_velocity())
            R_time.put(ticks_diff(R_t_new, R_t_start))
        yield state


test_start_time = 0

"""!
UI Task guide:
    r = increase right motor effort by 10
    e = decrease right motor effort by 10
    l = increase left motor effort by 10
    k = decrease left motor effort by 10
    c = enable/disable right motor
    n = enable/disable left motor
    p = print current motor efforts
    s = run step response test
    b = print current task state (for debugging, can remove)
    z = print left queues
    x = print right queues
    t = Spin in circle and collect euler data

Motor step response test:
    Turns both motors off
    Sets both motor efforts to what the right motor effort currently is
    Runs for ~2.5 to 3 seconds
    Turns both motors off (effort does not reset)
    Sets run share to start/stop data collection (data collection is not working)
!"""


def run_UI(shares):
    L_lin_speed, L_en, R_lin_speed, R_en, Run, Print_out, test_start_time_share = shares
    state = 0
    while True:
        if state == 0:  # init state
            # Init messenger variables
            l_lin_spd = 0
            l_en = 1
            r_lin_spd = 0
            r_en = 1
            test_start_time = 0
            test_end_time = 0
            Run.put(0)
            state = 1
            # clear out all unread inputs.
            if uart.any():
                uart.read()
        elif state == 1:
            if uart.any():  # wait for any character
                char_in = uart.read(1).decode()
                state = 2
        elif state == 2:  # decode character
            if char_in == "r":
                r_lin_spd += 2
                R_lin_speed.put(r_lin_spd)
                state = 1
            elif char_in == "e":
                r_lin_spd -= 10
                R_lin_speed.put(r_lin_spd)
                state = 1
            elif char_in == "c":
                # print(r_en)
                if r_en == 1:
                    r_en = 0
                else:
                    r_en = 1
                R_en.put(r_en)
                state = 1
            elif char_in == "l":
                l_lin_spd += 2
                L_lin_speed.put(l_lin_spd)
                state = 1
            elif char_in == "k":
                l_lin_spd -= 10
                L_lin_speed.put(l_lin_spd)
                state = 1
            elif char_in == "n":
                if l_en == 1:
                    l_en = 0
                else:
                    l_en = 1
                L_en.put(l_en)
                state = 1
            elif char_in == "p":
                uart.write("Left motor effort: ", l_lin_spd, "\nRight motor effort: ", r_lin_spd)
                state = 1
            elif char_in == "b":
                print(state)
                state = 1

            elif char_in == "t":  # Have Romi turn in place and collect angle data
                r_en = 0
                l_en = 0
                R_en.put(r_en)
                L_en.put(l_en)
                l_lin_spd = 0
                r_lin_spd = 100
                L_lin_speed.put(l_lin_spd)
                R_lin_speed.put(r_lin_spd)
                # state = 1
                Run.put(1)  # Indicates start to data collection
                test_start_time = ticks_ms()  # Record start time of test
                data_collect_comp_time = ticks_us()
                test_start_time_share.put(data_collect_comp_time)
                state = 3

            elif char_in == "z":
                # print("Z pressed! Print out: ", Print_out.get(), " Run: ", Run.get())
                if Print_out.get() != 1:
                    Print_out.put(1)
                    # print("Print_out set!")
                state = 1
            elif char_in == "s":  # Run step response test, at whatever effort the right motor was last set to
                uart.write("starting step reponse!______________")

                r_en = 0
                l_en = 0
                R_en.put(r_en)
                L_en.put(l_en)
                l_lin_spd = r_lin_spd
                L_lin_speed.put(l_lin_spd)
                Run.put(1)  # Indicates start to data collection
                test_start_time = ticks_ms()  # Record start time of test

                state = 3
            elif char_in == "i":  # run black calibration sequence for IR sensor
                calib_black.put(1)
                state = 1
                print("recieved i")
            elif char_in == "w":
                calib_white.put(1)
                state = 1
            elif char_in == "y":
                l_en = 1
                r_en = 1
                r_lin_spd = 2
                l_lin_spd = 2
                R_lin_speed.put(r_lin_spd)
                L_lin_speed.put(l_lin_spd)
                L_en.put(l_en)
                R_en.put(r_en)

                line_follow.put(1)
                state = 1

            else:
                state = 1
        elif state == 3:  # Start data collection without running the motors so the start of the step response can be observed
            if ticks_diff(ticks_ms(), test_start_time) >= 250:  # enables motors after 250 ms of data collection
                state = 4
        elif state == 4:  # Turn the motors on, record time so the data collection can be ended at the right time
            r_en = 1
            l_en = 1
            R_en.put(r_en)
            L_en.put(l_en)
            test_start_time = ticks_ms()
            state = 5
        elif state == 5:  # Stop collecting data, set flags for data collection task
            # print("I made it to state 3!")
            # print("Now:", ticks_ms(), "diff:", ticks_diff(ticks_ms(), test_start_time))
            if ticks_diff(ticks_ms(), test_start_time) >= 3000:  # stops test after ~ 2.5 seconds
                # print("state 3 exit")
                r_en = 0
                l_en = 0
                R_en.put(r_en)
                L_en.put(l_en)
                state = 1
                Run.put(0)  # indicates stop to data collection
                uart.write("STOP STEP RESPONSE______________")
        yield state


"""!
I got the below helper function from chatGPT, it does not work but it is here
!"""


# Helper function to dump all items from a queue into a list
def queue_to_list(q):
    data = []
    while q.any():
        data.append(q.get())
    return data


def collect_data(shares):
    # print("collect data")
    state = 0
    R_EFF, L_EFF, RIGHT_POS, RIGHT_VEL, R_TIME, LEFT_POS, LEFT_VEL, L_TIME, yaw_angle, yaw_rate, IMU_time_share, dist_traveled_share, run, print_out = shares
    while True:
        # print([x.get() for x in shares])
        # print("COLLECT DATA loop")
        # Initialize state
        # print("collect_data state: ", state)
        if state == 0:
            # Size of queue data being sent back to UI
            QUEUE_SIZE = 400

            # Initialize rightside queues
            RIGHT_POS_Q = cqueue.FloatQueue(QUEUE_SIZE)  # Position share is initialized as f
            RIGHT_VEL_Q = cqueue.FloatQueue(QUEUE_SIZE)  # Velocity share is initialized as f
            R_TIME_Q = cqueue.IntQueue(QUEUE_SIZE)  # Time share is initialized as I

            # Initialize leftside queues
            LEFT_POS_Q = cqueue.FloatQueue(QUEUE_SIZE)  # Position share is initialized as f
            LEFT_VEL_Q = cqueue.FloatQueue(QUEUE_SIZE)  # Velocity share is initialized as f
            L_TIME_Q = cqueue.IntQueue(QUEUE_SIZE)  # Time share is initialized as I

            # IMU Queues
            IMU_TIME_Q = cqueue.IntQueue(QUEUE_SIZE)  # Time share is initialized as I

            # X_Hat vector from IMU task
            S_Q = cqueue.FloatQueue(QUEUE_SIZE)

            # Y vector output queues from IMU task
            # S_L_Q = cqueue.FloatQueue(QUEUE_SIZE)
            # S_R_Q = cqueue.FloatQueue(QUEUE_SIZE)
            Psi_Q = cqueue.FloatQueue(QUEUE_SIZE)
            Psi_dot_Q = cqueue.FloatQueue(QUEUE_SIZE)

            print_out.put(0)
            state = 1
        # Wait state
        elif state == 1:
            # run is a shared integer, should be initialized in UI
            # run = task_share.Share('b', thread_protect=False, name="run")
            t = print_out.get()
            r = run.get()
            if r:
                # print("r: ", r)
                state = 2
            elif t:
                # print("t: ", t)
                if RIGHT_VEL_Q.any() and RIGHT_POS_Q.any() and R_TIME_Q.any() and LEFT_VEL_Q.any() and LEFT_POS_Q.any() and L_TIME_Q.any():
                    state = 3
                # else:
                # print("No data to print!")
        # Data collection state
        elif state == 2:
            if run.get():
                # Putting shares from right motor task into queues
                RIGHT_POS_Q.put(RIGHT_POS.get())
                RIGHT_VEL_Q.put(RIGHT_VEL.get())
                R_TIME_Q.put(R_TIME.get())

                # Putting shares from left motor task into queues
                LEFT_POS_Q.put(LEFT_POS.get())
                LEFT_VEL_Q.put(LEFT_VEL.get())
                L_TIME_Q.put(L_TIME.get())

                # Put IMU task shares
                # S_L_Q.put()
                # S_R_Q.put()
                temp = (IMU_time_share.get())
                # print(temp)
                # IMU_TIME_Q.put(temp)
                S_Q.put(dist_traveled_share.get())

                Psi_Q.put(yaw_angle.get())
                Psi_dot_Q.put(yaw_rate.get())

                state = 2
            else:
                state = 1
        # outputting data state
        elif state == 3:
            uart.write("OUTPUTTING TRIAL DATA _____________________________ \r\n")
            # r = queue_to_list(R_TIME_Q)
            # size = len(r)
            # print(r)
            # r = queue_to_list(RIGHT_VEL_Q)
            # print(r)

            # Step response output:

            size = 0
            # # uart.write(strR_EFF.get())
            # uart.write(f"RIGHT MOTOR: EFFORT = {R_EFF.get()}\r\n")
            # # print("TIME IN MICROSECS, VELOCITY IN ENCODER COUNTS PER MICROSEC")
            # while R_TIME_Q.any():
            #     size += 1
            #     uart.write(f"{R_TIME_Q.get()}, {RIGHT_VEL_Q.get()}\r\n")
            #     sleep_ms(5)
            # # uart.write(f"LEFT MOTOR: EFFORT =  {L_EFF.get()}\r\n")
            # sleep_ms(10)
            # uart.write(f"LEFT MOTOR\r\n")
            # while L_TIME_Q.any():
            #     uart.write(f"{L_TIME_Q.get()}, {LEFT_VEL_Q.get()}\r\n")
            #     sleep_ms(5)
            # uart.write(f"Number of data points: {size}\r\n")

            # IMU task output:

            uart.write("Euler Angles output\n")
            print("Got to the Euler angle write in collection task")
            while Psi_Q.any():
                size += 1
                uart.write(f"{IMU_TIME_Q.get()}, {Psi_Q.get()}, {Psi_dot_Q.get()}\r\n")
                sleep_ms(5)
            # uart.write(f"Yaw rate output")
            # while Psi_dot_Q.any():
            #     size += 1
            #     uart.write(f"{R_TIME_Q.get()}, {Psi_dot_Q.get()}\r\n")
            #     sleep_ms(5)
            # state = 1
            # uart.write(f"Number of data points: {size}\r\n")
            print("Done sending data")
            run.put(0)
            print_out.put(0)
            state = 1
        yield state


def battery_read(shares):
    while True:
        # print("BATTERY TASK")
        battery, low_bat_flag = shares
        battery_level = BAT_READ.read() * 3.3 / 4095  # CONVERT TO VOLTAGE READ BY THE ADC
        battery_level = battery_level * 14.7/4.7 # CONVERT TO ACTUAL BATTERY AMOUNT USING RESISTOR VALUES IN VOLTAGE DIVIDER
        battery.put(battery_level)
        cl_ctrl_mot_left.set_battery(battery_level)
        cl_ctrl_mot_right.set_battery(battery_level)
        if battery_level < cl_ctrl_mot_left.threshold:
            low_bat_flag.put(1)
        else:
            low_bat_flag.put(0)
        # print("END BATTERY TASK")
        # print(f"Battery task: {battery.get()}, {low_bat_flag.get()}")
        # gc.collect()  # Run garbage collector in battery task so that it runs regularly
        yield 0


# This code creates a share, a queue, and two tasks, then starts the tasks. The
# tasks run until somebody presses ENTER, at which time the scheduler stops and
# printouts show diagnostic information about the tasks, share, and queue.
if __name__ == "__main__":
    uart.write("Testing ME405 stuff in cotask.py and task_share.py\r\n"
               "Press Ctrl-C to stop and show diagnostics")

    # Create a share and a queue to test function and diagnostic printouts
    share0 = task_share.Share('h', thread_protect=False, name="Share 0")
    q0 = task_share.Queue('L', 16, thread_protect=False, overwrite=False,
                          name="Queue 0")

    # Create Share objects for inter-task communication
    L_lin_spd = task_share.Share('f', thread_protect=False, name="L lin spd")
    L_voltage_share = task_share.Share('f', thread_protect=False, name="L mot eff")
    L_en_share = task_share.Share('H', thread_protect=False, name="L en")
    L_pos_share = task_share.Share('f', thread_protect=False, name="L pos")
    L_vel_share = task_share.Share('f', thread_protect=False, name="L vel")
    L_time_share = task_share.Share('I', thread_protect=False, name="L time")
    R_dir_share = task_share.Share('H', thread_protect=False, name="R dir")
    R_lin_spd = task_share.Share('f', thread_protect=False, name="R lin spd")
    R_voltage_share = task_share.Share('f', thread_protect=False, name="R mot eff")
    R_en_share = task_share.Share('H', thread_protect=False, name="R en")
    R_pos_share = task_share.Share('f', thread_protect=False, name="R pos")
    R_vel_share = task_share.Share('f', thread_protect=False, name="R vel")
    R_time_share = task_share.Share('I', thread_protect=False, name="R time")
    run = task_share.Share('H', thread_protect=False, name="run")
    print_out = task_share.Share('H', thread_protect=False, name="print out")
    bat_share = task_share.Share('f', thread_protect=False, name="print out")
    bat_flag = task_share.Share('H', thread_protect=False, name="print out")
    calib_black = task_share.Share('H', thread_protect=False, name="print out")
    calib_white = task_share.Share('H', thread_protect=False, name="print out")
    line_follow = task_share.Share('H', thread_protect=False, name="print out")
    wheel_diff = task_share.Share('f', thread_protect=False, name="wheel speed diff")
    yaw_angle_share = task_share.Share('f', thread_protect=False, name="yaw angle")
    yaw_rate_share = task_share.Share('f', thread_protect=False, name="yaw rate")
    dist_traveled_share = task_share.Share('f', thread_protect=False, name="Distance traveled")
    IMU_time_share = task_share.Share('I', thread_protect=False, name="IMU time")
    time_start_share = task_share.Share('I', thread_protect=False, name="time start")

    # R_pos_queue = task_share.Queue('f', 100, name="R pos")
    # R_vel_queue = task_share.Queue('f', 100, name="R vel")
    # R_time_queue = task_share.Queue('I', 100, name="R time")
    # L_pos_queue = task_share.Queue('f', 100, name="L pos")
    # L_vel_queue = task_share.Queue('f', 100, name="L vel")
    # L_time_queue = task_share.Queue('I', 100, name="L time")
    # data_share = task_share.Share('i', name="data_share")

    # Create the tasks. If trace is enabled for any task, memory will be
    # allocated for state transition tracing, and the application will run out
    # of memory after a while and quit. Therefore, use tracing only for
    # debugging and set trace to False when it's not needed2

    task_left_ops = cotask.Task(left_ops, name="Left ops", priority=3, period=20,
                                profile=True, trace=True, shares=(L_lin_spd, L_en_share, L_pos_share, L_vel_share,
                                                                  L_time_share, wheel_diff, line_follow,
                                                                  L_voltage_share))
    task_right_ops = cotask.Task(right_ops, name="Right ops", priority=4, period=20,
                                 profile=True, trace=True, shares=(R_lin_spd, R_en_share, R_pos_share, R_vel_share,
                                                                   R_time_share, wheel_diff, line_follow,
                                                                   R_voltage_share))
    # task_dumb_ui = cotask.Task(dumb_ui, name="Dumb UI", priority=1, period=10,
    #                             profile=True, trace=True, shares=(L_eff_share, R_eff_share))

    task_ui = cotask.Task(run_UI, name="UI", priority=1, period=100,
                          profile=True, trace=True,
                          shares=(L_lin_spd, L_en_share, R_lin_spd, R_en_share, run, print_out, time_start_share))

    task_collect_data = cotask.Task(collect_data, name="Collect Data", priority=0, period=20,
                                    profile=True, trace=True, shares=(
            R_lin_spd, L_lin_spd, R_pos_share, R_vel_share, R_time_share, L_pos_share, L_vel_share, L_time_share,
            yaw_angle_share, yaw_rate_share, IMU_time_share, dist_traveled_share, run,
            print_out))

    task_read_battery = cotask.Task(battery_read, name="Battery", priority=0, period=2000,
                                    profile=True, trace=True, shares=(bat_share, bat_flag))
    task_IR_sensor = cotask.Task(IR_sensor, name="IR sensor", priority=0, period=50,
                                 profile=True, trace=True,
                                 shares=(calib_black, calib_white, line_follow, L_lin_spd, R_lin_spd, wheel_diff))
    task_state_estimator = cotask.Task(IMU_OP, name="state estimator", priority=10, period=50,
                                       profile=True, trace=True, shares=(
            L_pos_share, R_pos_share, L_voltage_share, R_voltage_share, L_vel_share, R_vel_share, yaw_angle_share,
            yaw_rate_share, dist_traveled_share, IMU_time_share, time_start_share))

    # cotask.task_list.append(task1)
    # cotask.task_list.append(task2)

    # Add tasks to task list to run in scheduler
    cotask.task_list.append(task_left_ops)
    cotask.task_list.append(task_right_ops)  # only testing with the left motor
    # cotask.task_list.append(task_dumb_ui)
    cotask.task_list.append(task_ui)
    cotask.task_list.append(task_collect_data)
    cotask.task_list.append(task_read_battery)
    # cotask.task_list.append(task_IR_sensor)
    cotask.task_list.append(task_state_estimator)

    # Run the memory garbage collector to ensure memory is as defragmented as
    # possible before the real-time scheduler is started
    gc.collect()

    print("PROG START")

    # Run the scheduler with the chosen scheduling algorithm. Quit if ^C pressed
    while True:
        try:
            cotask.task_list.pri_sched()
        except KeyboardInterrupt:
            print('\n' + str(cotask.task_list))
            print(task_share.show_all())
            print('')
            break
    # Print a table of task data and a table of shared information data
    print('\n' + str(cotask.task_list))
    print(task_share.show_all())
    print('')



