import gc
import pyb
import cotask
import task_share
import cqueue
from time import ticks_us, ticks_ms, ticks_diff  # Use to get dt value in update()
from pyb import Timer, Pin
from Encoder import Encoder
from motor_driver import motor_driver
from time import sleep_ms
from pyb import USB_VCP
from controller import CLMotorController, IRController
from ir_sensor import IR_sensor
from sensor_array import sensor_array
from machine import UART

ser = USB_VCP()

# Setup Pins for bluetooth module
# Deconfigure default pins
Pin(Pin.cpu.A2,  mode=Pin.ANALOG)   # Set pin modes back to default
Pin(Pin.cpu.A3,  mode=Pin.ANALOG)

# Configure the selected pins in coordination with the alternate function table
Pin(Pin.cpu.B6,  mode=Pin.ALT, alt=7) # Set pin modes to UART matching column 7 in alt. fcn. table
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
timLeft = Timer(1, freq=100)
ch1Left = timLeft.channel(1, Timer.ENC_AB, pin=PA8)
ch2Left = timLeft.channel(2, Timer.ENC_AB, pin=PA9)
left_encoder = Encoder(timLeft, ch1Left, ch2Left)

PB4 = Pin(Pin.cpu.B4, mode=Pin.OUT_PP)  # Timer3_Channel1: Encoder A right
PB5 = Pin(Pin.cpu.B5, mode=Pin.OUT_PP)  # Timer3_Channel2: Encoder B right
timRight = Timer(3, freq=100)
ch1Right = timRight.channel(1, Timer.ENC_AB, pin=PB4)
ch2Right = timRight.channel(2, Timer.ENC_AB, pin=PB5)
right_encoder = Encoder(timRight, ch1Right, ch2Right)

mot_left = motor_driver(Pin.cpu.B9, Pin.cpu.C9, Pin.cpu.C8, Timer(17, freq=60000), 1)
mot_right = motor_driver(Pin.cpu.A6, Pin.cpu.A1, Pin.cpu.A0, Timer(16, freq=60000), 1)

PC2 = Pin(Pin.cpu.C2, mode=Pin.ANALOG)
BAT_READ = pyb.ADC(PC2)


#CONTROLLER SETPOINT IS IN MM/S
cl_ctrl_mot_left = CLMotorController(0, 0, 0, Kp=1, Ki=5, min_sat=-100, max_sat=100, t_init=0,
                 v_nom=2.878, threshold=1.439, K3=1.382)
cl_ctrl_mot_right = CLMotorController(0, 0, 0, Kp=1, Ki=5, min_sat=-100, max_sat=100, t_init=0,
                 v_nom=2.878, threshold=1.439, K3=1.4272)

ir_ch1 = IR_sensor(Pin(Pin.cpu.C3, mode=Pin.ANALOG))
ir_ch3 = IR_sensor(Pin(Pin.cpu.A4, mode=Pin.ANALOG))
ir_ch5 = IR_sensor(Pin(Pin.cpu.B0, mode=Pin.ANALOG))
ir_ch7 = IR_sensor(Pin(Pin.cpu.C1, mode=Pin.ANALOG))
ir_ch9 = IR_sensor(Pin(Pin.cpu.C0, mode=Pin.ANALOG))
ir_ch11 = IR_sensor(Pin(Pin.cpu.C4, mode=Pin.ANALOG))
ir_ch13= IR_sensor(Pin(Pin.cpu.C5, mode=Pin.ANALOG))
channels = [ir_ch1, ir_ch3, ir_ch5, ir_ch7, ir_ch9, ir_ch11, ir_ch13]
ir_sensor_array = sensor_array(channels, 4, 8)

centroid_set_point = 0
ir_controller = IRController(centroid_set_point, 0, 0, Kp=1, Ki=0)

def IR_sensor(shares):
    global centroid_set_point
    calib_black, calib_white, line_follow, L_speed_share, R_speed_share, wheel_diff = shares
    state = 0
    while True:
        # print("IR TASK CALLED")
        if state == 0: # wait for a flag to be set
            if calib_black.get() == 1:
                state = 1
            elif calib_white.get() == 1:
                state = 2
            elif line_follow.get() == 1:
                state = 3
        elif state == 1:
            print("Starting Black Calibration!")
            calib_start = ticks_ms()
            ir_sensor_array.calibrate_black()
            calib_end = ticks_ms()
            calib_time = ticks_diff(calib_end, calib_start)
            print(f"Calibration complete! Time elapsed: {calib_time/1000}")
            print(f"Black Values: {ir_sensor_array.blacks}")
            calib_black.put(0)
            state = 0
        elif state == 2:
            print("Starting White Calibration!")
            calib_start = ticks_ms()
            ir_sensor_array.calibrate_white()
            calib_end = ticks_ms()
            calib_time = ticks_diff(calib_end, calib_start)
            print(f"Calibration complete! Time elapsed: {calib_time/1000}")
            print(f"White Values: {ir_sensor_array.whites}")
            calib_white.put(0)
            state = 0
        elif state == 3:
            ir_controller.set_target(centroid_set_point)
            ir_sensor_array.array_read()
            ir_ticks_new = ticks_us() # timestamp sensor reading for controller
            wheel_speed_diff = ir_controller.get_action(ir_ticks_new, ir_sensor_array.find_centroid())
            # split the difference in wheel speeds evenly between the two wheels
            L_speed = L_speed_share.get()
            R_speed = R_speed_share.get()
            L_speed += wheel_speed_diff/2
            R_speed += wheel_speed_diff/2
            L_speed_share.put(L_speed)
            R_speed_share.put(R_speed)
            print(f"Centroid: {ir_sensor_array.find_centroid()} R_speed: {R_speed} L_speed: {L_speed}")
            # print(wheel_speed_diff)
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
    L_dir, L_eff, L_en, L_pos, L_vel, L_time = shares
    global L_prev_dir, L_prev_eff, L_prev_en, L_t_start
    # State 0: init
    while True:
        if state == 0: # initialize shares and vars
            mot_left.enable()
            left_encoder.zero()
            left_encoder.update()
            L_t_start = ticks_us()
            L_en.put(1)
            L_dir.put(0)
            state = 1
        elif state == 1: # task stays in state 1 permanently
            left_encoder.update() # update encoder
            L_t_new = ticks_us()
            if L_en.get() != L_prev_en: # check if the enable signal has changed using the global variables
                # global variables are used to store the internal data/state of the task
                if L_en.get() > 0:
                    left_encoder.zero()
                    mot_left.enable()
                    L_prev_en = L_en.get() # update internal state variable
                else:
                    mot_left.disable()
                    L_prev_en = L_en.get()
            if L_eff.get() != L_prev_eff:
                L_prev_dir = L_dir.get() # update the direction to match the effort input
                if L_eff.get() > 0:
                    L_dir.put(1)
                else:
                    L_dir.put(0)
                cl_ctrl_mot_left.set_target(L_eff.get())
                L_prev_eff = L_eff.get() # store and update the effort
            # print("LINE 113")
            t_print = cl_ctrl_mot_left.get_action(L_t_new, left_encoder.get_velocity())
            # print(f"ticks: {L_t_new}, vel: {left_encoder.get_velocity()}, eff: {t_print}")
            # print("Line 115")
            # print(t_print)
            mot_left.set_effort(t_print)
            L_pos.put(left_encoder.get_position())
            L_vel.put(left_encoder.get_velocity())
            L_time.put(ticks_diff(L_t_new, L_t_start))
            # print(L_pos.get(), L_vel.get(), L_time.get())
        yield 1

def right_ops(shares):
    # print("RIGHT OPS")
    state = 0
    # params: R dir, R eff, R en, R pos, R vel, R time
    R_dir, R_eff, R_en, R_pos, R_vel, R_time = shares
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
            R_dir.put(0)
            state = 1
        elif state == 1:
            right_encoder.update()
            R_t_new = ticks_us()
            if R_en.get() != R_prev_en:
                if R_en.get() > 0:
                    right_encoder.zero()
                    mot_right.enable()
                    R_prev_en = R_en.get()

                else:
                    mot_right.disable()
                    R_prev_en = R_en.get()
            if R_eff.get() != R_prev_eff:
                R_prev_dir = R_dir.get()  # update the direction to match the effort input
                if R_eff.get() > 0:
                    R_dir.put(1)
                else:
                    R_dir.put(0)
                cl_ctrl_mot_right.set_target(R_eff.get())
                R_prev_eff = R_eff.get()  # store and update the effort
            mot_right.set_effort(cl_ctrl_mot_right.get_action(R_t_new, right_encoder.get_velocity()))
            R_pos.put(right_encoder.get_position())
            R_vel.put(right_encoder.get_velocity())
            R_time.put(ticks_diff(R_t_new, R_t_start))
            # print(R_pos.get(), R_vel.get(), R_time.get())
        yield 1

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

Motor step response test:
    Turns both motors off
    Sets both motor efforts to what the right motor effort currently is
    Runs for ~2.5 to 3 seconds
    Turns both motors off (effort does not reset)
    Sets run share to start/stop data collection (data collection is not working)
!"""


def run_UI(shares):
    L_eff, L_en, R_eff, R_en, Run, Print_out = shares
    global state, l_dir, l_eff, l_en, r_dir, r_eff, r_en, test_start_time
    state = 0
    while True:
        if state == 0: # init state
            # Init messenger variables
            l_eff = 0
            l_en = 1
            r_eff = 0
            r_en = 1
            test_start_time = 0
            test_end_time = 0
            Run.put(0)
            state = 1
            # clear out all unread inputs.
            if uart.any():
                uart.read()
        elif state == 1:
            if uart.any(): # wait for any character
                char_in = uart.read(1).decode()
                state = 2
        elif state == 2: # decode character
            if char_in == "r":
                r_eff += 10
                R_eff.put(r_eff)
                state = 1
            elif char_in == "e":
                r_eff -= 10
                R_eff.put(r_eff)
                state = 1
            elif char_in == "c":
                # print(r_en)
                if r_en==1:
                    r_en = 0
                else:
                    r_en = 1
                R_en.put(r_en)
                state = 1
            elif char_in == "l":
                l_eff += 10
                L_eff.put(l_eff)
                state = 1
            elif char_in == "k":
                l_eff -= 10
                L_eff.put(l_eff)
                state = 1
            elif char_in == "n":
                if l_en==1:
                    l_en = 0
                else:
                    l_en = 1
                L_en.put(l_en)
                state = 1
            elif char_in == "p":
                uart.write("Left motor effort: " , l_eff, "\nRight motor effort: ", r_eff)
                state = 1
            elif char_in =="b":
                print(state)
                state = 1
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
                l_eff = r_eff
                L_eff.put(l_eff)
                Run.put(1) # Indicates start to data collection
                test_start_time = ticks_ms() # Record start time of test
                state = 3
            elif char_in == "i": # run black calibration sequence for IR sensor
               calib_black.put(1)
               state = 1
               print("recieved i")
            elif char_in == "w":
               calib_white.put(1)
               state = 1
            elif char_in == "y":
                l_en = 1
                r_en = 1
                r_eff = 2
                l_eff = 2
                R_eff.put(r_eff)
                L_eff.put(l_eff)
                L_en.put(l_en)
                R_en.put(r_en)
                
                line_follow.put(1)
                state = 1
            
            else:
                state = 1
        elif state == 3: # Start data collection without running the motors so the start of the step response can be observed
            if ticks_diff(ticks_ms(), test_start_time) >= 250:  # enables motors after 250 ms of data collection
                state = 4
        elif state == 4: # Turn the motors on, record time so the data collection can be ended at the right time
            r_en = 1
            l_en = 1
            R_en.put(r_en)
            L_en.put(l_en)
            test_start_time = ticks_ms()
            state = 5
        elif state == 5: # Stop collecting data, set flags for data collection task
            # print("I made it to state 3!")
            # print("Now:", ticks_ms(), "diff:", ticks_diff(ticks_ms(), test_start_time))
            if ticks_diff(ticks_ms(), test_start_time) >= 2500: # stops test after ~ 2.5 seconds
                # print("state 3 exit")
                r_en = 0
                l_en = 0
                R_en.put(r_en)
                L_en.put(l_en)
                state = 1
                Run.put(0) # indicates stop to data collection
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

"""!
Most recent error message (Katherine):
    Traceback (most recent call last):
  File "main.py", line 460, in <module>
  File "cotask.py", line 154, in schedule
  File "main.py", line 378, in collect_data
TypeError: can't convert dict to int

What it references as 378 is now 399 because I've added comments'

!"""

def collect_data(shares):
    # print("collect data")
    state = 0
    R_EFF, L_EFF, RIGHT_POS, RIGHT_VEL, R_TIME, LEFT_POS, LEFT_VEL, L_TIME, run, print_out = shares
    while True:
        # print([x.get() for x in shares])
        # print("COLLECT DATA loop")
        # Initialize state
        # print("collect_data state: ", state)
        if state == 0:
            # Size of queue data being sent back to UI
            QUEUE_SIZE = 400

            # Initialize rightside queues
            RIGHT_POS_Q = cqueue.FloatQueue(QUEUE_SIZE) # Position share is initialized as f
            RIGHT_VEL_Q = cqueue.FloatQueue(QUEUE_SIZE) # Velocity share is initialized as f
            R_TIME_Q = cqueue.IntQueue(QUEUE_SIZE)      # Time share is initialized as I

            # Initialize leftside queues
            LEFT_POS_Q = cqueue.FloatQueue(QUEUE_SIZE)   # Position share is initialized as f
            LEFT_VEL_Q = cqueue.FloatQueue(QUEUE_SIZE)   # Velocity share is initialized as f
            L_TIME_Q = cqueue.IntQueue(QUEUE_SIZE)     # Time share is initialized as I
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
            size = 0
            # uart.write(strR_EFF.get())
            uart.write(f"RIGHT MOTOR: EFFORT = {R_EFF.get()}\r\n")
            # print("TIME IN MICROSECS, VELOCITY IN ENCODER COUNTS PER MICROSEC")
            while R_TIME_Q.any():
                size += 1
                uart.write(f"{R_TIME_Q.get()}, {RIGHT_VEL_Q.get()}\r\n")
                sleep_ms(5)
            # uart.write(f"LEFT MOTOR: EFFORT =  {L_EFF.get()}\r\n")
            sleep_ms(10)
            uart.write(f"LEFT MOTOR\r\n")
            while L_TIME_Q.any():
                uart.write(f"{L_TIME_Q.get()}, {LEFT_VEL_Q.get()}\r\n")
                sleep_ms(5)
            uart.write(f"Number of data points: {size}\r\n")
            state = 1
        yield state

def battery_read(shares):
    while True:
        # print("BATTERY TASK")
        battery, low_bat_flag = shares
        battery_level = BAT_READ.read()*3.3/4095 # CONVERT TO VOLTAGE
        battery.put(battery_level)
        cl_ctrl_mot_left.set_battery(battery_level)
        cl_ctrl_mot_right.set_battery(battery_level)
        if battery_level<cl_ctrl_mot_left.threshold:
            low_bat_flag.put(1)
        else:
            low_bat_flag.put(0)
        # print("END BATTERY TASK")
        # print(f"Battery task: {battery.get()}, {low_bat_flag.get()}")
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
    L_dir_share = task_share.Share('H', thread_protect=False, name="L dir")
    L_eff_share = task_share.Share('f', thread_protect=False, name="L eff")
    L_en_share = task_share.Share('H', thread_protect=False, name="L en")
    L_pos_share = task_share.Share('f', thread_protect=False, name="L pos")
    L_vel_share = task_share.Share('f', thread_protect=False,name="L vel")
    L_time_share = task_share.Share('I', thread_protect=False, name="L time")
    R_dir_share = task_share.Share('H', thread_protect=False, name="R dir")
    R_eff_share = task_share.Share('f', thread_protect=False, name="R eff")
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

    task_left_ops = cotask.Task(left_ops, name="Left ops", priority=3, period=50,
                                profile=True, trace=True, shares=(L_dir_share,
                                                                    L_eff_share, L_en_share, L_pos_share, L_vel_share, L_time_share))
    task_right_ops = cotask.Task(right_ops, name="Right ops", priority=4, period=50,
                                 profile=True, trace=True, shares=(R_dir_share,
                                                                     R_eff_share, R_en_share, R_pos_share, R_vel_share, R_time_share))
    # task_dumb_ui = cotask.Task(dumb_ui, name="Dumb UI", priority=1, period=10,
    #                             profile=True, trace=True, shares=(L_eff_share, R_eff_share))

    task_ui = cotask.Task(run_UI, name="UI", priority=0, period=60,
                          profile=True, trace=True, shares=(L_eff_share, L_en_share, R_eff_share, R_en_share, run, print_out))

    task_collect_data = cotask.Task(collect_data, name="Collect Data", priority=2, period=50,
                                profile=True, trace=True, shares=(R_eff_share, L_eff_share, R_pos_share, R_vel_share, R_time_share, L_pos_share, L_vel_share, L_time_share, run, print_out))

    task_read_battery = cotask.Task(battery_read, name="Battery", priority=0, period=1000,
                                profile=True, trace=True, shares=(bat_share, bat_flag))
    task_IR_sensor = cotask.Task(IR_sensor, name="IR sensor", priority=5, period=50,
                                    profile=True, trace=True, shares=(calib_black, calib_white, line_follow, L_eff_share, R_eff_share))

    # cotask.task_list.append(task1)
    # cotask.task_list.append(task2)

    # Add tasks to task list to run in scheduler
    cotask.task_list.append(task_left_ops)
    cotask.task_list.append(task_right_ops) # only testing with the left motor
    # cotask.task_list.append(task_dumb_ui)
    cotask.task_list.append(task_ui)
    cotask.task_list.append(task_collect_data)
    cotask.task_list.append(task_read_battery)
    cotask.task_list.append(task_IR_sensor)

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
    print('\n' + str (cotask.task_list))
    print(task_share.show_all())
    print('')


