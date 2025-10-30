"""!
@file basic_tasks.py
    This file contains a demonstration program that runs some tasks, an
    inter-task shared variable, and a queue. The tasks don't really @b do
    anything; the example just shows how these elements are created and run.

@author JR Ridgely
@date   2021-Dec-15 JRR Created from the remains of previous example
@copyright (c) 2015-2021 by JR Ridgely and released under the GNU
    Public License, Version 2. 
"""

import gc
import pyb
import cotask
import task_share
from time import ticks_us, ticks_diff  # Use to get dt value in update()
from pyb import Timer, Pin
from Encoder import Encoder
from motor_driver import motor_driver
from time import sleep_ms

# L_en = 0
# L_eff = 0
L_prev_dir = 0
L_prev_en = 0
L_prev_eff = 0
L_t_start = 0
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

def task1_fun(shares):
    """!
    Task which puts things into a share and a queue.
    @param shares A list holding the share and queue used by this task
    """
    # Get references to the share and queue which have been passed to this task
    my_share, my_queue = shares

    counter = 0
    while True:
        my_share.put(counter)
        my_queue.put(counter)
        counter += 1

        yield 0


def task2_fun(shares):
    """!
    Task which takes things out of a queue and share and displays them.
    @param shares A tuple of a share and queue from which this task gets data
    """
    # Get references to the share and queue which have been passed to this task
    the_share, the_queue = shares

    while True:
        # Show everything currently in the queue and the value in the share
        print(f"Share: {the_share.get ()}, Queue: ", end='')
        while q0.any():
            print(f"{the_queue.get ()} ", end='')
        print('')

        yield 0

def left_ops(shares):
    # print("LEFT OPS")
    state = 0
    # params: L dir, L eff, L en, L pos, L vel, L time
    L_dir, L_eff, L_en, L_pos, L_vel, L_time = shares
    global L_prev_dir, L_prev_eff, L_prev_en, L_t_start
    # State 0: init
    while True:
        # print("LEFT OPS LOOP")
        if state == 0:
            mot_left.enable()
            left_encoder.zero()
            L_t_start = ticks_us()
            L_en.put(0)
            L_eff.put(0)
            L_dir.put(0)
            state = 1
            yield 1
        elif state == 1:
            left_encoder.update()
            L_t_new = ticks_us()
            if L_en.get() != L_prev_en:
                print("LO1")
                if L_en.get() > 0:
                    mot_left.enable()
                else:
                    mot_left.disable()
            if L_eff.get() != L_prev_eff:
                print("LO2")
                L_prev_dir = L_dir.get()
                if L_eff.get() > 0:
                    L_dir.put(1)
                else:
                    L_dir.put(0)
                L_prev_eff = L_eff.get()
                mot_left.set_effort(L_eff.get())
            if not L_pos.full() and not L_vel.full() and not L_time.full():
                L_pos.put(left_encoder.get_position())
                L_vel.put(left_encoder.get_velocity())
                L_time.put(ticks_diff(L_t_new, L_t_start))
            print(L_pos.get(), L_vel.get(), L_time.get())
            yield 1

def dumb_ui(shares):
    print("DUMB UI")
    global R_t_UI
    L_t_UI = ticks_us()
    L_eff = shares
    L_eff.put(-10)
    yield 0
    while True:
        c_time = ticks_us()
        diff = ticks_diff(c_time, L_t_UI)
        if diff > 100000:
            if L_eff.get() == 10:
                L_eff.put(0)
                print("eff: 0")
            else:
                L_eff.put(10)
                print("eff: -10")
            L_t_UI = c_time
        yield 0



# This code creates a share, a queue, and two tasks, then starts the tasks. The
# tasks run until somebody presses ENTER, at which time the scheduler stops and
# printouts show diagnostic information about the tasks, share, and queue.
if __name__ == "__main__":
    print("Testing ME405 stuff in cotask.py and task_share.py\r\n"
          "Press Ctrl-C to stop and show diagnostics.")

    # Create a share and a queue to test function and diagnostic printouts
    share0 = task_share.Share('h', thread_protect=False, name="Share 0")
    q0 = task_share.Queue('L', 16, thread_protect=False, overwrite=False,
                          name="Queue 0")
    L_dir_share = task_share.Share('H', thread_protect=False, name="L dir")
    L_eff_share = task_share.Share('f', thread_protect=False, name="L eff")
    L_en_share = task_share.Share('H', thread_protect=False, name="L en")
    L_pos_queue = task_share.Queue('f', 100, name="L pos")
    L_vel_queue = task_share.Queue('f', 100, name="L vel")
    L_time_queue = task_share.Queue('I', 100, name="L time")

    # Create the tasks. If trace is enabled for any task, memory will be
    # allocated for state transition tracing, and the application will run out
    # of memory after a while and quit. Therefore, use tracing only for
    # debugging and set trace to False when it's not needed2
    # task1 = cotask.Task(task1_fun, name="Task_1", priority=1, period=400,
    #                     profile=True, trace=False, shares=(share0, q0))
    # task2 = cotask.Task(task2_fun, name="Task_2", priority=2, period=1500,
    #                     profile=True, trace=False, shares=(share0, q0))
    task_left_ops = cotask.Task(left_ops, name="Left ops", priority=100, period=100,
                        profile=False, trace=False, shares=(L_dir_share,
                                L_eff_share, L_en_share, L_pos_queue, L_vel_queue, L_time_queue))
    task_dumb_ui = cotask.Task(dumb_ui, name="Dumb UI", priority=1, period=1,
                                profile=False, trace=False, shares=(L_eff_share))
    # cotask.task_list.append(task1)
    # cotask.task_list.append(task2)
    cotask.task_list.append(task_left_ops)
    cotask.task_list.append(task_dumb_ui)
    # Run the memory garbage collector to ensure memory is as defragmented as
    # possible before the real-time scheduler is started
    gc.collect()

    # Run the scheduler with the chosen scheduling algorithm. Quit if ^C pressed
    while True:
        try:
            cotask.task_list.pri_sched()
        except KeyboardInterrupt:
            break

    # Print a table of task data and a table of shared information data
    print('\n' + str (cotask.task_list))
    print(task_share.show_all())
    print(task1.get_trace())
    print('')
