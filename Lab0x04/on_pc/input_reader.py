from serial import Serial
from time import sleep
import pandas as pd
import matplotlib.pyplot as plt
# from matplotlib import pyplot
import csv
from inputimeout import inputimeout, TimeoutOccurred

times = []
data = []

# ComPort = "/dev/tty.usbmodem2058397458562"   # For MacOS, will need to change if board is reflashed
ComPort = "COM13"


# If ComPort is different on Windows, add here, and then comment/uncomment which one to use


def read_data(ser):
    data = []
    while True:
        line = ser.readline().decode(errors="replace").strip()
        if "MOTOR:" in line or "Number of data points:" in line or not line:  # or "MICROSEC" in line:
            return data, line
        data.append(line)



def save_csv(filename, data_lines):
    if not data_lines:
        print(f"No data for {filename}")
        return
    header = data_lines[0].split(",")
    rows = [line.split(",") for line in data_lines[1:] if "," in line]
    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([h.strip() for h in header])
        for row in rows:
            writer.writerow([x.strip() for x in row])
    print(f"Saved {len(rows)} rows to {filename}")


with Serial(ComPort, baudrate=115_200, timeout=1) as ser:
    print("Opening serial port")
    sleep(0.5)

    print("Sending Ctrl-C to break into REPL...")
    ser.write(b'\x03')  # Ctrl-C
    sleep(0.5)

    print("Sending Ctrl-D to soft reboot and run main.py...")
    ser.write(b'\x04')  # Ctrl-D
    sleep(1)

    print("Flushing serial port")
    while ser.in_waiting:
        ser.read()

    print("""Type Commands:\n\r 
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
    x = print right queues""")
    print("Issue a command! \n")
    valid_inputs = "relkcnpsbzx"
    while True:
        key = input()
        if key in valid_inputs and len(key) == 1:
            ser.write(key.encode())
            print("valid command")
        else:
            print("invalid command")


