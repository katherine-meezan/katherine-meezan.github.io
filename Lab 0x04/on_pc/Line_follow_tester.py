from serial import Serial
from time import sleep
import pandas as pd
import matplotlib.pyplot as plt
# from matplotlib import pyplot
import csv

times = []
data = []

# ComPort = "/dev/tty.usbmodem2058397458562"   # For MacOS, will need to change if board is reflashed
ComPort = "COM5" #COM13, COM5 for Katherine


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
    ser.write(b"\x03")  # Ctrl-C
    sleep(0.5)

    print("Sending Ctrl-D to soft reboot and run main.py...")
    ser.write(b"\x04")  # Ctrl-D
    sleep(1)

    print("Flushing serial port")
    while ser.in_waiting:
        ser.read()
        

    print("Starting Black Calibration")
    
    ser.write(b"i\r\n")
    sleep(2)
    print("Move to White")
    sleep(4)
    print("Starting White calibration")
    ser.write(b"w\r\n")
    sleep(2)
    print("Calibration Done. Move to line")
    sleep(4)
    ser.write(b"y\r\n")

    while not ser.in_waiting: continue



