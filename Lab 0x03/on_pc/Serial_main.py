from serial import Serial
from time import sleep
import pandas as pd
import matplotlib.pyplot as plt
# from matplotlib import pyplot
import csv

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
    ser.write(b"\x03")  # Ctrl-C
    sleep(0.5)

    print("Sending Ctrl-D to soft reboot and run main.py...")
    ser.write(b"\x04")  # Ctrl-D
    sleep(1)

    print("Flushing serial port")
    while ser.in_waiting:
        ser.read()

    print("Sending command to start data collection")


    ser.write(b"r\r\n")
    sleep(.5)
    ser.write(b"s\r\n")
    sleep(5)
    ser.write(b"z\r\n")
    sleep(2)

    print("Flushing serial port")
    while ser.in_waiting:
        ser.read()
    ser.write(b"r\r\n")
    ser.write(b"s\r\n")
    sleep(5)
    ser.write(b"z\r\n")
    sleep(1)


    while not ser.in_waiting: continue

    # while True:
    #         line = ser.readline().decode(errors="replace").strip()
    #         if not line:
    #             continue
    #         if "MOTOR:" in line or "Number of data points:" in line

    right_motor_data = []
    left_motor_data = []

    read_right = False
    read_left = False
    while True:
        line = ser.readline().decode(errors="replace").strip()
        # if not line:
        #     continue
        if line.startswith("RIGHT MOTOR"):
            print("Reading right motor data...")
            read_right = True
            continue
        if read_right:
            if line.startswith("LEFT"):
                print("Reading left motor data...")
                read_right = False
                read_left = True
            else:
                right_motor_data.append(line)
        elif read_left:
            if line:
                left_motor_data.append(line)
            else:
                print("Data collection complete")
                read_left = True
                break


with open("right_motor.csv", 'w') as file:
    file.seek(0)
    file.truncate()
    save_csv("right_motor.csv", right_motor_data)
with open("left_motor.csv", 'w') as file:
    file.seek(0)
    file.truncate()
    save_csv("left_motor.csv", left_motor_data)

data_right = pd.read_csv("right_motor.csv", header=None, names=["time", "velocity"])
data_left = pd.read_csv("left_motor.csv", header=None, names=["time", "velocity"])

plt.figure()
plt.plot(data_left["time"], data_left["velocity"])

plt.title("Left Velocity vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (units/s)")

# Plot
plt.figure()
plt.plot(data_right["time"], data_right["velocity"])
plt.title("Right Velocity vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (units/s)")

plt.show()
