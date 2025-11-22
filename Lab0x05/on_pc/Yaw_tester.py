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
        if "Euler" in line or "Yaw" in line or "Number of data points:" in line or not line:  # or "MICROSEC" in line:
            return data, line
        data.append(line)
        if not line:
            return

def read_IMU_data(ser):
    data = []
    while True:
        line = ser.readline().decode(errors="replace").strip()
        print(f"line: {line}")
        if "Euler" in line or "Yaw" in line or "Number of data points:" in line or not line:  # or "MICROSEC" in line:
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
    
def save_csv_IMU(filename, data_lines):
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
    # ser.write(b"t\r\n")
    # ser.write(b"r\r\n")
    # ser.write(b"l\r\n")
    #
    # sleep(3)
    #
    # ser.write(b"e\r\n")
    # ser.write(b"k\r\n")

    # ser.write(b"r\r\n")
    ser.write(b"t\r\n")
    sleep(12)
    ser.write(b"z\r\n")
    
    
    while not ser.in_waiting: continue

    print("starting to read")
    num_loops = 0
    while True:
        num_loops += 1
        line = ser.readline().decode(errors="replace").strip()
        if not line:
            continue
        print(f"line: {line}")
        if line.startswith("Euler"):
            print("Reading Euler Angle data...")
            angle_data, next_line = read_IMU_data(ser)
            break
        # if line.startswith("Yaw"):
        #     print("Reading Yaw rate  data...")
        #     yaw_data, _ = read_IMU_data(ser)
        #     break

print("finished reading data")
with open("Euler_Angle.csv", 'w') as file:
    file.seek(0)
    file.truncate()
    save_csv("Euler_Angle.csv", angle_data)
# with open("Yaw_rate.csv", 'w') as file:
#     file.seek(0)
#     file.truncate()
#     save_csv("Yaw_rate.csv", yaw_data)

data_Euler = pd.read_csv("Euler_Angle.csv", header=None, names=["time", "EulerAngle", "YawRate"])

plt.figure()
plt.plot(data_Euler["time"], data_Euler["EulerAngle"])

plt.title("Euler Angle vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Euler Angle (rad)")

# Plot
plt.figure()
plt.plot(data_Euler["time"], data_Euler["YawRate"])
plt.title("Yaw Rate vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Yaw Rate (rad/s)")

plt.show()


    