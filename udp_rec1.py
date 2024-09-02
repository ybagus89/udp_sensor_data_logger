import socket
import csv
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to a specific address and port
server_address = ('192.168.4.4', 55555)
sock.bind(server_address)

print('UDP server started. Waiting for data...')

# Create lists to store data
data_lists = [
    [],  # orientation
    [],  # ang_velocity
    [],  # linear_accel
    [],  # magnetometer
    [],  # accelerometer
    [],  # gravity
    [],  # position
    []  # time_data
]

# Initialize position and velocity variables
xPos = 0.0
yPos = 0.0
headingVel = 0.0

# Constants
ACCEL_VEL_TRANSITION = 0.01  # assuming 10ms sampling rate
ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION
DEG_2_RAD = 0.01745329251  # trig functions require radians, BNO055 outputs degrees

while True:
    # Receive UDP data
    data, addr = sock.recvfrom(1024)
    print("Data:", data.decode())

    # Process received data
    data_split = data.decode().split(",")
    euler_angles = [float(x) for x in data_split[:3]]
    gyro_angles = [float(x) for x in data_split[3:6]]
    linear_accel_values = [float(x) for x in data_split[6:9]]
    magnetometer_values = [float(x) for x in data_split[9:12]]
    accelerometer_values = [float(x) for x in data_split[12:15]]
    gravity_values = [float(x) for x in data_split[15:18]]
    position_values = [float(x) for x in data_split[18:21]]

    # Update position and velocity
    xPos += ACCEL_POS_TRANSITION * linear_accel_values[0]
    yPos += ACCEL_POS_TRANSITION * linear_accel_values[1]
    headingVel = ACCEL_VEL_TRANSITION * linear_accel_values[0] / math.cos(DEG_2_RAD * euler_angles[0])

    # Append data to lists
    data_lists[0].append(euler_angles)
    data_lists[1].append(gyro_angles)
    data_lists[2].append(linear_accel_values)
    data_lists[3].append(magnetometer_values)
    data_lists[4].append(accelerometer_values)
    data_lists[5].append(gravity_values)
    data_lists[6].append(position_values)
    data_lists[7].append(len(data_lists[7]) * 0.01)  # assuming 10ms sampling rate

    time.sleep(0.01)

    # Print data
    #print("xPos:", xPos)
    #print("yPos:", yPos)
    #print("headingVel:", headingVel)

    # Save data to CSV file
    with open('experiment3_08316.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([
            'Time (s)', 'Euler X (degrees)', 'Euler Y (degrees)', 'Euler Z (degrees)',
            'Gyro X (degrees/s)', 'Gyro Y (degrees/s)', 'Gyro Z (degrees/s)',
            'Linear Accel X (m/s^2)', 'Linear Accel Y (m/s^2)', 'Linear Accel Z (m/s^2)',
            'Magnetometer X (uT)', 'Magnetometer Y (uT)', 'Magnetometer Z (uT)',
            'Accelerometer X (m/s^2)', 'Accelerometer Y (m/s^2)', 'Accelerometer Z (m/s^2)',
            'Gravity X (m/s^2)', 'Gravity Y (m/s^2)', 'Gravity Z (m/s^2)','xpos','ypos','Speed','xpos1','ypos1','Speed1'
        ])  # header row
        for i in range(len(data_lists[7])):
            writer.writerow([
                data_lists[7][i],
                *data_lists[0][i], *data_lists[1][i], *data_lists[2][i],
                *data_lists[3][i], *data_lists[4][i], *data_lists[5][i], *data_lists[6][i],
                xPos, yPos, headingVel
            ])

print('Data saved to data.csv')