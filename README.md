# udp_sensor_data_logger
This Python script functions as a UDP server designed to receive, process, and log real-time sensor data. The script listens on a specified IP address and port, capturing various types of sensor data transmitted in packets.
Key Features:
1. UDP Server: Listens for incoming data on a defined IP and port.
2. Data Parsing: Processes incoming sensor data, including orientation (Euler angles), angular velocity, linear acceleration, magnetometer readings, accelerometer readings, gravity vectors, and position data.
3. Real-Time Data Logging: Logs the processed data into a CSV file, providing time-stamped entries for each data packet.
4. Position and Velocity Calculation: Computes and updates positional and velocity data based on received accelerometer values.
5. Visualization Ready: The data is stored in a format suitable for further analysis and visualization using tools like Matplotlib.
