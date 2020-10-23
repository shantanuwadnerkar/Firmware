#!/usr/bin/env python3

import matplotlib.pyplot as plt
import pandas as pd

one_sec_in_us = 1000000

# Location of the log files. Please change the date folder accordinly
csv_file_location = "../../../../build/px4_sitl_default/tmp/rootfs/log/2020-10-22/"
# csv_file1 = csv_file_location + "18_30_35_sensor_accel_integration_0.csv"
# csv_file2 = csv_file_location + "18_30_35_vehicle_local_position_0.csv"

# Above file location is not necessary if the log files exist in the same location as this script. 
# Provide the file names for the uORB topics "integrated_accel" and "vehicle_local_position" respectively.
csv_file1 = "19_13_04_integrated_accel_0.csv"
csv_file2 = "19_13_04_vehicle_local_position_0.csv"


# Read data from integrated_accel uORB topic
data1 = pd.read_csv(csv_file1)

vel_ts = list(data1["timestamp"])
vel_x = list(data1["vel_x"])
vel_y = list(data1["vel_y"])
vel_z = list(data1["vel_z"])

vel_len = len(vel_x)
vel_start_ts = data1["timestamp"][0]
vel_end_ts = data1["timestamp"][vel_len - 1]


# Read data from vehicle_local_position uORB topic
data2 = pd.read_csv(csv_file2)

estimate_vel_ts = data2["timestamp"]
estimate_vel_x = data2["vx"]
estimate_vel_y = data2["vy"]
estimate_vel_z = data2["vz"]

estimate_len = len(estimate_vel_x)
estimate_start_idx = -1
estimate_end_idx = -1

# Timestamps for integrated_accel are a subset of vehicle_local_position topic; integrated_accel 
# works only when the UAV is armed while vehicle_local_position is recorded for the whole duration of the log file,
# Find the topics from the same range for vehicle_local_position and plot only those.
# Note: integrated_accel and vehicle_local_position are recorded at different rates.
# So the range of data points while the drone is armed might vary by a few data points. That's why they are plotted separately.
for idx in range(estimate_len):
    if (estimate_start_idx == -1) and abs(vel_start_ts - estimate_vel_ts[idx]) < one_sec_in_us:
        estimate_start_idx = idx

    if (estimate_end_idx == -1) and abs(vel_end_ts - estimate_vel_ts[idx]) < one_sec_in_us:
        estimate_end_idx = idx


# Plot figures
plt.figure()
plt.subplot(211)
plt.plot(vel_ts, vel_x, 'b-', label="vel_x")
plt.plot(vel_ts, vel_y, 'g-', label="vel_y")
plt.plot(vel_ts, vel_z, 'r-', label="vel_z")

plt.title('Integrated Accel (velocities)')
plt.legend(loc="lower left")

plt.ylim(-300.0, 20.0)
plt.xlabel('Timestamp')
plt.ylabel('Velocity (m/s)')


plt.subplot(212)
plt.plot(estimate_vel_ts[estimate_start_idx:estimate_end_idx], estimate_vel_x[estimate_start_idx:estimate_end_idx], 'b-',
            estimate_vel_ts[estimate_start_idx:estimate_end_idx], estimate_vel_y[estimate_start_idx:estimate_end_idx], 'g-',
            estimate_vel_ts[estimate_start_idx:estimate_end_idx], estimate_vel_z[estimate_start_idx:estimate_end_idx], 'r-')

plt.title('Vehicle Location Estimate (velocities)')
plt.legend(loc="lower left")

plt.ylim(-4.5, 1.5)
plt.xlabel('Timestamp')
plt.ylabel('Velocity (m/s)')

plt.show()
