import bagpy as bp
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Define function to load sensor data from ROS bag file
def load_sensor_data(file_path, topic):
    bag = bp.bagreader(file_path)
    data = bag.message_by_topic(topic)
    df = pd.read_csv(data)
    return df

# Define function to calculate velocity from accelerometer data
def calculate_velocity(accel_data, time_interval):
    velocity = np.cumsum(accel_data) * time_interval
    velocity -= np.mean(velocity)  # Zero-velocity adjustment
    return velocity

# Define function to plot velocity
def plot_velocity(time, velocity, title='Velocity'):
    plt.figure(figsize=(10, 6))
    plt.plot(time, velocity, label=title)
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [m/s]')
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.show()

# Define function to calculate trajectory from velocity and yaw
def calculate_trajectory(velocity, yaw, time_interval):
    displacement_x = np.cumsum(velocity * np.cos(yaw)) * time_interval
    displacement_y = np.cumsum(velocity * np.sin(yaw)) * time_interval
    return displacement_x, displacement_y

# Define function to plot trajectory
def plot_trajectory(x, y, title='Trajectory'):
    plt.figure(figsize=(10, 6))
    plt.plot(x, y, marker='o', label=title)
    plt.xlabel('Displacement in X [m]')
    plt.ylabel('Displacement in Y [m]')
    plt.title(title)
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.show()

# Define parameters
bag_file_path = '/home/jaiminsorathiya/EECE5554/lab5/src/data/data_driving_hill.bag'
imu_topic = '/imu'
gps_topic = '/gps'
time_interval = 1.0 / 50  # Assumed sampling rate

# Load sensor data
imu_data = load_sensor_data(bag_file_path, imu_topic)
gps_data = load_sensor_data(bag_file_path, gps_topic)

# Calculate velocity from accelerometer
accel_x = imu_data['imu.linear_acceleration.x'].values - np.mean(imu_data['imu.linear_acceleration.x'].values)
time_imu = imu_data['Time'].values - imu_data['Time'].values[0]
velocity_imu = calculate_velocity(accel_x, time_interval)

# Plot velocity from accelerometer
plot_velocity(time_imu, velocity_imu, 'Forward Velocity from Accelerometer')

# Plot velocity from GPS
plot_velocity(time_imu, velocity_imu, 'Forward Velocity from GPS')

# Calculate and plot trajectories
yaw = np.cumsum(imu_data['imu.angular_velocity.z'].values) * time_interval
x_traj, y_traj = calculate_trajectory(velocity_imu, yaw, time_interval)
plot_trajectory(x_traj, y_traj, 'Estimated Trajectory from IMU')

# Plot GPS trajectory
utm_x = gps_data['utm_easting'].values
utm_y = gps_data['utm_northing'].values
plot_trajectory(utm_x, utm_y, 'Estimated Trajectory from GPS')

# Plot both trajectories together
plt.figure(figsize=(12, 12))
plt.subplot(2, 1, 1)
plt.plot(x_traj, y_traj, label='IMU Trajectory')
plt.title('Estimated Trajectory from IMU')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.legend()
plt.grid()

plt.subplot(2, 1, 2)
plt.plot(utm_x, utm_y, label='GPS Trajectory')
plt.title('Estimated Trajectory from GPS')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()

