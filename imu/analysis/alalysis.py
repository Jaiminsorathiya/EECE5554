import rosbag
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import os

def read_imu_bag(bag_file_path):
    timestamps, angular_velocities, linear_accelerations, quaternions = [], [], [], []

    if not os.path.exists(bag_file_path):
        print(f"ROS bag file not found at {bag_file_path}")
        exit(1)

    with rosbag.Bag(bag_file_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/imu']):
            timestamps.append(t.to_sec())
            angular_velocities.append((msg.imu.angular_velocity.x, msg.imu.angular_velocity.y, msg.imu.angular_velocity.z))
            linear_accelerations.append((msg.imu.linear_acceleration.x, msg.imu.linear_acceleration.y, msg.imu.linear_acceleration.z))
            quaternions.append((msg.imu.orientation.x, msg.imu.orientation.y, msg.imu.orientation.z, msg.imu.orientation.w))

    if not timestamps:
        print("No data extracted from the ROS bag. Check topic names and bag content.")
        exit(1)

    timestamps = np.array(timestamps)
    timestamps -= timestamps[0]

    return timestamps, angular_velocities, linear_accelerations, quaternions

def quaternions_to_euler(quaternions):
    euler_angles = R.from_quat(quaternions).as_euler('xyz', degrees=True)
    return euler_angles[:, 0], euler_angles[:, 1], euler_angles[:, 2]

def plot_gyro_rotational_rate(timestamps, angular_velocities):
    plt.figure(figsize=(15, 5))
    for i, axis in enumerate(['X', 'Y', 'Z']):
        plt.plot(timestamps, [ang_vel[i] for ang_vel in angular_velocities], label=f'{axis}')
    plt.title('Gyro Rotational Rate')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (deg/s)')
    plt.legend()
    plt.show()

def plot_accelerometer_linear_acceleration(timestamps, linear_accelerations):
    plt.figure(figsize=(15, 5))
    for i, axis in enumerate(['X', 'Y', 'Z']):
        plt.plot(timestamps, [acc[i] for acc in linear_accelerations], label=f'{axis}')
    plt.title('Accelerometer Linear Acceleration')
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (m/s^2)')
    plt.legend()
    plt.show()

def plot_euler_angles(timestamps, roll, pitch, yaw):
    plt.figure(figsize=(15, 5))
    plt.plot(timestamps, roll, label='Roll')
    plt.plot(timestamps, pitch, label='Pitch')
    plt.plot(timestamps, yaw, label='Yaw')
    plt.title('Euler Angles from Quaternion')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (degrees)')
    plt.legend()
    plt.show()

def plot_rotation_histograms(roll, pitch, yaw):
    fig, axs = plt.subplots(1, 3, figsize=(18, 5))
    for i, angle in enumerate([roll, pitch, yaw]):
        axs[i].hist(angle, bins=50)
        axs[i].set_title(f'{["Roll", "Pitch", "Yaw"][i]} Distribution')
        axs[i].set_xlabel(f'{["Roll", "Pitch", "Yaw"][i]} (degrees)')
        axs[i].set_ylabel('Frequency')

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    imu_bag_path = '/home/jaiminsorathiya/EECE5554/imu/data/IMU_data.bag'
    
    timestamps, angular_velocities, linear_accelerations, quaternions = read_imu_bag(imu_bag_path)
    roll, pitch, yaw = quaternions_to_euler(quaternions)

    plot_gyro_rotational_rate(timestamps, angular_velocities)
    plot_accelerometer_linear_acceleration(timestamps, linear_accelerations)
    plot_euler_angles(timestamps, roll, pitch, yaw)
    plot_rotation_histograms(roll, pitch, yaw)

