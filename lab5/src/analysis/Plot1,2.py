import bagpy as bp
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

plt.rcParams.update({'font.size': 10})
sns.set_style("dark")
sns.color_palette("viridis", as_cmap=True)

def load_magnetometer_data(file_path, topic_name):
    bag = bp.bagreader(file_path)
    data = bag.message_by_topic(topic_name)
    readings = pd.read_csv(data)
    return readings

def hard_iron_calibration(magnetometer_data):
    min_x = magnetometer_data['mag_field.magnetic_field.x'].min()
    max_x = magnetometer_data['mag_field.magnetic_field.x'].max()
    min_y = magnetometer_data['mag_field.magnetic_field.y'].min()
    max_y = magnetometer_data['mag_field.magnetic_field.y'].max()

    x_offset = (min_x + max_x) / 2.0
    y_offset = (min_y + max_y) / 2.0
    print("Hard Iron X Offset:", x_offset)
    print("Hard Iron Y Offset:", y_offset)

    hard_iron_x = magnetometer_data['mag_field.magnetic_field.x'] - x_offset
    hard_iron_y = magnetometer_data['mag_field.magnetic_field.y'] - y_offset

    return hard_iron_x, hard_iron_y

def soft_iron_calibration(hard_iron_x, hard_iron_y):
    data = np.vstack((hard_iron_x, hard_iron_y))
    cov_matrix = np.cov(data)
    eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)
    mean_x, mean_y = np.mean(data[0, :]), np.mean(data[1, :])
    centered_data = data - np.array([[mean_x], [mean_y]])
    rotated_data = eigenvectors.T.dot(centered_data)
    scale_factors = np.sqrt(eigenvalues)
    max_scale = max(scale_factors)
    adjusted_scale_factors = scale_factors / max_scale
    scaled_data = np.diag(1 / adjusted_scale_factors).dot(rotated_data)

    return scaled_data

def calculate_yaw(mag_x, mag_y):
    yaw = np.arctan2(mag_y, mag_x)
    return np.degrees(yaw)

file_path = '/home/jaiminsorathiya/EECE5554/lab5/src/data/data_going_in_circles.bag'
topic_name = '/imu'
readings = load_magnetometer_data(file_path, topic_name)

hard_iron_x, hard_iron_y = hard_iron_calibration(readings)
calibrated_data = soft_iron_calibration(hard_iron_x, hard_iron_y)

yaw_uncalibrated = calculate_yaw(readings['mag_field.magnetic_field.x'], readings['mag_field.magnetic_field.y'])
yaw_calibrated = calculate_yaw(calibrated_data[0, :], calibrated_data[1, :])

time_array = readings['Time'].to_numpy()
yaw_uncalibrated_array = np.array(yaw_uncalibrated)
yaw_calibrated_array = np.array(yaw_calibrated)

plt.figure(figsize=(10, 8))
plt.scatter(readings['mag_field.magnetic_field.x'], readings['mag_field.magnetic_field.y'], color='blue', label='Before Correction')
plt.scatter(calibrated_data[0, :], calibrated_data[1, :], color='red', label='After Correction')
plt.title('Magnetometer Data Before and After Correction')
plt.xlabel('Magnetic Field X (Guass)')
plt.ylabel('Magnetic Field Y (Guass)')
plt.legend()
plt.tight_layout()
plt.show()

plt.figure(figsize=(10, 8))
plt.plot(time_array, yaw_uncalibrated_array, label='Yaw Before Calibration')
plt.plot(time_array, yaw_calibrated_array, label='Yaw After Calibration', color='orange')
plt.title('Yaw Estimation Before and After Calibration')
plt.xlabel('Time')
plt.ylabel('Yaw (Degrees)')
plt.legend()
plt.tight_layout()
plt.show()

