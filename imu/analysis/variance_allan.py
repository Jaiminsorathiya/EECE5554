import rosbag
import numpy as np
import matplotlib.pyplot as plt
import allantools as atl

# Define the path to your ROS bag file and the corresponding topic
rosbag_path = '/home/jaiminsorathiya/EECE5554/imu/data/LocationC.bag'
topic = '/vectornav'

# Initialize lists to store gyro data for each axis
gyro_x_data, gyro_y_data, gyro_z_data = [], [], []

def clean_and_convert(value):
    """Remove non-numeric characters and convert to float."""
    try:
        return float(value.replace('\x00', '').split('*')[0])
    except ValueError:
        return None

def parse_vnymr_string(data_string):
    """Parse and extract gyro data from the VNYMR string."""
    parts = data_string.split(',')
    if len(parts) < 12:
        return  # Skip incomplete data strings
    try:
        x_value = clean_and_convert(parts[9])
        y_value = clean_and_convert(parts[10])
        z_value = clean_and_convert(parts[11].split('*')[0])
        if x_value is not None and y_value is not None and z_value is not None:
            gyro_x_data.append(x_value)
            gyro_y_data.append(y_value)
            gyro_z_data.append(z_value)
    except ValueError as error:
        print(f"Skipping invalid data due to error: {error}")

# Process the ROS bag
with rosbag.Bag(rosbag_path, 'r') as bag:
    for _, message, _ in bag.read_messages(topics=[topic]):
        parse_vnymr_string(message.data)

# Check if data was extracted
if not gyro_x_data or not gyro_y_data or not gyro_z_data:
    print("Failed to extract gyro data. Exiting.")
    exit()

# Function to calculate and plot Allan variance for each gyro axis
def calculate_and_plot_allan_variance(data, label):
    tau, adev, _, _ = atl.oadev(data, rate=40, data_type='freq', taus='all')
    adev_sqrt = np.sqrt(adev)
    
    # Find noise parameters
    bias_instability = np.min(adev_sqrt)
    angle_random_walk = adev_sqrt[np.argmin(np.abs(tau - 1))]
    idx_min = np.argmin(adev_sqrt)
    rate_random_walk, _ = np.polyfit(np.log10(tau[idx_min:]), np.log10(adev_sqrt[idx_min:]), 1)
    
    plt.loglog(tau, adev_sqrt, label=f'{label} (Bias={bias_instability:.3e}, ARW={angle_random_walk:.3e}, RRW={rate_random_walk:.2f})')
    print(f"{label}: Bias Instability = {bias_instability:.3e}, Angle Random Walk = {angle_random_walk:.3e}, Rate Random Walk = {rate_random_walk:.2f}")

plt.figure(figsize=(12, 8))
calculate_and_plot_allan_variance(gyro_x_data, 'Gyro X')
calculate_and_plot_allan_variance(gyro_y_data, 'Gyro Y')
calculate_and_plot_allan_variance(gyro_z_data, 'Gyro Z')

plt.xlabel('Log(Time Interval / s)')
plt.ylabel('Log(Allan Deviation)')
plt.legend()
plt.title('Allan Deviation Analysis for Gyroscopes')
plt.grid(True, which="both", ls="--")
plt.show()

