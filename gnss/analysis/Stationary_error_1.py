import rosbag
import matplotlib.pyplot as plt
import numpy as np
import utm

def process_and_plot_gps_data(bag_path_open, bag_path_occluded):
    def read_gps_data(bag_path):
        eastings, northings = [], []

        with rosbag.Bag(bag_path, 'r') as bag:
            for topic, msg, t in bag.read_messages(topics=['/gps']):
                e, n, _, _ = utm.from_latlon(msg.latitude, msg.longitude)
                eastings.append(e)
                northings.append(n)

        # Normalize UTM coordinates by subtracting the first measurement
        eastings = np.array(eastings) - eastings[0]
        northings = np.array(northings) - northings[0]

        return eastings, northings

    def filter_data(eastings, northings, northing_limit=(-50, 300), easting_limit=(-500, 500)):
        filtered_eastings, filtered_northings = [], []
        for e, n in zip(eastings, northings):
            if northing_limit[0] <= n <= northing_limit[1] and easting_limit[0] <= e <= easting_limit[1]:
                filtered_eastings.append(e)
                filtered_northings.append(n)
        return filtered_eastings, filtered_northings

    eastings_open, northings_open = read_gps_data(bag_path_open)
    eastings_occluded, northings_occluded = read_gps_data(bag_path_occluded)

    # Filter the data points based on specified limits
    eastings_open, northings_open = filter_data(eastings_open, northings_open)
    eastings_occluded, northings_occluded = filter_data(eastings_occluded, northings_occluded)

    # Calculate and print error for standalone_gps open
    error_open = np.sqrt(np.mean(np.square(eastings_open)) + np.mean(np.square(northings_open)))
    print(f'Error for standalone_gps open: {error_open}')

    # Calculate and print error for standalone_gps occluded
    error_occluded = np.sqrt(np.mean(np.square(eastings_occluded)) + np.mean(np.square(northings_occluded)))
    print(f'Error for standalone_gps occluded: {error_occluded}')

    # Plotting
    plt.figure(figsize=(10, 6))
    plt.scatter(northings_open, eastings_open, marker='o', color='blue', label='Open Conditions')
    plt.scatter(northings_occluded, eastings_occluded, marker='x', color='red', label='Occluded Conditions')
    plt.title('Northing vs Easting Scatter Plot')
    plt.xlabel('Northing in meters')
    plt.ylabel('Easting in meters')
    plt.xlim(-50, 300)
    plt.ylim(-500, 500)
    plt.legend()
    plt.grid(True)
    plt.show()

# Replace the placeholders with your actual ROS bag file paths
bag_path_open = '/home/jaiminsorathiya/EECE5554/gnss/data/openRTK.bag'
bag_path_occluded = '/home/jaiminsorathiya/EECE5554/gnss/data/occludedRTK.bag'
process_and_plot_gps_data(bag_path_open, bag_path_occluded)

