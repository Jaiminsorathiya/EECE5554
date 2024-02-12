import matplotlib.pyplot as plt
import numpy as np
import rosbag
import utm

def process_and_plot_data(bag_path, topic_name):
    def read_data(bag_path, topic_name):
        eastings, northings, times = [], [], []
        with rosbag.Bag(bag_path, 'r') as bag:
            for _, msg, t in bag.read_messages(topics=[topic_name]):
                e, n, _, _ = utm.from_latlon(msg.latitude, msg.longitude)
                eastings.append(e)
                northings.append(n)
                times.append(t.to_sec())

        eastings = np.array(eastings)
        northings = np.array(northings)
        times = np.array(times) - times[0]
        return eastings, northings, times

    def calculate_and_print_error(eastings, northings):
        fit_coeffs = np.polyfit(northings, eastings, 1)
        fit_fn = np.poly1d(fit_coeffs)

        predicted_eastings = fit_fn(northings)
        residuals = eastings - predicted_eastings
        std_dev_error = np.std(residuals)

        print(f'Error from the line of best fit: {std_dev_error} meters')

    eastings, northings, times = read_data(bag_path, topic_name)
    calculate_and_print_error(eastings, northings)

    # Plotting
    plt.figure(figsize=(10, 6))
    plt.scatter(northings, eastings, marker='o', color='blue', label='GPS Data')
    plt.plot(northings, fit_fn(northings), 'r-', label='Best Fit Line')
    plt.title('Northing vs Easting Scatter Plot with Best Fit Line')
    plt.xlabel('Northing in meters')
    plt.ylabel('Easting in meters')
    plt.legend()
    plt.grid(True)
    plt.show()

# Specify your ROS bag file path and topic
bag_path = '/home/jaiminsorathiya/EECE5554/gnss/data/moving.bag'
topic_name = '/gps'

process_and_plot_data(bag_path, topic_name)

