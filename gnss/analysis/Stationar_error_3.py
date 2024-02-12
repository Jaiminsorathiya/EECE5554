import rosbag
import matplotlib.pyplot as plt
import numpy as np
import utm

def process_and_plot_data(bag_path_open, bag_path_occluded, topic):
    def read_bag(bag_path, topic):
        eastings, northings = [], []
        with rosbag.Bag(bag_path, 'r') as bag:
            for _, msg, _ in bag.read_messages(topics=[topic]):
                e, n, _, _ = utm.from_latlon(msg.latitude, msg.longitude)
                eastings.append(e)
                northings.append(n)
        return np.array(eastings), np.array(northings)

    def calculate_and_print_error(eastings, northings, label):
        centroid = (np.mean(eastings), np.mean(northings))
        eastings -= centroid[0]
        northings -= centroid[1]

        deviation_squared = np.square(eastings) + np.square(northings)
        mean_deviation = np.mean(deviation_squared)
        error = np.sqrt(mean_deviation)

        print(f'Error for {label}: {error}')

    eastings_open, northings_open = read_bag(bag_path_open, topic)
    calculate_and_print_error(eastings_open, northings_open, 'rtk open')

    eastings_occluded, northings_occluded = read_bag(bag_path_occluded, topic)
    calculate_and_print_error(eastings_occluded, northings_occluded, 'rtk occluded')

# Paths of ROS bag files
bag_path_open = '/home/jaiminsorathiya/EECE5554/gnss/data/openRTK.bag'
bag_path_occluded = '/home/jaiminsorathiya/EECE5554/gnss/data/occludedRTK.bag'
topic_name = '/rtk'

process_and_plot_data(bag_path_open, bag_path_occluded, topic_name)

