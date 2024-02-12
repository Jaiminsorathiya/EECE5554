import rosbag
import matplotlib.pyplot as plt
import numpy as np
import utm

def read_bag_data(bag_path, topic_name):
    eastings, northings = [], []

    with rosbag.Bag(bag_path, 'r') as bag:
        for _, msg, _ in bag.read_messages(topics=[topic_name]):
            e, n, _, _ = utm.from_latlon(msg.latitude, msg.longitude)
            eastings.append(e)
            northings.append(n)

    return np.array(eastings), np.array(northings)

def centralize_data_around_centroid(eastings, northings):
    centroid = (np.mean(eastings), np.mean(northings))
    eastings -= centroid[0]
    northings -= centroid[1]
    return eastings, northings

def plot_scatterplots_centered_on_centroids(open_data, occluded_data):
    open_eastings, open_northings = open_data
    occluded_eastings, occluded_northings = occluded_data

    # Centralize data around centroids
    open_eastings, open_northings = centralize_data_around_centroid(open_eastings, open_northings)
    occluded_eastings, occluded_northings = centralize_data_around_centroid(occluded_eastings, occluded_northings)

    # Plot
    plt.figure(figsize=(10, 8))
    plt.scatter(open_northings, open_eastings, c='blue', label='Open')
    plt.scatter(occluded_northings, occluded_eastings, c='red', label='Occluded')
    plt.xlabel('Northing - Centered')
    plt.ylabel('Easting - Centered')
    plt.title('Stationary Northing vs Easting Scatterplots Centered on Centroids')
    plt.legend()
    plt.grid(True)
    plt.annotate(f'Total Offset Open: Easting={open_eastings.ptp():.2f}, Northing={open_northings.ptp():.2f}', 
                 xy=(0.05, 0.95), xycoords='axes fraction')
    plt.annotate(f'Total Offset Occluded: Easting={occluded_eastings.ptp():.2f}, Northing={occluded_northings.ptp():.2f}', 
                 xy=(0.05, 0.90), xycoords='axes fraction')
    plt.show()

def main():
    # Paths to your ROS bag files
    open_bag_path = '/home/jaiminsorathiya/EECE5554/gnss/data/openRTK.bag'
    occluded_bag_path = '/home/jaiminsorathiya/EECE5554/gnss/data/occludedRTK.bag'
    topic_name = '/rtk_gps'

    # Read data
    open_data = read_bag_data(open_bag_path, topic_name)
    occluded_data = read_bag_data(occluded_bag_path, topic_name)

    # Plot scatterplots centered on centroids
    plot_scatterplots_centered_on_centroids(open_data, occluded_data)

if __name__ == "__main__":
    main()

