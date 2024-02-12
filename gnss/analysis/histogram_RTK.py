import rosbag
import matplotlib.pyplot as plt
import numpy as np
import utm

def read_data(bag_path, topic):
    eastings, northings = [], []
    
    with rosbag.Bag(bag_path, 'r') as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            e, n, _, _ = utm.from_latlon(msg.latitude, msg.longitude)
            eastings.append(e)
            northings.append(n)
    
    return np.array(eastings), np.array(northings)

def calculate_distance(eastings, northings, ref_easting, ref_northing):
    distances = np.sqrt((eastings - ref_easting)**2 + (northings - ref_northing)**2)
    return distances

def plot_histogram(data, label, color='blue'):
    plt.hist(data, bins=20, alpha=0.7, label=label, color=color)
    plt.xlabel('Distance to Reference Point (m)')
    plt.ylabel('Frequency')
    plt.title(f'{label} Data')
    plt.legend()

def main():
    # Reference coordinates converted to UTM
    ref_open_easting, ref_open_northing, _, _ = utm.from_latlon(42.3382544, -71.0865255)
    ref_occluded_easting, ref_occluded_northing, _, _ = utm.from_latlon(42.3372702, -71.0869305)

    # Read data
    open_eastings, open_northings = read_data('/home/jaiminsorathiya/EECE5554/gnss/data/open_RTK.bag.bag', '/rtk_gps')
    occluded_eastings, occluded_northings = read_data('/home/jaiminsorathiya/EECE5554/gnss/data/occludedRTK.bag', '/rtk_gps')

    # Calculate distances
    distances_open = calculate_distance(open_eastings, open_northings, ref_open_easting, ref_open_northing)
    distances_occluded = calculate_distance(occluded_eastings, occluded_northings, ref_occluded_easting, ref_occluded_northing)

    # Plot histograms
    plt.figure(figsize=(12, 6))

    plt.subplot(1, 2, 1)
    plot_histogram(distances_open, 'Open', color='blue')

    plt.subplot(1, 2, 2)
    plot_histogram(distances_occluded, 'Occluded', color='red')

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()

