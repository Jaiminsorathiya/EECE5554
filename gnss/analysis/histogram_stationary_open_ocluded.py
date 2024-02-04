import numpy as np
import matplotlib.pyplot as plt
import rosbag
from gps_driver.msg import Customgps  # Assuming Customgps is your custom message

class UTMDataAnalyzer:
    def __init__(self, open_bag_path, occluded_bag_path):
        self.open_bag_path = open_bag_path
        self.occluded_bag_path = occluded_bag_path

    def read_utm_data(self, bag_path):
        bag = rosbag.Bag(bag_path)
        utm_easting = []
        utm_northing = []
        for topic, msg, t in bag.read_messages(topics=['/gps']):
            utm_easting.append(msg.utm_easting)
            utm_northing.append(msg.utm_northing)
        bag.close()
        return np.array(utm_easting), np.array(utm_northing)

    def calculate_distances(self, easting, northing, centroid_easting, centroid_northing):
        distances = np.sqrt((easting - centroid_easting)**2 + (northing - centroid_northing)**2)
        return distances

    def analyze_and_plot_histograms(self):
        open_easting, open_northing = self.read_utm_data(self.open_bag_path)
        occluded_easting, occluded_northing = self.read_utm_data(self.occluded_bag_path)

        centroid_open_easting, centroid_open_northing = np.mean(open_easting), np.mean(open_northing)
        centroid_occluded_easting, centroid_occluded_northing = np.mean(occluded_easting), np.mean(occluded_northing)

        distances_open = self.calculate_distances(open_easting, open_northing, centroid_open_easting, centroid_open_northing)
        distances_occluded = self.calculate_distances(occluded_easting, occluded_northing, centroid_occluded_easting, centroid_occluded_northing)

        fig, axs = plt.subplots(1, 2, figsize=(16, 6))

        axs[0].hist(distances_open, bins=15, alpha=0.7, color='blue', label='Open Data - Stationary')
        axs[0].set_xlabel('Distance to Centroid (meters)')
        axs[0].set_ylabel('Frequency')
        axs[0].set_title('Histogram of Distances for Open Data')
        axs[0].legend()
        axs[0].grid(True)

        axs[1].hist(distances_occluded, bins=15, alpha=0.7, color='red', label='Occluded Data - Stationary')
        axs[1].set_xlabel('Distance to Centroid (meters)')
        axs[1].set_ylabel('Frequency')
        axs[1].set_title('Histogram of Distances for Occluded Data')
        axs[1].legend()
        axs[1].grid(True)

        plt.tight_layout()
        plt.show()

def main():
    open_bag_path = '/home/jaiminsorathiya/EECE5554/gnss/data/open_data.bag'
    occluded_bag_path = '/home/jaiminsorathiya/EECE5554/gnss/data/oclud.bag'

    analyzer = UTMDataAnalyzer(open_bag_path, occluded_bag_path)
    analyzer.analyze_and_plot_histograms()

if __name__ == '__main__':
    main()

