import rosbag
import matplotlib.pyplot as plt
import numpy as np
import utm

class GPSDataPlotter:
    def __init__(self, open_bag_path, occluded_bag_path):
        self.open_bag_path = open_bag_path
        self.occluded_bag_path = occluded_bag_path
        self.utm_easting_open, self.utm_northing_open = [], []
        self.utm_easting_occluded, self.utm_northing_occluded = [], []

    def process_gps_data(self, bag_path, utm_easting, utm_northing):
        with rosbag.Bag(bag_path, 'r') as bag:
            for topic, msg, t in bag.read_messages(topics=['/gps']):
                e, n, _, _ = utm.from_latlon(msg.latitude, msg.longitude)
                utm_easting.append(e)
                utm_northing.append(n)

    def normalize_utm_coordinates(self):
        self.utm_easting_open = np.array(self.utm_easting_open) - self.utm_easting_open[0]
        self.utm_northing_open = np.array(self.utm_northing_open) - self.utm_northing_open[0]
        self.utm_easting_occluded = np.array(self.utm_easting_occluded) - self.utm_easting_occluded[0]
        self.utm_northing_occluded = np.array(self.utm_northing_occluded) - self.utm_northing_occluded[0]

    def plot_scatter_plot(self):
        plt.figure(figsize=(10, 6))
        plt.scatter(self.utm_northing_open, self.utm_easting_open, marker='o', color='blue', label='Open Conditions')
        plt.scatter(self.utm_northing_occluded, self.utm_easting_occluded, marker='x', color='red', label='Occluded Conditions')
        plt.title('Northing vs Easting Scatter Plot')
        plt.xlabel('Northing in meters')
        plt.ylabel('Easting in meters')
        plt.legend()
        plt.grid(True)
        plt.show()

    def plot_gps_data(self):
        self.process_gps_data(self.open_bag_path, self.utm_easting_open, self.utm_northing_open)
        self.process_gps_data(self.occluded_bag_path, self.utm_easting_occluded, self.utm_northing_occluded)
        self.normalize_utm_coordinates()
        self.plot_scatter_plot()

def main():
    open_bag_path = '/home/jaiminsorathiya/EECE5554/gnss/data/open_data.bag'
    occluded_bag_path = '/home/jaiminsorathiya/EECE5554/gnss/data/oclud.bag'

    gps_plotter = GPSDataPlotter(open_bag_path, occluded_bag_path)
    gps_plotter.plot_gps_data()

if __name__ == '__main__':
    main()

