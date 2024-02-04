import matplotlib.pyplot as plt
import rosbag


class UTMDataVisualizer:
    def __init__(self, bag_path):
        self.bag_path = bag_path
        self.utm_easting = []
        self.utm_northing = []

    def read_utm_data(self):
        with rosbag.Bag(self.bag_path) as bag:
            for topic, msg, t in bag.read_messages(topics=['/gps']):
                self.utm_easting.append(msg.utm_easting)
                self.utm_northing.append(msg.utm_northing)

    def normalize_data(self):
        self.utm_easting = [e - self.utm_easting[0] for e in self.utm_easting]
        self.utm_northing = [n - self.utm_northing[0] for n in self.utm_northing]

    def create_scatter_plot(self):
        plt.scatter(self.utm_northing, self.utm_easting, marker='.', color='b')
        plt.title('Northing vs Easting Open Walking')
        plt.xlabel('Northing in meters')
        plt.ylabel('Easting in meters')
        plt.grid(True)
        plt.show()

def main():
    bag_path = '/home/jaiminsorathiya/EECE5554/gnss/data/moving.bag'

    visualizer = UTMDataVisualizer(bag_path)
    visualizer.read_utm_data()
    visualizer.normalize_data()
    visualizer.create_scatter_plot()

if __name__ == '__main__':
    main()

