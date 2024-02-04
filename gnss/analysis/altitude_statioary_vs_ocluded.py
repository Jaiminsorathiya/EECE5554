import rosbag
import matplotlib.pyplot as plt
import numpy as np

class AltitudeDataAnalyzer:
    def __init__(self, open_bag_path, occluded_bag_path):
        self.open_bag_path = open_bag_path
        self.occluded_bag_path = occluded_bag_path

    def read_altitude_data(self, bag_path):
        bag = rosbag.Bag(bag_path)
        altitudes = []
        for topic, msg, t in bag.read_messages(topics=['/gps']):
            altitudes.append(msg.altitude)
        bag.close()
        return altitudes

    def plot_altitude_comparison(self):
        open_altitudes = self.read_altitude_data(self.open_bag_path)
        occluded_altitudes = self.read_altitude_data(self.occluded_bag_path)

        time_open = np.arange(len(open_altitudes))
        time_occluded = np.arange(len(occluded_altitudes))

        plt.figure(figsize=(10, 6))
        plt.scatter(time_open, open_altitudes, color='blue', label='Open Data')
        plt.scatter(time_occluded, occluded_altitudes, color='red', label='Occluded Data')
        plt.title('Altitude vs Time for Open and Occluded Conditions')
        plt.xlabel('Time (s)')
        plt.ylabel('Altitude (m)')
        plt.legend()
        plt.grid(True)
        plt.show()

def main():
    open_bag_path = '/home/jaiminsorathiya/EECE5554/gnss/data/open_data.bag'
    occluded_bag_path = '/home/jaiminsorathiya/EECE5554/gnss/data/oclud.bag'

    analyzer = AltitudeDataAnalyzer(open_bag_path, occluded_bag_path)
    analyzer.plot_altitude_comparison()

if __name__ == '__main__':
    main()

