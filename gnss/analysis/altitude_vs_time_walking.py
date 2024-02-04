import matplotlib.pyplot as plt
import rosbag
import matplotlib.dates as mdates
from datetime import datetime

class AltitudeDataPlotter:
    def __init__(self, bag_path):
        self.bag_path = bag_path
        self.altitudes = []
        self.timestamps = []

    def read_altitude_data(self):
        with rosbag.Bag(self.bag_path) as bag:
            for topic, msg, t in bag.read_messages(topics=['/gps']):
                self.altitudes.append(msg.altitude)
                timestamp = datetime.fromtimestamp(t.to_sec())
                self.timestamps.append(timestamp)

    def plot_altitude_vs_time(self):
        plt.figure(figsize=(10, 6))
        plt.plot(self.timestamps, self.altitudes, marker='.', linestyle='-', color='b')
        plt.title('Altitude vs. Time for Moving Data')
        plt.xlabel('Time')
        plt.ylabel('Altitude (meters)')
        plt.grid(True)

        plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
        plt.gca().xaxis.set_major_locator(mdates.SecondLocator(interval=30))
        plt.gcf().autofmt_xdate()

        plt.show()

def main():
    bag_path = '/home/jaiminsorathiya/EECE5554/gnss/data/moving.bag'
    plotter = AltitudeDataPlotter(bag_path)
    plotter.read_altitude_data()
    plotter.plot_altitude_vs_time()

if __name__ == '__main__':
    main()

