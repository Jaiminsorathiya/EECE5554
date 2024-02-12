import rosbag
import matplotlib.pyplot as plt
import numpy as np
import utm

def read_gps_data(bag_path, topic):
    eastings, northings = [], []

    with rosbag.Bag(bag_path, 'r') as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            e, n, _, _ = utm.from_latlon(msg.latitude, msg.longitude)
            eastings.append(e)
            northings.append(n)

    return np.array(eastings), np.array(northings)

def calculate_best_fit_line(eastings, northings):
    A = np.vstack([eastings, np.ones(len(eastings))]).T
    m, c = np.linalg.lstsq(A, northings, rcond=None)[0]
    return m, c

def plot_gps_data_with_best_fit(eastings, northings, m, c):
    plt.figure(figsize=(10, 6))
    plt.scatter(eastings, northings, color='blue', label='Walking Data')
    plt.plot(eastings, m * eastings + c, 'r', label=f'Best Fit Line: y = {m:.2f}x + {c:.2f}')
    plt.title('Moving Data: Northing vs Easting with Line of Best Fit')
    plt.xlabel('Easting (m)')
    plt.ylabel('Northing (m)')
    plt.legend()
    plt.grid(True)
    plt.show()

def main():
    # Adjust the bag_path and topic to match your ROS bag and topic name
    bag_path = '/home/jaiminsorathiya/EECE5554/gnss/data/walkingRTK.bag'
    topic = '/rtk_gps'  # Update this to your GPS topic name

    # Read and process GPS data
    eastings, northings = read_gps_data(bag_path, topic)

    # Calculate best fit line
    m, c = calculate_best_fit_line(eastings, northings)

    # Plot GPS data with best fit line
    plot_gps_data_with_best_fit(eastings, northings, m, c)

if __name__ == "__main__":
    main()

