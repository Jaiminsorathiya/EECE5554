import rosbag
import matplotlib.pyplot as plt
import numpy as np

def read_altitude_time(bag_path, topic_name):
    times, altitudes = [], []
    
    with rosbag.Bag(bag_path, 'r') as bag:
        for _, msg, t in bag.read_messages(topics=[topic_name]):
            # Convert ROS Time to seconds
            time_sec = t.to_sec()
            times.append(time_sec)
            altitudes.append(msg.altitude)
    
    return times, altitudes

def normalize_times(times):
    min_time = min(times)
    return np.array(times) - min_time

def plot_altitudes(open_times, open_altitudes, occluded_times, occluded_altitudes):
    plt.figure(figsize=(12, 6))
    plt.plot(open_times, open_altitudes, 'b-', label='Open Area')
    plt.plot(occluded_times, occluded_altitudes, 'r-', label='Occluded Area')
    plt.xlabel('Time (s)')
    plt.ylabel('Altitude (m)')
    plt.title('Altitude vs. Time')
    plt.legend()
    plt.grid(True)
    plt.show()

def main():
    # Update the paths to your ROS bag files
    open_bag_path = '/home/jaiminsorathiya/EECE5554/gnss/data/open_RTK.bag'
    occluded_bag_path = '/home/jaiminsorathiya/EECE5554/gnss/data/occludedRTK.bag'
    topic_name = '/rtk_gps'

    # Read data
    open_times, open_altitudes = read_altitude_time(open_bag_path, topic_name)
    occluded_times, occluded_altitudes = read_altitude_time(occluded_bag_path, topic_name)

    # Normalize time to start at 0
    open_times = normalize_times(open_times)
    occluded_times = normalize_times(occluded_times)

    # Plot altitudes
    plot_altitudes(open_times, open_altitudes, occluded_times, occluded_altitudes)

if __name__ == "__main__":
    main()

